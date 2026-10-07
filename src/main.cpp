// SDR++ source module for the RX888 mkII.
//
// Device control, firmware and the real-to-IQ DDC are ported from ExtIO_sddc
// by Oscar Steila IK1XPV and contributors (MIT); see THIRD_PARTY_NOTICES.md.
#include <imgui.h>
#include <module.h>
#include <gui/gui.h>
#include <gui/smgui.h>
#include <gui/style.h>
#include <gui/tuner.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <config.h>
#include <utils/flog.h>
#include <utils/optionlist.h>

#include "device/block_queue.h"
#include "device/firmware.h"
#include "device/fx3_device.h"
#include "device/rx888_mk2.h"
#include "device/tuning.h"
#include "dsp/r2iq.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "rx888_mkii_source",
    /* Description:     */ "RX888 mkII source (HF direct sampling and R828D VHF/UHF)",
    /* Author:          */ "Matt Mills (ExtIO_sddc port)",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

ConfigManager config;

using namespace rx888;
using Input = RX888mk2::Input;

namespace {

constexpr size_t USB_TRANSFER_SIZE = 131072; // 64 Ki samples, multiple of 16 KiB
constexpr int USB_TRANSFERS = 16;
constexpr size_t QUEUE_BLOCKS = 512;         // 64 MiB of slack

// ADC clocks offered. 16 and 20 MS/s fit a USB 2.0 High-Speed link (which
// tops out around 42 MB/s = 21 MS/s); the rest need USB 3.
const uint32_t ADC_RATES[] = { 16000000, 20000000, 32000000, 64000000, 128000000 };
constexpr double USB2_MAX_RATE = 21e6;

// Widest output allowed on the VHF path: the R828D IF filter is ~8 MHz wide
// around a 4.57 MHz IF.
constexpr double VHF_MAX_OUTPUT = 10e6;

std::string rateLabel(double hz) {
    char buf[64];
    if (hz >= 1e6) { snprintf(buf, sizeof(buf), "%g MHz", hz / 1e6); }
    else { snprintf(buf, sizeof(buf), "%g kHz", hz / 1e3); }
    return buf;
}

} // namespace

class RX888SourceModule : public ModuleManager::Instance {
public:
    RX888SourceModule(std::string name) : name(name) {
        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;

        for (auto r : ADC_RATES) { adcRates.define(r, rateLabel(r) + "  (" + rateLabel(r / 2.0) + " HF span)", r); }
        inputs.define("hf", "HF  (direct sampling)", Input::HF);
        inputs.define("vhf", "VHF/UHF  (R828D tuner)", Input::VHF);

        refresh();
        config.acquire();
        std::string dev = config.conf["device"];
        config.release();
        selectDevice(dev);

        sigpath::sourceManager.registerSource("RX888 mkII", &handler);
    }

    ~RX888SourceModule() {
        stop(this);
        sigpath::sourceManager.unregisterSource("RX888 mkII");
    }

    void postInit() {}
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    bool isEnabled() { return enabled; }

private:
    // ------------------------------------------------------------ devices
    void refresh() {
        devices.clear();
        auto list = Fx3Device::enumerate();
        for (const auto& d : list) {
            std::string label = "RX888 mkII [" + (d.serial.empty() ? d.path : d.serial) + "]";
            if (d.bootloader) { label += " (bootloader)"; }
            if (!d.accessible) { label += " (no driver)"; }
            if (!devices.keyExists(d.path)) { devices.define(d.path, label, d.path); }
        }
    }

    void selectDevice(const std::string& path) {
        statusText.clear();
        if (devices.empty()) {
            selectedPath.clear();
            statusText = "No RX888 found. Check the USB cable and the WinUSB driver (Zadig).";
            return;
        }
        std::string p = devices.keyExists(path) ? path : devices.key(0);

        // Load the firmware if the FX3 is still in its ROM bootloader. The
        // device re-enumerates, possibly on a different port path.
        for (const auto& d : Fx3Device::enumerate()) {
            if (d.path != p || !d.bootloader) { continue; }
            if (!d.accessible) {
                statusText = "Bootloader (04B4:00F3) has no WinUSB driver: install it with Zadig.";
                selectedPath.clear();
                return;
            }
            std::string err, newPath;
            flog::info("RX888: uploading firmware to {}", p);
            if (!Fx3Device::loadFirmware(p, sddc_fx3_firmware, sddc_fx3_firmware_size, newPath, err)) {
                statusText = "Firmware upload failed: " + err;
                flog::error("RX888: {}", statusText);
                selectedPath.clear();
                refresh();
                return;
            }
            refresh();
            p = newPath;
        }

        // Probe the device.
        RX888mk2 probe;
        std::string err;
        if (!probe.open(p, err)) {
            statusText = "Cannot open device: " + err;
            flog::error("RX888: {}", statusText);
            selectedPath.clear();
            return;
        }
        model = probe.model();
        fwVersion = probe.firmwareVersion();
        usbSpeed = probe.device().speed();
        probe.close();
        if (model != MODEL_RX888R2) {
            flog::warn("RX888: device reports model {} ({}), this module targets the RX888 mkII", (int)model, modelName(model));
        }
        if (fwVersion != 0x0202) {
            // A different SDDC firmware was loaded by other software; unplug
            // the receiver to return it to the bootloader so ours is loaded.
            char ver[16];
            snprintf(ver, sizeof(ver), "%d.%02d", fwVersion >> 8, fwVersion & 0xFF);
            flog::warn("RX888: firmware {} is running, this module was tested with 2.02", std::string(ver));
        }

        selectedPath = p;
        devId = devices.keyId(p);
        superSpeed = usbSpeed >= 4; // LIBUSB_SPEED_SUPER
        // Settings are per device; without a serial (USB 2.0 descriptors have
        // none) fall back to one shared entry.
        devKey = "default";
        for (const auto& d : Fx3Device::enumerate()) {
            if (d.path == p && !d.serial.empty()) { devKey = d.serial; }
        }
        loadSettings();
        updateRateList();

        config.acquire();
        config.conf["device"] = selectedPath;
        config.release(true);
    }

    void loadSettings() {
        config.acquire();
        json& c = config.conf["devices"][devKey];
        auto get = [&](const char* k, auto def) {
            if (!c.contains(k)) { c[k] = def; }
            return c[k].get<decltype(def)>();
        };
        std::string in = get("input", std::string("hf"));
        input = (in == "vhf") ? Input::VHF : Input::HF;
        adcRate = get("adcRate", (uint32_t)(superSpeed ? 64000000 : 20000000));
        if (!adcRates.keyExists(adcRate)) { adcRate = superSpeed ? 64000000 : 20000000; }
        hfOutRate = get("hfOutRate", 2e6);
        vhfOutRate = get("vhfOutRate", 2.5e6);
        hfAtt = get("hfAtt", 0);
        hfVga = get("hfVga", 30);
        vhfRf = get("vhfRfGain", 12);
        vhfIf = get("vhfIfGain", 6);
        vhfVga = get("vhfIfVga", 18);
        adcHighGain = get("adcPga", true);
        dither = get("dither", false);
        randomizer = get("randomizer", false);
        dcRemoval = get("dcRemoval", true);
        biasHf = get("biasTeeHf", false);
        biasVhf = get("biasTeeVhf", false);
        ppb = get("ppb", 0);
        lastFreq[0] = get("lastFreqHf", 7.1e6);
        lastFreq[1] = get("lastFreqVhf", 100e6);
        config.release(true);
        inputId = inputs.valueId(input);
        adcId = adcRates.keyId(adcRate);
    }

    template <typename T>
    void save(const char* key, T value) {
        if (devKey.empty()) { return; }
        config.acquire();
        config.conf["devices"][devKey][key] = value;
        config.release(true);
    }

    // Output rates for the current ADC clock and input: fs/2 / 2^d.
    void updateRateList() {
        outRates.clear();
        for (int d = 0; d <= R2IQ::MAX_DECIM; d++) {
            double r = adcRate / (double)(2 << d);
            if (input == Input::VHF && r > VHF_MAX_OUTPUT) { continue; }
            outRates.define(d, rateLabel(r), r);
        }
        double want = (input == Input::HF) ? hfOutRate : vhfOutRate;
        // Pick the closest available rate.
        rateId = 0;
        for (int i = 0; i < outRates.size(); i++) {
            if (std::fabs(outRates.value(i) - want) < std::fabs(outRates.value(rateId) - want)) { rateId = i; }
        }
        decim = outRates.key(rateId);
        sampleRate = outRates.value(rateId);
    }

    // Output rate in the corrected frequency frame: the Si5351 synthesis error
    // and the ppb correction scale the real ADC clock. Giving SDR++ this rate
    // (instead of the round nominal one) keeps its frequency axis exact away
    // from the centre too.
    double effectiveRate() const {
        return RX888mk2::si5351OutputHz(adcRate) * (1.0 + ppb * 1e-9) / (double)(2 << decim);
    }

    // ------------------------------------------------------------ handlers
    static void menuSelected(void* ctx) {
        RX888SourceModule* _this = (RX888SourceModule*)ctx;
        core::setInputSampleRate(_this->effectiveRate());
        flog::info("RX888SourceModule '{}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        RX888SourceModule* _this = (RX888SourceModule*)ctx;
        flog::info("RX888SourceModule '{}': Menu Deselect!", _this->name);
    }

    static void start(void* ctx) {
        RX888SourceModule* _this = (RX888SourceModule*)ctx;
        if (_this->running) { return; }
        if (_this->selectedPath.empty()) {
            _this->refresh();
            _this->selectDevice("");
            if (_this->selectedPath.empty()) {
                flog::error("RX888: no device selected");
                return;
            }
        }
        std::string err;
        if (!_this->radio.open(_this->selectedPath, err)) {
            // The device may have been replugged (back in bootloader mode).
            _this->refresh();
            _this->selectDevice(_this->selectedPath);
            if (_this->selectedPath.empty() || !_this->radio.open(_this->selectedPath, err)) {
                _this->statusText = "Cannot open device: " + err;
                flog::error("RX888: {}", _this->statusText);
                return;
            }
        }
        _this->statusText.clear();
        _this->usbSpeed = _this->radio.device().speed();

        RX888mk2& r = _this->radio;
        r.setAdcRate(_this->adcRate);
        r.setInput(_this->input);
        _this->applyAllControls();

        // One core keeps up with 128 MS/s when FFTW has SIMD; extra threads
        // leave headroom for SDR++ itself at the high clocks.
        int hw = (int)std::thread::hardware_concurrency();
        int threads = _this->adcRate >= 100000000 ? 3 : (_this->adcRate >= 50000000 ? 2 : 1);
        _this->r2iq.setThreads(std::clamp(threads, 1, std::max(1, hw / 2)));
        _this->r2iq.configure(_this->decim);
        _this->r2iq.reset();
        _this->r2iq.setDcRemoval(_this->dcRemoval);
        _this->applyTuning();

        _this->queue.init(USB_TRANSFER_SIZE, QUEUE_BLOCKS);
        _this->usbBytes = 0;
        _this->workerRun = true;
        _this->worker = std::thread(&RX888SourceModule::dspWorker, _this);

        r.startAdc();
        if (!r.device().startStreaming(USB_TRANSFER_SIZE, USB_TRANSFERS, [_this](const uint8_t* d, size_t n) {
                _this->usbBytes += n;
                _this->queue.push(d, n);
            }, err)) {
            _this->statusText = "USB streaming failed: " + err;
            flog::error("RX888: {}", _this->statusText);
            _this->workerRun = false;
            _this->queue.stop();
            if (_this->worker.joinable()) { _this->worker.join(); }
            r.close();
            return;
        }
        _this->statsTime = std::chrono::steady_clock::now();
        _this->statsBytes = 0;
        _this->running = true;
        flog::info("RX888SourceModule '{}': Start! ADC {} MS/s, output {} MS/s, {}", _this->name, _this->adcRate / 1e6,
                   _this->sampleRate / 1e6, _this->input == Input::HF ? "HF" : "VHF");
    }

    static void stop(void* ctx) {
        RX888SourceModule* _this = (RX888SourceModule*)ctx;
        if (!_this->running) { return; }
        _this->running = false;
        _this->stream.stopWriter();
        _this->radio.device().stopStreaming();
        _this->radio.stopAdc();
        _this->workerRun = false;
        _this->queue.stop();
        if (_this->worker.joinable()) { _this->worker.join(); }
        _this->stream.clearWriteStop();
        _this->radio.close();
        flog::info("RX888SourceModule '{}': Stop!", _this->name);
    }

    static void tune(double freq, void* ctx) {
        RX888SourceModule* _this = (RX888SourceModule*)ctx;
        _this->freq = freq;
        if (_this->running) { _this->applyTuning(); }
        int m = (_this->input == Input::HF) ? 0 : 1;
        _this->lastFreq[m] = freq;
    }

    // Restart the stream after a change that alters the sample rate or path.
    void restartIfRunning(bool rateChanged) {
        bool wasRunning = running;
        if (wasRunning) { stop(this); }
        if (rateChanged) { core::setInputSampleRate(effectiveRate()); }
        if (wasRunning) { start(this); }
    }

    void applyTuning() {
        const double ppm = ppb / 1000.0;
        if (input == Input::HF) {
            tuning = computeTuning(input, freq, adcRate, ppm);
        }
        else {
            // Keep the tuner in range; the DDC still follows the requested
            // frequency so the frequency scale stays right (the band is empty).
            double f = std::clamp(freq, (double)RX888mk2::VHF_MIN_HZ, (double)RX888mk2::VHF_MAX_HZ);
            tuning = computeTuning(input, f, adcRate, ppm);
            tuning.tuneHz -= freq - f;
            if (radio.isOpen()) { radio.tune(tuning.tunerHz); }
        }
        r2iq.setTuning(tuning.tuneHz, tuning.adcHz, tuning.invert);
    }

    void applyAllControls() {
        RX888mk2& r = radio;
        r.setAdcHighGain(adcHighGain);
        r.setDither(dither);
        r.setRandomizer(randomizer);
        if (input == Input::HF) {
            r.setHfAttenuation(hfAtt);
            r.setHfVga(hfVga);
            r.setBiasTeeHf(biasHf);
            r.setBiasTeeVhf(false);
        }
        else {
            r.setVhfRfGain(vhfRf);
            r.setVhfIfGain(vhfIf);
            r.setIfVgaCode(RX888mk2::hfVgaCode(vhfVga));
            r.setBiasTeeVhf(biasVhf);
            r.setBiasTeeHf(false);
        }
    }

    // ------------------------------------------------------------ DSP thread
    void dspWorker() {
        // Hand ~10 ms of output to SDR++ at a time (never more than the
        // stream buffer holds).
        const size_t maxPerBlock = r2iq.maxOutput(USB_TRANSFER_SIZE / 2);
        size_t chunk = (size_t)std::clamp(sampleRate / 100.0, 4096.0, (double)(STREAM_BUFFER_SIZE - maxPerBlock));
        size_t fill = 0;
        while (workerRun) {
            BlockQueue::Block* b = queue.front();
            if (!b) { continue; }
            auto* out = (R2IQ::cf*)stream.writeBuf + fill;
            fill += r2iq.process((const int16_t*)b->data.data(), b->len / 2, randomizer, out);
            queue.pop();
            if (fill >= chunk) {
                if (!stream.swap((int)fill)) { break; }
                fill = 0;
            }
        }
    }

    // ------------------------------------------------------------ UI
    static void menuHandler(void* ctx) {
        RX888SourceModule* _this = (RX888SourceModule*)ctx;
        const std::string& n = _this->name;

        // Device selection
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_rx888_dev_", n), &_this->devId, _this->devices.txt)) {
            bool wasRunning = _this->running;
            if (wasRunning) { stop(_this); }
            _this->selectDevice(_this->devices.key(_this->devId));
            core::setInputSampleRate(_this->effectiveRate());
            if (wasRunning) { start(_this); }
        }
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Button(CONCAT("Refresh##_rx888_refr_", n))) {
            if (!_this->running) {
                _this->refresh();
                _this->selectDevice(_this->selectedPath);
                core::setInputSampleRate(_this->effectiveRate());
            }
        }

        if (!_this->statusText.empty()) {
            SmGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), _this->statusText.c_str());
        }
        if (_this->selectedPath.empty()) { return; }

        char buf[256];
        snprintf(buf, sizeof(buf), "%s, firmware %d.%02d, %s", modelName(_this->model), _this->fwVersion >> 8, _this->fwVersion & 0xFF,
                 _this->superSpeed ? "USB 3" : "USB 2.0");
        SmGui::Text(buf);
        if (!_this->superSpeed && _this->adcRate > USB2_MAX_RATE) {
            SmGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "USB 2.0 link: ADC clocks above ~20 MS/s lose samples.");
        }

        // Input path
        SmGui::LeftLabel("Input");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_rx888_input_", n), &_this->inputId, _this->inputs.txt)) {
            Input prev = _this->input;
            _this->input = _this->inputs.value(_this->inputId);
            _this->save("input", _this->inputs.key(_this->inputId));
            _this->updateRateList();
            _this->restartIfRunning(true);
            // Move to a frequency that exists on the new path.
            int m = (_this->input == Input::HF) ? 0 : 1;
            bool outOfRange = (_this->input == Input::HF) ? (_this->freq > _this->adcRate / 2.0)
                                                          : (_this->freq < RX888mk2::VHF_MIN_HZ || _this->freq > RX888mk2::VHF_MAX_HZ);
            if (prev != _this->input && outOfRange) { tuner::centerTuning(gui::waterfall.selectedVFO, _this->lastFreq[m]); }
        }

        SmGui::LeftLabel("ADC clock");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_rx888_adc_", n), &_this->adcId, _this->adcRates.txt)) {
            _this->adcRate = _this->adcRates.key(_this->adcId);
            _this->save("adcRate", _this->adcRate);
            _this->updateRateList();
            _this->restartIfRunning(true);
        }

        SmGui::LeftLabel("Bandwidth");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_rx888_sr_", n), &_this->rateId, _this->outRates.txt)) {
            _this->decim = _this->outRates.key(_this->rateId);
            _this->sampleRate = _this->outRates.value(_this->rateId);
            if (_this->input == Input::HF) {
                _this->hfOutRate = _this->sampleRate;
                _this->save("hfOutRate", _this->hfOutRate);
            }
            else {
                _this->vhfOutRate = _this->sampleRate;
                _this->save("vhfOutRate", _this->vhfOutRate);
            }
            _this->restartIfRunning(true);
        }

        RX888mk2& r = _this->radio;
        const bool live = _this->running;
        if (_this->input == Input::HF) {
            snprintf(buf, sizeof(buf), "HF range: 0 - %.1f MHz", _this->adcRate / 2e6);
            SmGui::Text(buf);

            SmGui::LeftLabel("RF atten.");
            SmGui::FillWidth();
            float att = _this->hfAtt * 0.5f;
            if (SmGui::SliderFloatWithSteps(CONCAT("##_rx888_att_", n), &att, 0.0f, 31.5f, 0.5f, SmGui::FMT_STR_FLOAT_DB_ONE_DECIMAL)) {
                _this->hfAtt = (int)std::lround(att * 2.0f);
                if (live) { r.setHfAttenuation(_this->hfAtt); }
                _this->save("hfAtt", _this->hfAtt);
            }

            snprintf(buf, sizeof(buf), "IF gain %+5.1f dB", RX888mk2::hfVgaDb(_this->hfVga));
            SmGui::LeftLabel(buf);
            SmGui::FillWidth();
            if (SmGui::SliderInt(CONCAT("##_rx888_vga_", n), &_this->hfVga, 0, RX888mk2::HF_VGA_STEPS - 1, SmGui::FMT_STR_NONE)) {
                if (live) { r.setHfVga(_this->hfVga); }
                _this->save("hfVga", _this->hfVga);
            }

            if (SmGui::Checkbox(CONCAT("Bias-T (HF)##_rx888_bhf_", n), &_this->biasHf)) {
                if (live) { r.setBiasTeeHf(_this->biasHf); }
                _this->save("biasTeeHf", _this->biasHf);
            }
        }
        else {
            snprintf(buf, sizeof(buf), "Tuner range: %d - %d MHz (IF %.2f MHz)", (int)(RX888mk2::VHF_MIN_HZ / 1000000),
                     (int)(RX888mk2::VHF_MAX_HZ / 1000000), RX888mk2::R828D_IF_HZ / 1e6);
            SmGui::Text(buf);

            snprintf(buf, sizeof(buf), "LNA/Mixer %+5.1f dB", RX888mk2::vhfRfGainDb(_this->vhfRf));
            SmGui::LeftLabel(buf);
            SmGui::FillWidth();
            if (SmGui::SliderInt(CONCAT("##_rx888_vrf_", n), &_this->vhfRf, 0, RX888mk2::VHF_RF_STEPS - 1, SmGui::FMT_STR_NONE)) {
                if (live) { r.setVhfRfGain(_this->vhfRf); }
                _this->save("vhfRfGain", _this->vhfRf);
            }

            snprintf(buf, sizeof(buf), "Tuner IF %+5.1f dB", RX888mk2::vhfIfGainDb(_this->vhfIf));
            SmGui::LeftLabel(buf);
            SmGui::FillWidth();
            if (SmGui::SliderInt(CONCAT("##_rx888_vif_", n), &_this->vhfIf, 0, RX888mk2::VHF_IF_STEPS - 1, SmGui::FMT_STR_NONE)) {
                if (live) { r.setVhfIfGain(_this->vhfIf); }
                _this->save("vhfIfGain", _this->vhfIf);
            }

            snprintf(buf, sizeof(buf), "IF VGA %+5.1f dB", RX888mk2::hfVgaDb(_this->vhfVga));
            SmGui::LeftLabel(buf);
            SmGui::FillWidth();
            if (SmGui::SliderInt(CONCAT("##_rx888_vvga_", n), &_this->vhfVga, 0, RX888mk2::HF_VGA_STEPS - 1, SmGui::FMT_STR_NONE)) {
                if (live) { r.setIfVgaCode(RX888mk2::hfVgaCode(_this->vhfVga)); }
                _this->save("vhfIfVga", _this->vhfVga);
            }

            if (SmGui::Checkbox(CONCAT("Bias-T (VHF)##_rx888_bvhf_", n), &_this->biasVhf)) {
                if (live) { r.setBiasTeeVhf(_this->biasVhf); }
                _this->save("biasTeeVhf", _this->biasVhf);
            }
            if (_this->freq < RX888mk2::VHF_MIN_HZ || _this->freq > RX888mk2::VHF_MAX_HZ) {
                SmGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Frequency outside the tuner range");
            }
        }

        // ADC options (both paths)
        if (SmGui::Checkbox(CONCAT("ADC PGA +3.5 dB (1.5 Vpp)##_rx888_pga_", n), &_this->adcHighGain)) {
            if (live) { r.setAdcHighGain(_this->adcHighGain); }
            _this->save("adcPga", _this->adcHighGain);
        }
        if (SmGui::Checkbox(CONCAT("ADC dither##_rx888_dith_", n), &_this->dither)) {
            if (live) { r.setDither(_this->dither); }
            _this->save("dither", _this->dither);
        }
        if (SmGui::Checkbox(CONCAT("ADC randomizer##_rx888_rand_", n), &_this->randomizer)) {
            // Hardware first, so the decoder never sees plain data as randomized
            // for long; a few ms of garbage at the switch is unavoidable.
            if (live) { r.setRandomizer(_this->randomizer); }
            _this->save("randomizer", _this->randomizer);
        }
        if (SmGui::Checkbox(CONCAT("Remove ADC DC offset##_rx888_dc_", n), &_this->dcRemoval)) {
            _this->r2iq.setDcRemoval(_this->dcRemoval);
            _this->save("dcRemoval", _this->dcRemoval);
        }

        SmGui::LeftLabel("Freq. corr. (ppb)");
        SmGui::FillWidth();
        if (SmGui::InputInt(CONCAT("##_rx888_ppb_", n), &_this->ppb, 100, 1000)) {
            _this->ppb = std::clamp(_this->ppb, -200000, 200000);
            _this->save("ppb", _this->ppb);
            core::setInputSampleRate(_this->effectiveRate());
            if (live) { _this->applyTuning(); }
        }

        if (live) { _this->drawStats(); }
    }

    void drawStats() {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - statsTime).count();
        if (dt >= 1.0) {
            uint64_t b = usbBytes;
            usbRate = (b - statsBytes) / dt / 2.0;
            statsBytes = b;
            statsTime = now;
            int pk;
            uint64_t clip;
            r2iq.takeStats(pk, clip);
            adcPeakDb = 20.0 * log10((pk + 0.5) / 32768.0);
            clipped = clip;
            dropped = queue.droppedCount();
            usbErrors = radio.device().transferErrors();
        }
        char buf[256];
        bool slow = usbRate > 0 && usbRate < adcRate * 0.98;
        snprintf(buf, sizeof(buf), "USB: %.2f MS/s of %.2f", usbRate / 1e6, adcRate / 1e6);
        if (slow) { SmGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), buf); }
        else { SmGui::Text(buf); }
        if (dropped || usbErrors) {
            snprintf(buf, sizeof(buf), "Dropped blocks: %llu, USB errors: %llu", (unsigned long long)dropped, (unsigned long long)usbErrors);
            SmGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), buf);
        }
        if (clipped) { snprintf(buf, sizeof(buf), "ADC peak: %.1f dBFS, CLIPPING (%llu samples/s): reduce gain", adcPeakDb, (unsigned long long)clipped); }
        else { snprintf(buf, sizeof(buf), "ADC peak: %.1f dBFS", adcPeakDb); }
        if (clipped) { SmGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), buf); }
        else { SmGui::Text(buf); }
    }

    // ------------------------------------------------------------ state
    std::string name;
    bool enabled = true;
    SourceManager::SourceHandler handler;
    dsp::stream<dsp::complex_t> stream;

    OptionList<std::string, std::string> devices;
    OptionList<uint32_t, uint32_t> adcRates;
    OptionList<std::string, Input> inputs;
    OptionList<int, double> outRates;
    int devId = 0, adcId = 0, inputId = 0, rateId = 0;

    std::string selectedPath, devKey, statusText;
    uint8_t model = 0;
    uint16_t fwVersion = 0;
    int usbSpeed = 0;
    bool superSpeed = false;

    // Settings
    Input input = Input::HF;
    uint32_t adcRate = 64000000;
    double hfOutRate = 2e6, vhfOutRate = 2.5e6;
    int decim = 4;
    double sampleRate = 2e6;
    int hfAtt = 0, hfVga = 30;
    int vhfRf = 12, vhfIf = 6, vhfVga = 18;
    bool adcHighGain = true, dither = false, randomizer = false, dcRemoval = true;
    bool biasHf = false, biasVhf = false;
    int ppb = 0;
    double freq = 7.1e6;
    double lastFreq[2] = { 7.1e6, 100e6 };

    // Runtime
    RX888mk2 radio;
    R2IQ r2iq;
    DdcTuning tuning;
    BlockQueue queue;
    std::thread worker;
    std::atomic<bool> workerRun{ false };
    std::atomic<bool> running{ false };
    std::atomic<uint64_t> usbBytes{ 0 };

    // Stats
    std::chrono::steady_clock::time_point statsTime;
    uint64_t statsBytes = 0;
    double usbRate = 0, adcPeakDb = -100;
    uint64_t clipped = 0, dropped = 0, usbErrors = 0;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    def["devices"] = json({});
    def["device"] = "";
    config.setPath(core::args["root"].s() + "/rx888_mkii_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new RX888SourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (RX888SourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
