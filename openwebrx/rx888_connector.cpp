// rx888_connector: OpenWebRX connector for the RX888 mkII, built on this
// repository's device and DSP code (rx888_core) and the owrx_connector
// framework, which provides the command line, the IQ socket (complex float32)
// and the control socket OpenWebRX drives.
//
// HF (centre <= 64 MHz) samples the HF input directly. The ADC runs at the
// highest rate <= 128 MS/s for which samp_rate = adc / (2 << decim) exactly,
// and the fast-convolution DDC (R2IQ) tunes and decimates on the CPU. The
// widest setting, samp_rate 64 MS/s centred at 32 MHz, shows 0-64 MHz at
// once. Above 64 MHz the R828D tuner is used with the ADC at <= 64 MS/s.
//
// The ADC clock is requested at nominal / (1 + ppm), so with the crystal error
// passed as ppm the delivered sample rate is the nominal one. OpenWebRX
// places every signal by that rate: 10 ppm of rate error would put a signal
// 30 MHz from the centre 300 Hz off.
//
// Installed under the name sddc_connector too, it serves OpenWebRX's existing
// "SDDC" device type with no change to OpenWebRX.
#include "device/block_queue.h"
#include "device/firmware.h"
#include "device/fx3_device.h"
#include "device/rx888_mk2.h"
#include "device/tuning.h"
#include "dsp/r2iq.h"
#include <owrx/connector.hpp>
#include <owrx/gainspec.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef RX888_CONNECTOR_VERSION
#define RX888_CONNECTOR_VERSION "0.1.0"
#endif

using namespace rx888;

namespace {

constexpr double HF_MAX_HZ = 64e6;    // above this the R828D tuner is used
constexpr double ADC_MAX_SS = 128e6;  // SuperSpeed
constexpr double ADC_MAX_HS = 20e6;   // USB 2.0 High Speed sustains about 20 MS/s
constexpr double ADC_MAX_VHF = 64e6;
constexpr double ADC_MIN = 8e6;
constexpr size_t XFER = 131072;       // bytes per USB transfer
constexpr int HF_VGA_DEFAULT = 30;
constexpr int VHF_RF_DEFAULT = 16;    // best measured on ATSC ch18; 28 overloads
constexpr int VHF_IF_DEFAULT = 8;

std::string programName = "rx888_connector";

} // namespace

class Rx888Connector : public Owrx::Connector {
public:
    void print_version() override {
        std::cout << programName << " version " << RX888_CONNECTOR_VERSION << std::endl;
        Connector::print_version();
    }

protected:
    // floats per processSamples() call at most; the IQ ring holds ten of these
    uint32_t get_buffer_size() override { return 1u << 22; }

    std::stringstream get_usage_string() override {
        std::stringstream s = Connector::get_usage_string();
        s << "\nRX888 mkII notes:\n"
             "  centre <= 64 MHz: HF direct sampling; samp_rate = adc / 2^k, adc <= 128 MS/s (widest: -s 64000000 -f 32000000)\n"
             "  centre  > 64 MHz: R828D tuner\n"
             "  gain: dB (HF: AD8370 VGA gain, below its minimum the step attenuator; VHF: R828D LNA gain),\n"
             "        'auto' for the defaults, or att=DB,vga=INDEX (HF) / rf=INDEX,if=INDEX (VHF)\n"
             "  -d: device index, USB path or serial\n";
        return s;
    }

    int open() override {
        auto devs = Fx3Device::enumerate();
        if (devs.empty()) {
            std::cerr << "ERROR: no RX888 / FX3 device found" << std::endl;
            return 1;
        }
        const UsbDeviceInfo* pick = &devs[0];
        if (device_id != nullptr) {
            std::string id = device_id;
            pick = nullptr;
            char* end = nullptr;
            long idx = std::strtol(id.c_str(), &end, 10);
            if (end && *end == '\0' && idx >= 0 && (size_t)idx < devs.size()) { pick = &devs[idx]; }
            for (auto& d : devs) {
                if (!pick && (d.path == id || d.serial == id)) { pick = &d; }
            }
            if (!pick) {
                std::cerr << "ERROR: no device matches \"" << id << "\"" << std::endl;
                return 1;
            }
        }
        std::string path = pick->path, err;
        if (pick->bootloader) {
            std::cerr << "device " << path << " is in bootloader mode, uploading the embedded firmware" << std::endl;
            std::string newPath;
            if (!Fx3Device::loadFirmware(path, sddc_fx3_firmware, sddc_fx3_firmware_size, newPath, err)) {
                std::cerr << "ERROR: firmware upload failed: " << err << std::endl;
                return 1;
            }
            path = newPath;
        }
        // After an abrupt end of a previous session (a killed process, a container
        // restart mid-stream) the FX3 can answer the first requests and then time
        // out every control request (3 s each) for some 20 s. Reopen rather than
        // queue timeouts, and give up after a few tries.
        for (int attempt = 1;; attempt++) {
            if (!radio.open(path, err)) {
                std::cerr << "ERROR: could not open " << path << ": " << err << std::endl;
                return 1;
            }
            if (radio.setRandomizer(false) && radio.setDither(false) && radio.setAdcHighGain(true)) { break; }
            std::cerr << "device not answering control requests (" << radio.lastError() << "), reopening" << std::endl;
            radio.close();
            if (attempt == 5 || !run) { return 1; }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        superSpeed = radio.device().speed() >= 4; // LIBUSB_SPEED_SUPER
        std::cerr << "opened " << path << ": model 0x" << std::hex << (int)radio.model() << std::dec << ", firmware "
                  << (radio.firmwareVersion() >> 8) << "." << (radio.firmwareVersion() & 0xFF) << ", "
                  << usbSpeedName(radio.device().speed()) << std::endl;
        int hw = (int)std::thread::hardware_concurrency();
        r2iq.setThreads(std::clamp(hw / 4, 1, 6)); // 6 decode 128 MS/s at decimation 0 with headroom
        adcReq = 0;
        decim = -1;
        modeSet = false;
        return 0;
    }

    int read() override {
        failed = false;
        queue.init(XFER, 1024);
        dspRun = true;
        dsp = std::thread([this] { dspLoop(); });
        {
            std::lock_guard<std::mutex> lk(ctl);
            active = true;
            if (apply() != 0) { failed = true; }
        }
        auto last = std::chrono::steady_clock::now();
        while (run && !failed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            auto now = std::chrono::steady_clock::now();
            if (now - last >= std::chrono::seconds(30)) {
                last = now;
                int peak = 0;
                uint64_t clipped = 0;
                r2iq.takeStats(peak, clipped);
                std::cerr << "rx888: ADC peak " << peak << "/32767, " << clipped << " samples near full scale, "
                          << queue.droppedCount() << " blocks dropped, " << radio.device().transferErrors() << " USB errors"
                          << std::endl;
            }
        }
        {
            std::lock_guard<std::mutex> lk(ctl);
            std::lock_guard<std::mutex> dl(dspMtx);
            active = false;
            stopStream();
        }
        dspRun = false;
        queue.stop();
        dsp.join();
        return failed ? 1 : 0;
    }

    int close() override {
        radio.close();
        return 0;
    }

    int set_center_frequency(double frequency) override {
        std::lock_guard<std::mutex> lk(ctl);
        centre = frequency;
        return apply();
    }

    int set_sample_rate(double sample_rate) override {
        std::lock_guard<std::mutex> lk(ctl);
        rate = sample_rate;
        return apply();
    }

    int set_ppm(double new_ppm) override {
        std::lock_guard<std::mutex> lk(ctl);
        // the base class leaves ppm uninitialised when -P is not given
        ppm = (std::isfinite(new_ppm) && std::fabs(new_ppm) < 1000) ? new_ppm : 0.0;
        return apply();
    }

    int set_gain(Owrx::GainSpec* gain) override {
        std::lock_guard<std::mutex> lk(ctl);
        return (active && modeSet) ? applyGain(gain) : 0;
    }

private:
    // Plan the ADC rate and decimation for (centre, rate, ppm) and push it to
    // the hardware. Before read() it only records the values. Caller holds ctl.
    int apply() {
        if (!active || !radio.isOpen() || !run) { return 0; }
        if (rate <= 0) {
            std::cerr << "ERROR: no sample rate set" << std::endl;
            return 3;
        }
        const bool vhf = centre > HF_MAX_HZ;
        double adcMax = vhf ? ADC_MAX_VHF : ADC_MAX_SS;
        if (!superSpeed) { adcMax = std::min(adcMax, ADC_MAX_HS); }
        int d = -1;
        for (int k = R2IQ::MAX_DECIM; k >= 0; k--) {
            if (rate * (2 << k) <= adcMax + 0.5) {
                d = k;
                break;
            }
        }
        if (d < 0 || rate * (2 << d) < ADC_MIN) {
            std::cerr << "ERROR: sample rate " << rate << " is not reachable: it must be adc / 2^k with " << ADC_MIN / 1e6
                      << " MS/s <= adc <= " << adcMax / 1e6 << " MS/s" << std::endl;
            return 3;
        }
        const double adcNominal = rate * (2 << d);
        const uint32_t req = (uint32_t)std::llround(adcNominal / (1.0 + ppm * 1e-6));
        if (!vhf && centre + rate / 2 > adcNominal / 2 + 1) {
            std::cerr << "warning: the band reaches above the ADC Nyquist frequency (" << adcNominal / 2e6 << " MHz)" << std::endl;
        }

        std::lock_guard<std::mutex> dl(dspMtx);
        const bool modeChange = !modeSet || vhf != curVhf;
        const bool restart = modeChange || req != adcReq;
        if (restart && streaming) { stopStream(); }
        // Each failed control request costs a 3 s USB timeout: stop at the first
        // one, and between steps when the process has been asked to exit.
        if (modeChange) {
            if (!radio.setInput(vhf ? RX888mk2::Input::VHF : RX888mk2::Input::HF)) {
                std::cerr << "ERROR: switching the input failed: " << radio.lastError() << std::endl;
                return 2;
            }
            curVhf = vhf;
            modeSet = true;
            if (applyGain(get_gain()) != 0) {
                std::cerr << "ERROR: setting the gain failed: " << radio.lastError() << std::endl;
                return 2;
            }
        }
        if (!run) { return 0; }
        if (req != adcReq) {
            if (!radio.setAdcRate(req)) {
                std::cerr << "ERROR: setting the ADC rate failed: " << radio.lastError() << std::endl;
                return 3;
            }
            adcReq = req;
        }
        DdcTuning tn = computeTuning(vhf ? RX888mk2::Input::VHF : RX888mk2::Input::HF, centre, req, ppm);
        if (vhf && !radio.tune(tn.tunerHz)) {
            std::cerr << "ERROR: tuning the R828D failed" << std::endl;
            return 2;
        }
        if (d != decim) {
            r2iq.configure(d);
            decim = d;
        }
        if (restart) { r2iq.reset(); }
        r2iq.setTuning(tn.tuneHz, tn.adcHz, tn.invert);
        if (!streaming && !startStream()) { return 2; }
        const double actual = tn.adcHz / (2 << d);
        std::cerr << (vhf ? "VHF" : "HF") << " centre " << centre / 1e6 << " MHz, " << rate / 1e6 << " MS/s = ADC "
                  << req / 1e6 << " MHz / " << (2 << d) << "; delivered rate off nominal by " << (actual / rate - 1) * 1e6
                  << " ppm" << std::endl;
        return 0;
    }

    int applyGain(Owrx::GainSpec* gain) {
        auto* simple = dynamic_cast<Owrx::SimpleGainSpec*>(gain);
        auto* multi = dynamic_cast<Owrx::MultiGainSpec*>(gain);
        bool ok = true;
        if (!curVhf) {
            int att = 0, vga = HF_VGA_DEFAULT;
            if (simple) {
                const float g = simple->getValue();
                if (g >= RX888mk2::hfVgaDb(0)) {
                    for (int i = 0; i < RX888mk2::HF_VGA_STEPS; i++) {
                        if (std::fabs(RX888mk2::hfVgaDb(i) - g) < std::fabs(RX888mk2::hfVgaDb(vga) - g)) { vga = i; }
                    }
                }
                else {
                    vga = 0;
                    att = (int)std::lround((RX888mk2::hfVgaDb(0) - g) * 2);
                }
            }
            else if (multi) {
                auto m = multi->getValue();
                if (m.count("att")) { att = (int)std::lround(std::stod(m["att"]) * 2); }
                if (m.count("vga")) { vga = std::stoi(m["vga"]); }
            }
            att = std::clamp(att, 0, RX888mk2::HF_ATT_STEPS - 1);
            vga = std::clamp(vga, 0, RX888mk2::HF_VGA_STEPS - 1);
            ok = radio.setHfAttenuation(att) && radio.setHfVga(vga);
            std::cerr << "HF gain: attenuator " << att * 0.5 << " dB, VGA index " << vga << " (" << RX888mk2::hfVgaDb(vga)
                      << " dB)" << std::endl;
        }
        else {
            int rf = VHF_RF_DEFAULT, ifg = VHF_IF_DEFAULT;
            if (simple) {
                const float g = simple->getValue();
                for (int i = 0; i < RX888mk2::VHF_RF_STEPS; i++) {
                    if (std::fabs(RX888mk2::vhfRfGainDb(i) - g) < std::fabs(RX888mk2::vhfRfGainDb(rf) - g)) { rf = i; }
                }
            }
            else if (multi) {
                auto m = multi->getValue();
                if (m.count("rf")) { rf = std::stoi(m["rf"]); }
                if (m.count("if")) { ifg = std::stoi(m["if"]); }
            }
            rf = std::clamp(rf, 0, RX888mk2::VHF_RF_STEPS - 1);
            ifg = std::clamp(ifg, 0, RX888mk2::VHF_IF_STEPS - 1);
            ok = radio.setVhfRfGain(rf) && radio.setVhfIfGain(ifg);
            std::cerr << "VHF gain: R828D LNA index " << rf << " (" << RX888mk2::vhfRfGainDb(rf) << " dB), IF index " << ifg
                      << std::endl;
        }
        return ok ? 0 : 1;
    }

    // Caller holds dspMtx.
    bool startStream() {
        radio.startAdc();
        std::string err;
        if (!radio.device().startStreaming(XFER, 16, [this](const uint8_t* data, size_t len) { queue.push(data, len); }, err)) {
            std::cerr << "ERROR: could not start streaming: " << err << std::endl;
            radio.stopAdc();
            return false;
        }
        streaming = true;
        return true;
    }

    // Caller holds dspMtx: stop USB and drop whatever is queued.
    void stopStream() {
        if (!streaming) { return; }
        radio.device().stopStreaming();
        radio.stopAdc();
        while (queue.front(0)) { queue.pop(); }
        streaming = false;
    }

    void dspLoop() {
        std::vector<R2IQ::cf> buf;
        while (dspRun) {
            size_t n = 0;
            {
                std::unique_lock<std::mutex> lk(dspMtx);
                auto* b = queue.front(50);
                if (!b) { continue; }
                const size_t in = b->len / 2;
                if (buf.size() < r2iq.maxOutput(in)) { buf.resize(r2iq.maxOutput(in)); }
                n = r2iq.process((const int16_t*)b->data.data(), in, false, buf.data());
                queue.pop();
            }
            if (n) { processSamples((float*)buf.data(), (uint32_t)(2 * n)); }
        }
    }

    RX888mk2 radio;
    R2IQ r2iq;
    BlockQueue queue;
    std::mutex ctl;    // settings, from the control socket and setup()
    std::mutex dspMtx; // R2IQ reconfiguration and USB restarts vs the DSP thread
    std::thread dsp;
    std::atomic<bool> dspRun{ false };
    std::atomic<bool> failed{ false };
    bool active = false;    // read() has started: settings go to the hardware
    bool streaming = false;
    bool superSpeed = true;
    bool modeSet = false;
    bool curVhf = false;
    uint32_t adcReq = 0;    // Hz requested from the Si5351
    int decim = -1;
    double centre = 0, rate = 0, ppm = 0;
};

int main(int argc, char** argv) {
    if (argc > 0) {
        const char* slash = std::strrchr(argv[0], '/');
        programName = slash ? slash + 1 : argv[0];
    }
    Rx888Connector* connector = new Rx888Connector();
    return connector->main(argc, argv);
}
