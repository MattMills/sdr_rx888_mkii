// rx888_tool: command line test utility for the RX888 mkII.
// Uses the same device and DSP code as the SDR++ module.
#include "device/block_queue.h"
#include "device/firmware.h"
#include "device/fx3_device.h"
#include "device/rx888_mk2.h"
#include "device/tuning.h"
#include "dsp/r2iq.h"
#include <fftw3.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace rx888;
using Clock = std::chrono::steady_clock;

namespace {

std::map<std::string, std::string> opts;

bool has(const char* k) { return opts.count(k) > 0; }
std::string str(const char* k, const std::string& def = "") { return has(k) ? opts[k] : def; }
double num(const char* k, double def) { return has(k) ? atof(opts[k].c_str()) : def; }

void usage() {
    printf(
        "usage: rx888_tool <command> [options]\n"
        "  list                         list FX3/SDDC devices\n"
        "  fwload  [--dev P] [--img F]  upload firmware (embedded SDDC_FX3.img by default)\n"
        "  info    [--dev P]            load firmware if needed and print model/firmware/speed\n"
        "  stream  [--adc HZ] [--seconds S] [--xfer BYTES] [--ntx N] [--ddc DECIM] [--rand]\n"
        "                               USB throughput test, optionally running the DDC live\n"
        "  capture --out PREFIX [--mode hf|vhf] [--adc HZ] [--freq HZ] [--decim D] [--seconds S]\n"
        "          [--att 0..63] [--vga 0..126] [--rfgain 0..28] [--ifgain 0..15] [--ifvga CODE]\n"
        "          [--dither] [--rand] [--pgaoff] [--ppm X] [--raw] [--settle MS]\n"
        "                               capture raw ADC (PREFIX.s16) and/or DDC output (PREFIX.cf32)\n"
        "  scan    [--mode vhf|hf] [--start HZ] [--stop HZ] [--step HZ] [--adc HZ] [--decim D] [--snr DB] [--csv F]\n"
        "                               step the tuner across a range and list spectral peaks\n"
        "  selftest                     synthetic DDC checks (frequency, level, image, threads)\n"
        "  bench   [--threads N]        DDC throughput on synthetic data\n");
}

std::string pickDevice() {
    auto devs = Fx3Device::enumerate();
    if (devs.empty()) {
        fprintf(stderr, "no FX3 device found\n");
        exit(1);
    }
    if (has("dev")) { return str("dev"); }
    return devs[0].path;
}

// Load the firmware if the device at path is still in bootloader mode.
// Returns the path of the device running the firmware.
std::string ensureFirmware(const std::string& path) {
    for (const auto& d : Fx3Device::enumerate()) {
        if (d.path != path) { continue; }
        if (!d.bootloader) { return path; }
        printf("device %s is in bootloader mode, uploading firmware (%zu bytes)...\n", path.c_str(), sddc_fx3_firmware_size);
        std::string err, newPath;
        auto t0 = Clock::now();
        if (!Fx3Device::loadFirmware(path, sddc_fx3_firmware, sddc_fx3_firmware_size, newPath, err)) {
            fprintf(stderr, "firmware upload failed: %s\n", err.c_str());
            exit(1);
        }
        printf("firmware running at %s (%.2f s)\n", newPath.c_str(), std::chrono::duration<double>(Clock::now() - t0).count());
        return newPath;
    }
    return path;
}

int cmdList() {
    auto devs = Fx3Device::enumerate();
    printf("%zu device(s)\n", devs.size());
    for (const auto& d : devs) {
        printf("  path=%s  %04X:%04X  %s  %s  serial='%s' product='%s' %s\n", d.path.c_str(), d.vid, d.pid,
               d.bootloader ? "BOOTLOADER" : "FIRMWARE", usbSpeedName(d.speed), d.serial.c_str(), d.product.c_str(),
               d.accessible ? "" : ("[" + d.error + "]").c_str());
    }
    return 0;
}

int cmdFwload() {
    std::string path = pickDevice();
    std::vector<uint8_t> img(sddc_fx3_firmware, sddc_fx3_firmware + sddc_fx3_firmware_size);
    if (has("img")) {
        FILE* f = fopen(str("img").c_str(), "rb");
        if (!f) {
            fprintf(stderr, "cannot open %s\n", str("img").c_str());
            return 1;
        }
        fseek(f, 0, SEEK_END);
        img.resize(ftell(f));
        fseek(f, 0, SEEK_SET);
        if (fread(img.data(), 1, img.size(), f) != img.size()) {
            fprintf(stderr, "read error\n");
            return 1;
        }
        fclose(f);
    }
    std::string err, newPath;
    auto t0 = Clock::now();
    if (!Fx3Device::loadFirmware(path, img.data(), img.size(), newPath, err)) {
        fprintf(stderr, "firmware upload failed: %s\n", err.c_str());
        return 1;
    }
    printf("firmware uploaded and device re-enumerated in %.2f s\n", std::chrono::duration<double>(Clock::now() - t0).count());
    return cmdList();
}

bool openRadio(RX888mk2& radio, std::string& path) {
    path = ensureFirmware(pickDevice());
    std::string err;
    if (!radio.open(path, err)) {
        fprintf(stderr, "open failed: %s\n", err.c_str());
        return false;
    }
    printf("opened %s: model 0x%02X (%s), firmware %d.%02d, %s\n", path.c_str(), radio.model(), modelName(radio.model()),
           radio.firmwareVersion() >> 8, radio.firmwareVersion() & 0xFF, usbSpeedName(radio.device().speed()));
    return true;
}

int cmdInfo() {
    RX888mk2 radio;
    std::string path;
    if (!openRadio(radio, path)) { return 1; }
    radio.close();
    return 0;
}

int cmdStream() {
    RX888mk2 radio;
    std::string path;
    if (!openRadio(radio, path)) { return 1; }
    const uint32_t adc = (uint32_t)num("adc", 64e6);
    const double seconds = num("seconds", 5);
    const size_t xfer = (size_t)num("xfer", 131072);
    const int ntx = (int)num("ntx", 16);
    const int ddc = (int)num("ddc", -1);
    const bool rand = has("rand");

    radio.setAdcRate(adc);
    radio.setRandomizer(rand);

    BlockQueue q;
    q.init(xfer, 256);
    std::atomic<uint64_t> bytes{ 0 };
    std::atomic<bool> run{ true };
    R2IQ r2iq;
    double dspBusy = 0;
    uint64_t outSamples = 0;
    std::thread dsp;
    if (ddc >= 0) {
        r2iq.configure(ddc);
        r2iq.setTuning(adc / 4.0, adc, false);
        dsp = std::thread([&] {
            std::vector<R2IQ::cf> out(r2iq.maxOutput(xfer / 2) + 4096);
            while (run) {
                auto* b = q.front();
                if (!b) { continue; }
                auto t0 = Clock::now();
                outSamples += r2iq.process((const int16_t*)b->data.data(), b->len / 2, rand, out.data());
                dspBusy += std::chrono::duration<double>(Clock::now() - t0).count();
                q.pop();
            }
        });
    }

    radio.startAdc();
    std::string err;
    auto t0 = Clock::now();
    if (!radio.device().startStreaming(xfer, ntx, [&](const uint8_t* d, size_t n) {
            bytes += n;
            if (ddc >= 0) { q.push(d, n); }
        }, err)) {
        fprintf(stderr, "stream start failed: %s\n", err.c_str());
        return 1;
    }
    uint64_t last = 0;
    auto tl = t0;
    while (std::chrono::duration<double>(Clock::now() - t0).count() < seconds) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto now = Clock::now();
        uint64_t b = bytes;
        double dt = std::chrono::duration<double>(now - tl).count();
        printf("  %.1f MB/s (%.2f MS/s)  queue %zu/%zu  dropped %llu  usb errors %llu\n", (b - last) / dt / 1e6, (b - last) / dt / 2e6,
               q.fill(), q.capacity(), (unsigned long long)q.droppedCount(), (unsigned long long)radio.device().transferErrors());
        last = b;
        tl = now;
    }
    double el = std::chrono::duration<double>(Clock::now() - t0).count();
    radio.device().stopStreaming();
    radio.stopAdc();
    run = false;
    q.stop();
    if (dsp.joinable()) { dsp.join(); }
    printf("average %.1f MB/s = %.3f MS/s (ADC set to %.3f MS/s), usb errors %llu, dropped blocks %llu\n", bytes / el / 1e6, bytes / el / 2e6,
           adc / 1e6, (unsigned long long)radio.device().transferErrors(), (unsigned long long)q.droppedCount());
    if (ddc >= 0) {
        printf("DDC decim %d: output %.3f MS/s, DSP thread busy %.1f%%\n", ddc, outSamples / el / 1e6, 100.0 * dspBusy / el);
    }
    if (!radio.device().lastError().empty()) { printf("last USB error: %s\n", radio.device().lastError().c_str()); }
    radio.close();
    return 0;
}

int cmdCapture() {
    if (!has("out")) {
        usage();
        return 1;
    }
    RX888mk2 radio;
    std::string path;
    if (!openRadio(radio, path)) { return 1; }

    const std::string prefix = str("out");
    const bool vhf = str("mode", "hf") == "vhf";
    const uint32_t adc = (uint32_t)num("adc", 64e6);
    const double freq = num("freq", vhf ? 100e6 : adc / 4.0);
    const int decim = (int)num("decim", vhf ? 3 : 2);
    const double seconds = num("seconds", 0.5);
    const double ppm = num("ppm", 0);
    const bool rand = has("rand");
    const int settleMs = (int)num("settle", 300);

    radio.setAdcRate(adc);
    radio.setDither(has("dither"));
    radio.setRandomizer(rand);
    radio.setAdcHighGain(!has("pgaoff"));
    if (vhf) {
        radio.setInput(RX888mk2::Input::VHF);
        radio.setVhfRfGain((int)num("rfgain", 16));
        radio.setVhfIfGain((int)num("ifgain", 8));
        if (has("ifvga")) { radio.setIfVgaCode((uint8_t)num("ifvga", 0x83)); }
    }
    else {
        radio.setInput(RX888mk2::Input::HF);
        radio.setHfAttenuation((int)num("att", 0));
        radio.setHfVga((int)num("vga", 30));
    }
    DdcTuning tn = computeTuning(vhf ? RX888mk2::Input::VHF : RX888mk2::Input::HF, freq, adc, ppm);
    if (vhf) {
        radio.tune(tn.tunerHz);
        printf("R828D: RF %.0f Hz, LO %.3f Hz (nominal %.0f, error %+.3f Hz), IF %.3f Hz\n", freq, tn.loHz, freq + RX888mk2::R828D_IF_HZ,
               tn.loHz - (freq + RX888mk2::R828D_IF_HZ), tn.tuneHz);
    }

    const size_t want = (size_t)(seconds * adc);
    const size_t skip = (size_t)(settleMs / 1000.0 * adc);
    std::vector<int16_t> raw(want);
    std::atomic<size_t> got{ 0 }, seen{ 0 };
    radio.startAdc();
    std::string err;
    if (!radio.device().startStreaming(131072, 16, [&](const uint8_t* d, size_t n) {
            size_t ns = n / 2;
            size_t s = seen.fetch_add(ns);
            if (s < skip) { return; }
            size_t g = got.load();
            if (g >= want) { return; }
            size_t c = std::min(ns, want - g);
            memcpy(raw.data() + g, d, c * 2);
            got = g + c;
        }, err)) {
        fprintf(stderr, "stream start failed: %s\n", err.c_str());
        return 1;
    }
    auto t0 = Clock::now();
    while (got < want && std::chrono::duration<double>(Clock::now() - t0).count() < seconds + settleMs / 1000.0 + 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    radio.device().stopStreaming();
    radio.stopAdc();
    uint64_t usbErr = radio.device().transferErrors();
    radio.close();
    if (got < want) {
        fprintf(stderr, "only captured %zu of %zu samples\n", (size_t)got, want);
        raw.resize(got);
    }

    // Raw statistics (before and after de-randomization).
    auto stats = [&](bool derand) {
        double sum = 0, sum2 = 0;
        int mn = 32767, mx = -32768;
        size_t odd = 0;
        for (int16_t s0 : raw) {
            int16_t s = s0;
            if (derand && (s & 1)) { s ^= (int16_t)0xFFFE; }
            sum += s;
            sum2 += (double)s * s;
            mn = std::min<int>(mn, s);
            mx = std::max<int>(mx, s);
            odd += (s0 & 1);
        }
        double n = (double)raw.size();
        double mean = sum / n;
        double rms = sqrt(sum2 / n - mean * mean);
        printf("  %-12s mean %+8.2f  rms %8.2f (%6.1f dBFS)  min %6d  max %6d  odd-LSB %.3f\n", derand ? "derandomized" : "raw", mean, rms,
               20 * log10(rms / 32768.0 + 1e-12), mn, mx, odd / n);
    };
    printf("captured %zu samples (%.3f s at %.3f MS/s), usb errors %llu\n", raw.size(), raw.size() / (double)adc, adc / 1e6,
           (unsigned long long)usbErr);
    stats(false);
    if (rand) { stats(true); }

    if (has("raw")) {
        std::string fn = prefix + ".s16";
        FILE* f = fopen(fn.c_str(), "wb");
        fwrite(raw.data(), 2, raw.size(), f);
        fclose(f);
        printf("wrote %s\n", fn.c_str());
    }

    R2IQ r2iq;
    r2iq.configure(decim);
    r2iq.setTuning(tn.tuneHz, tn.adcHz, tn.invert);
    std::vector<R2IQ::cf> out(r2iq.maxOutput(raw.size()) + 8192);
    size_t nout = 0;
    for (size_t off = 0; off < raw.size(); off += 65536) {
        size_t n = std::min<size_t>(65536, raw.size() - off);
        nout += r2iq.process(raw.data() + off, n, rand, out.data() + nout);
    }
    std::string fn = prefix + ".cf32";
    FILE* f = fopen(fn.c_str(), "wb");
    fwrite(out.data(), sizeof(R2IQ::cf), nout, f);
    fclose(f);
    int pk;
    uint64_t clip;
    r2iq.takeStats(pk, clip);
    printf("DDC: centre %.3f Hz, tune %.3f Hz%s, decim %d -> %.6f MS/s, %zu samples -> %s (ADC peak %d = %.1f dBFS, %llu near full scale)\n",
           freq, tn.tuneHz, tn.invert ? " (inverted)" : "", decim, tn.adcHz / r2iq.ratio() / 1e6, nout, fn.c_str(), pk,
           20 * log10(pk / 32768.0 + 1e-12), (unsigned long long)clip);
    return 0;
}

// Welch PSD (Blackman-Harris, 50% overlap) of complex samples, in dB
// relative to a full-scale complex tone, with DC at the centre (fftshifted).
std::vector<double> psd(const std::vector<R2IQ::cf>& x, int n) {
    std::vector<double> acc(n, 0.0), win(n);
    double wsum = 0;
    for (int i = 0; i < n; i++) {
        double a = 6.283185307179586 * i / (n - 1);
        win[i] = 0.35875 - 0.48829 * cos(a) + 0.14128 * cos(2 * a) - 0.01168 * cos(3 * a);
        wsum += win[i];
    }
    fftwf_complex* buf = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * n);
    fftwf_plan p = fftwf_plan_dft_1d(n, buf, buf, FFTW_FORWARD, FFTW_ESTIMATE);
    int segs = 0;
    for (size_t off = 0; off + n <= x.size(); off += n / 2, segs++) {
        for (int i = 0; i < n; i++) {
            buf[i][0] = x[off + i].real() * (float)win[i];
            buf[i][1] = x[off + i].imag() * (float)win[i];
        }
        fftwf_execute(p);
        for (int i = 0; i < n; i++) { acc[(i + n / 2) % n] += buf[i][0] * (double)buf[i][0] + buf[i][1] * (double)buf[i][1]; }
    }
    fftwf_destroy_plan(p);
    fftwf_free(buf);
    for (auto& v : acc) { v = 10 * log10(v / std::max(segs, 1) / (wsum * wsum) + 1e-30); }
    return acc;
}

// Tune across a range on the selected input and report the strongest
// narrow peaks (absolute frequency, level, height above the local median).
int cmdScan() {
    RX888mk2 radio;
    std::string path;
    if (!openRadio(radio, path)) { return 1; }
    const bool vhf = str("mode", "vhf") == "vhf";
    const uint32_t adc = (uint32_t)num("adc", 20e6);
    const int decim = (int)num("decim", 1);
    const double start = num("start", vhf ? 250e6 : 1e6);
    const double stop = num("stop", vhf ? 1000e6 : adc / 2.0);
    const double outRate = adc / (double)(2 << decim);
    const double step = num("step", outRate * 0.8);
    const double ppm = num("ppm", 0);
    const double minSnr = num("snr", 20);
    const int fftN = (int)num("fft", 8192);
    const std::string csv = str("csv");

    radio.setAdcRate(adc);
    radio.setAdcHighGain(!has("pgaoff"));
    if (vhf) {
        radio.setInput(RX888mk2::Input::VHF);
        radio.setVhfRfGain((int)num("rfgain", 20));
        radio.setVhfIfGain((int)num("ifgain", 8));
    }
    else {
        radio.setInput(RX888mk2::Input::HF);
        radio.setHfAttenuation((int)num("att", 0));
        radio.setHfVga((int)num("vga", 30));
    }

    const size_t want = (size_t)(num("dwell", 0.05) * adc);
    std::vector<int16_t> raw(want);
    std::atomic<size_t> got{ 0 };
    std::atomic<bool> armed{ false };
    radio.startAdc();
    std::string err;
    if (!radio.device().startStreaming(131072, 16, [&](const uint8_t* d, size_t n) {
            if (!armed) { return; }
            size_t g = got.load();
            size_t c = std::min(n / 2, want - g);
            memcpy(raw.data() + g, d, c * 2);
            got = g + c;
            if (got >= want) { armed = false; }
        }, err)) {
        fprintf(stderr, "stream start failed: %s\n", err.c_str());
        return 1;
    }
    FILE* fcsv = csv.empty() ? nullptr : fopen(csv.c_str(), "w");
    R2IQ r2iq;
    r2iq.configure(decim);
    printf("scan %.3f..%.3f MHz step %.3f MHz, output %.3f MS/s, %d-pt FFT (%.0f Hz bins)\n", start / 1e6, stop / 1e6, step / 1e6, outRate / 1e6,
           fftN, outRate / fftN);
    for (double fc = start; fc <= stop; fc += step) {
        DdcTuning tn = computeTuning(vhf ? RX888mk2::Input::VHF : RX888mk2::Input::HF, fc, adc, ppm);
        if (vhf) { radio.tune(tn.tunerHz); }
        std::this_thread::sleep_for(std::chrono::milliseconds((int)num("settle", 30)));
        got = 0;
        armed = true;
        while (armed) { std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
        r2iq.reset();
        r2iq.setTuning(tn.tuneHz, tn.adcHz, tn.invert);
        std::vector<R2IQ::cf> out(r2iq.maxOutput(want) + 8192);
        size_t nout = 0;
        for (size_t off = 0; off < want; off += 65536) {
            nout += r2iq.process(raw.data() + off, std::min<size_t>(65536, want - off), false, out.data() + nout);
        }
        out.resize(nout);
        auto p = psd(out, fftN);
        // Only the flat part of the passband (+-40% of the output rate).
        int lo = (int)(fftN * 0.1), hi = (int)(fftN * 0.9);
        std::vector<double> band(p.begin() + lo, p.begin() + hi);
        std::nth_element(band.begin(), band.begin() + band.size() / 2, band.end());
        double med = band[band.size() / 2];
        for (int i = lo; i < hi; i++) {
            double f = fc + (i - fftN / 2) * outRate / fftN;
            if (fcsv && (i % 4 == 0)) { fprintf(fcsv, "%.0f,%.2f\n", f, p[i]); }
            if (p[i] < med + minSnr) { continue; }
            bool isMax = true;
            for (int k = std::max(lo, i - 8); k <= std::min(hi - 1, i + 8); k++) {
                if (p[k] > p[i]) { isMax = false; }
            }
            if (!isMax) { continue; }
            double a = p[i - 1], b = p[i], c = p[i + 1];
            double d = 0.5 * (a - c) / (a - 2 * b + c);
            printf("  peak %14.3f Hz  %6.1f dBFS  +%4.1f dB  (step centre %.3f MHz)\n", fc + (i + d - fftN / 2) * outRate / fftN, b, b - med, fc / 1e6);
        }
    }
    if (fcsv) { fclose(fcsv); }
    radio.device().stopStreaming();
    radio.close();
    return 0;
}

int cmdBench() {
    const size_t n = 64 * 1024 * 1024;
    const int threads = (int)num("threads", 1);
    std::vector<int16_t> in(n);
    uint32_t x = 1;
    for (size_t i = 0; i < n; i++) {
        x = x * 1664525u + 1013904223u;
        in[i] = (int16_t)(1000.0 * sin(i * 0.1) + ((x >> 16) & 0xFF) - 128);
    }
    for (int d = 0; d <= R2IQ::MAX_DECIM; d++) {
        R2IQ r;
        r.configure(d);
        r.setThreads(threads);
        r.setTuning(10.1e6, 64e6, d & 1);
        std::vector<R2IQ::cf> out(r.maxOutput(65536) + 4096);
        auto t0 = Clock::now();
        size_t total = 0;
        for (size_t off = 0; off < n; off += 65536) { total += r.process(in.data() + off, 65536, false, out.data()); }
        double el = std::chrono::duration<double>(Clock::now() - t0).count();
        printf("decim %d, %d thread(s): %.1f MS/s input (%zu out)\n", d, threads, n / el / 1e6, total);
    }
    return 0;
}

// Synthetic end-to-end checks of the DDC: tone frequencies after tuning
// (including odd FFT bins and the inverted/VHF case), image rejection, and
// that the output is identical for 1 and N threads.
int cmdSelftest() {
    const double fs = 64e6;
    const size_t n = 1 << 22; // 65.5 ms
    struct Tone {
        double f, amp;
    };
    const Tone tones[] = { { 10123456.7, 0.25 }, { 10301000.0, 0.02 } };
    std::vector<int16_t> in(n);
    uint32_t seed = 12345;
    for (size_t i = 0; i < n; i++) {
        double v = 0;
        for (const auto& t : tones) { v += t.amp * 32767.0 * cos(6.283185307179586 * t.f * i / fs); }
        seed = seed * 1664525u + 1013904223u;
        v += ((seed >> 16) & 0x7) - 3.5; // ~2 LSB rms noise
        in[i] = (int16_t)lround(v);
    }
    int failures = 0;
    auto run = [&](int decim, double tune, bool invert, int threads, std::vector<R2IQ::cf>& out) {
        R2IQ r;
        r.configure(decim);
        r.setThreads(threads);
        r.setTuning(tune, fs, invert);
        out.assign(r.maxOutput(n) + 8192, R2IQ::cf());
        size_t got = 0;
        for (size_t off = 0; off < n; off += 65536) { got += r.process(in.data() + off, 65536, false, out.data() + got); }
        out.resize(got);
    };
    // Tunings: on a multiple-of-4 bin, odd bins, and between bins.
    const double binHz = fs / R2IQ::FFT_N;
    const double tunes[] = { 1296 * binHz, 1297 * binHz, 1299 * binHz, 1298.37 * binHz, 10.2e6 };
    for (int decim : { 2, 4, 6 }) {
        const double outRate = fs / (2 << decim);
        for (double tune : tunes) {
            for (bool invert : { false, true }) {
                std::vector<R2IQ::cf> o1, o4;
                run(decim, tune, invert, 1, o1);
                run(decim, tune, invert, 4, o4);
                bool same = o1.size() == o4.size() && memcmp(o1.data(), o4.data(), o1.size() * sizeof(R2IQ::cf)) == 0;
                // Tone check. Non-inverted: tone at (f - tune). Inverted: the
                // conjugate maps it to -(f - tune).
                std::vector<R2IQ::cf> seg(o1.begin() + 4096, o1.end());
                const int N = 1 << 16;
                if ((int)seg.size() < N) { continue; }
                seg.resize(N);
                auto p = psd(seg, N);
                for (const auto& t : tones) {
                    double off = invert ? -(t.f - tune) : (t.f - tune);
                    if (fabs(off) > 0.4 * outRate) { continue; }
                    int k = (int)lround(off / outRate * N) + N / 2;
                    int best = k;
                    for (int j = k - 3; j <= k + 3; j++) {
                        if (p[j] > p[best]) { best = j; }
                    }
                    double a = p[best - 1], b = p[best], c = p[best + 1];
                    double d = 0.5 * (a - c) / (a - 2 * b + c);
                    double meas = (best + d - N / 2) * outRate / N;
                    int km = N / 2 - (k - N / 2); // mirror bin
                    double img = -1000;
                    for (int j = km - 3; j <= km + 3; j++) { img = std::max(img, p[j]); }
                    double expectDb = 20 * log10(t.amp);
                    bool ok = fabs(meas - off) < outRate / N * 0.15 && fabs(b - expectDb) < 1.0 && (b - img) > 80 && same;
                    if (!ok) { failures++; }
                    printf("%s decim %d tune %.3f Hz %s: tone %+.1f Hz -> %+.2f Hz (err %+.2f), %.2f dBFS (exp %.2f), image %.1f dB down, 1 vs 4 threads %s\n",
                           ok ? "PASS" : "FAIL", decim, tune, invert ? "inv" : "   ", off, meas, meas - off, b, expectDb, b - img,
                           same ? "identical" : "DIFFERENT");
                }
            }
        }
    }
    printf("%s (%d failures)\n", failures ? "SELFTEST FAILED" : "SELFTEST PASSED", failures);
    return failures ? 1 : 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }
    std::string cmd = argv[1];
    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        if (a.rfind("--", 0) != 0) { continue; }
        a = a.substr(2);
        if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) { opts[a] = argv[++i]; }
        else { opts[a] = "1"; }
    }
    if (cmd == "list") { return cmdList(); }
    if (cmd == "fwload") { return cmdFwload(); }
    if (cmd == "info") { return cmdInfo(); }
    if (cmd == "stream") { return cmdStream(); }
    if (cmd == "capture") { return cmdCapture(); }
    if (cmd == "scan") { return cmdScan(); }
    if (cmd == "bench") { return cmdBench(); }
    if (cmd == "selftest") { return cmdSelftest(); }
    usage();
    return 1;
}
