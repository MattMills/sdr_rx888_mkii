// Real ADC samples -> decimated, tuned complex baseband ("r2iq").
//
// Fast-convolution DDC ported from ExtIO_sddc (Core/fft_mt_r2iq*.cpp),
// Copyright (c) 2017-2020 Oscar Steila ik1xpv and contributors, MIT License.
// See THIRD_PARTY_NOTICES.md.
//
// Each frame takes FFT_N real samples (hop HOP, i.e. 25% overlap), does a
// real-to-complex FFT, circularly shifts the bins so that the tuned frequency
// lands at DC (this is the mixer), multiplies by the spectrum of a Kaiser
// low-pass (the anti-alias filter) and runs a shorter inverse complex FFT,
// which decimates. Overlap-save keeps the first 3/4 of every inverse FFT.
//
// Differences from ExtIO_sddc's implementation:
//  - input is buffered internally, so any USB transfer size works;
//  - the coarse tuning step is one FFT bin (fs/8192) instead of four: the
//    per-frame phase jump of a bin that is not a multiple of 4 is
//    compensated (c_k = j^(bin*k));
//  - the residual (< 1/2 bin) is removed by a fine NCO at the output rate;
//  - the inverse-spectrum case (R828D high-side LO) conjugates the output
//    before the fine NCO, so the final spectrum is always upright;
//  - optional ADC DC-offset removal;
//  - frames are independent, so they are spread over worker threads with
//    all per-frame state (bin, NCO phase, frame index) computed up front:
//    the output does not depend on the thread count.
#pragma once
#include <atomic>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace rx888 {

class R2IQ {
public:
    static constexpr int FFT_N = 8192;           // real FFT length
    static constexpr int HALF = FFT_N / 2;       // complex bins 0..fs/2
    static constexpr int HOP = 3 * FFT_N / 4;    // new samples per frame
    static constexpr int MAX_DECIM = 6;          // output rate fs/2 .. fs/128
    static constexpr int FILTER_TAPS = HALF / 4 + 1;
    static constexpr int MAX_THREADS = 8;

    using cf = std::complex<float>;

    R2IQ();
    ~R2IQ();
    R2IQ(const R2IQ&) = delete;
    R2IQ& operator=(const R2IQ&) = delete;

    // Select the decimation (output rate = fs / (2 << decim)) and output
    // gain (1.0: a full-scale ADC sine gives |z| = 1). Creates FFTW plans on
    // first use, so call it from a single (UI) thread and never concurrently
    // with process().
    void configure(int decim, float gain = 1.0f);
    int decimation() const { return dec; }
    int ratio() const { return 2 << dec; }

    // Number of threads used by process() (including the calling thread).
    // Not concurrently with process().
    void setThreads(int n);
    int threads() const { return nThreads; }

    // Drop buffered samples (call before a new stream).
    void reset();

    // tuneHz: frequency in the real ADC spectrum (0..adcHz/2) to bring to DC.
    // adcHz:  actual ADC sample rate. invert: the RF spectrum is inverted in
    // the ADC signal (R828D IF) and the output must be conjugated.
    // Thread-safe; takes effect at the next process() call.
    void setTuning(double tuneHz, double adcHz, bool invert);

    // Upper bound on the output count for n more input samples.
    size_t maxOutput(size_t n) const;

    // Consume n ADC samples, write decoded complex samples to out and return
    // how many were written. derand: undo the LTC2208 output randomizer.
    size_t process(const int16_t* in, size_t n, bool derand, cf* out);

    // Subtract a slowly tracked estimate of the ADC DC offset before the FFT.
    // Without it the offset shows up as a spur at real-spectrum 0 Hz (HF: at
    // 0 Hz, VHF: at centre + R828D IF when the output band reaches it).
    void setDcRemoval(bool on) { dcRemoval = on; }

    // ADC statistics since the previous call: peak |sample| (raw ADC codes,
    // before DC removal) and number of samples within 0.1% of full scale.
    void takeStats(int& peak, uint64_t& clipped);
    double dcOffset() const { return dcEst; }

private:
    struct Scratch {
        void* freq = nullptr; // fftwf_complex[HALF + 1]
        void* tmp = nullptr;  // fftwf_complex[HALF]
    };
    struct Frame {
        const float* x;
        cf* out;
        int bin;
        bool inv;
        double fine;   // cycles per output sample
        double phase;  // NCO phase at the first output sample, cycles
        uint32_t idx;  // global frame index (bin phase correction)
    };

    void runFrame(const Frame& f, Scratch& s);
    void runFrames();
    void workerLoop(int id);
    void stopWorkers();
    void applyPendingTuning();
    void buildFilters();

    float* hist = nullptr;
    size_t histLen = 0;
    Scratch scratch[MAX_THREADS];
    void* filt[MAX_DECIM + 1] = {};
    void* planR2C = nullptr;
    void* planC2C[MAX_DECIM + 1] = {};
    float curGain = -1.0f;

    int dec = 0;
    int mfft = HALF;

    // Active tuning (owned by the processing thread).
    int bin = 0;
    bool inv = false;
    double fineCycles = 0.0;
    double ncoPhase = 0.0;
    uint32_t frameIdx = 0;

    // Pending tuning.
    std::mutex tuneMtx;
    bool tunePending = false;
    double pendTuneHz = 0, pendAdcHz = 64e6;
    bool pendInvert = false;

    // Frame batch shared with the workers.
    std::vector<Frame> frames;
    int nThreads = 1;
    std::vector<std::thread> workers;
    std::mutex poolMtx;
    std::condition_variable poolCv, doneCv;
    uint64_t generation = 0;
    int pendingWorkers = 0;
    bool poolExit = false;

    std::atomic<bool> dcRemoval{ true };
    std::atomic<double> dcEst{ 0.0 };
    std::atomic<int> peak{ 0 };
    std::atomic<uint64_t> clipCount{ 0 };
};

} // namespace rx888
