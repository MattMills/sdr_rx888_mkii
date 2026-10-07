#include "r2iq.h"
#include "kaiser.h"
#include <fftw3.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace rx888 {

namespace {

constexpr size_t HIST_CAP = R2IQ::FFT_N + 4 * 65536;
constexpr double TWO_PI = 6.283185307179586476925286766559;

inline fftwf_complex* C(void* p) { return (fftwf_complex*)p; }

} // namespace

R2IQ::R2IQ() {
    hist = (float*)fftwf_malloc(sizeof(float) * HIST_CAP);
    memset(hist, 0, sizeof(float) * HIST_CAP);
    for (auto& s : scratch) {
        s.freq = fftwf_malloc(sizeof(fftwf_complex) * (HALF + 1));
        s.tmp = fftwf_malloc(sizeof(fftwf_complex) * HALF);
    }
    for (int d = 0; d <= MAX_DECIM; d++) {
        filt[d] = fftwf_malloc(sizeof(fftwf_complex) * HALF);
    }
}

R2IQ::~R2IQ() {
    stopWorkers();
    if (planR2C) { fftwf_destroy_plan((fftwf_plan)planR2C); }
    for (int d = 0; d <= MAX_DECIM; d++) {
        if (planC2C[d]) { fftwf_destroy_plan((fftwf_plan)planC2C[d]); }
        fftwf_free(filt[d]);
    }
    for (auto& s : scratch) {
        fftwf_free(s.freq);
        fftwf_free(s.tmp);
    }
    fftwf_free(hist);
}

void R2IQ::buildFilters() {
    // Filter taps live on the fs/2 grid of the HALF-point bin spacing
    // (fs/FFT_N). They are placed at the end of the buffer (anti-causal), so
    // the first 3/4 of every inverse FFT is free of circular wrap-around.
    fftwf_complex* ht = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * HALF);
    fftwf_plan p = fftwf_plan_dft_1d(HALF, ht, C(filt[0]), FFTW_FORWARD, FFTW_ESTIMATE);
    std::vector<float> taps(FILTER_TAPS);
    // |X[k]| = A * FFT_N / 2 for a full-scale sine of amplitude A = 32768 and
    // the inverse FFT is unnormalized; scale so that maps to |z| = 1.
    const float scale = curGain / (32768.0f * (FFT_N / 2));
    for (int d = 0; d <= MAX_DECIM; d++) {
        const float bw = 0.5f / (float)(1 << d); // output Nyquist relative to fs/2
        kaiserLowpass(FILTER_TAPS, 120.0f, 0.85f * bw, 1.1f * bw, taps.data());
        memset(ht, 0, sizeof(fftwf_complex) * HALF);
        for (int t = 0; t < FILTER_TAPS; t++) {
            ht[HALF - 1 - t][0] = scale * taps[t];
        }
        fftwf_execute_dft(p, ht, C(filt[d]));
    }
    fftwf_destroy_plan(p);
    fftwf_free(ht);
}

void R2IQ::configure(int decim, float gain) {
    dec = std::clamp(decim, 0, MAX_DECIM);
    mfft = HALF >> dec;
    if (!planR2C) {
        planR2C = fftwf_plan_dft_r2c_1d(FFT_N, hist, C(scratch[0].freq), FFTW_MEASURE);
        for (int d = 0; d <= MAX_DECIM; d++) {
            planC2C[d] = fftwf_plan_dft_1d(HALF >> d, C(scratch[0].tmp), C(scratch[0].tmp), FFTW_BACKWARD, FFTW_MEASURE);
        }
    }
    if (gain != curGain) {
        curGain = gain;
        buildFilters();
    }
    // Re-derive the fine NCO for the new output rate.
    std::lock_guard<std::mutex> lck(tuneMtx);
    tunePending = true;
}

void R2IQ::setThreads(int n) {
    n = std::clamp(n, 1, MAX_THREADS);
    if (n == nThreads && (int)workers.size() == n - 1) { return; }
    stopWorkers();
    nThreads = n;
    poolExit = false;
    for (int i = 1; i < n; i++) { workers.emplace_back(&R2IQ::workerLoop, this, i); }
}

void R2IQ::stopWorkers() {
    {
        std::lock_guard<std::mutex> lck(poolMtx);
        poolExit = true;
    }
    poolCv.notify_all();
    for (auto& t : workers) { t.join(); }
    workers.clear();
}

void R2IQ::reset() {
    histLen = 0;
    ncoPhase = 0.0;
    frameIdx = 0;
    dcEst = 0.0;
}

void R2IQ::setTuning(double tuneHz, double adcHz, bool invert) {
    std::lock_guard<std::mutex> lck(tuneMtx);
    pendTuneHz = tuneHz;
    pendAdcHz = adcHz;
    pendInvert = invert;
    tunePending = true;
}

void R2IQ::applyPendingTuning() {
    std::lock_guard<std::mutex> lck(tuneMtx);
    if (!tunePending) { return; }
    tunePending = false;
    const double binHz = pendAdcHz / FFT_N;
    int b = (int)std::lround(pendTuneHz / binHz);
    b = std::clamp(b, 0, HALF);
    const double residual = pendTuneHz - b * binHz; // where the target sits after the bin shift
    const double outRate = pendAdcHz / ratio();
    bin = b;
    inv = pendInvert;
    // Without inversion the target is at +residual: shift by -residual.
    // With inversion the conjugate puts it at -residual: shift by +residual.
    fineCycles = (inv ? residual : -residual) / outRate;
}

size_t R2IQ::maxOutput(size_t n) const {
    return ((histLen + n) / HOP + 1) * (size_t)(3 * mfft / 4);
}

void R2IQ::runFrame(const Frame& f, Scratch& s) {
    fftwf_execute_dft_r2c((fftwf_plan)planR2C, (float*)f.x, C(s.freq));

    // Shift (mix) and filter into tmp: tmp[0, mfft/2) = positive offsets,
    // tmp[mfft/2, mfft) = negative offsets. Bins outside 0..HALF-1 (negative
    // frequencies / beyond Nyquist) are zero, so there is no mirror image.
    const cf* X = (const cf*)s.freq;
    const cf* H = (const cf*)filt[dec];
    cf* T = (cf*)s.tmp;
    const int h = mfft / 2;
    const int b = f.bin;

    int count = std::min(h, HALF - b);
    for (int i = 0; i < count; i++) { T[i] = X[b + i] * H[i]; }
    for (int i = std::max(count, 0); i < h; i++) { T[i] = 0; }

    int start = std::max(0, h - b);
    for (int i = 0; i < start; i++) { T[h + i] = 0; }
    const cf* X2 = X + (b - h);
    const cf* H2 = H + (HALF - h);
    for (int i = start; i < h; i++) { T[h + i] = X2[i] * H2[i]; }

    fftwf_execute_dft((fftwf_plan)planC2C[dec], C(s.tmp), C(s.tmp));

    // Keep the valid 3/4 and apply: frame phase correction for the bin shift,
    // optional conjugation and the fine NCO.
    const int n = 3 * mfft / 4;
    cf* out = f.out;
    static const cf jpow[4] = { cf(1, 0), cf(0, 1), cf(-1, 0), cf(0, -1) };
    cf ck = jpow[((uint32_t)b * f.idx) & 3];

    if (f.inv) {
        ck = std::conj(ck);
        for (int i = 0; i < n; i++) { T[i] = std::conj(T[i]); }
    }

    if (f.fine == 0.0) {
        if (ck == cf(1, 0)) { memcpy(out, T, sizeof(cf) * n); }
        else {
            for (int i = 0; i < n; i++) { out[i] = T[i] * ck; }
        }
        return;
    }
    // Four interleaved phasors so the recurrence vectorizes; restarted from
    // the double-precision phase every frame (no drift).
    std::complex<double> p = std::polar(1.0, TWO_PI * f.phase) * std::complex<double>(ck);
    const std::complex<double> st = std::polar(1.0, TWO_PI * f.fine);
    cf ph[4];
    for (int l = 0; l < 4; l++) {
        ph[l] = cf(p);
        p *= st;
    }
    const cf step4 = cf(std::polar(1.0, TWO_PI * f.fine * 4.0));
    int i = 0;
    for (; i + 4 <= n; i += 4) {
        out[i] = T[i] * ph[0];
        out[i + 1] = T[i + 1] * ph[1];
        out[i + 2] = T[i + 2] * ph[2];
        out[i + 3] = T[i + 3] * ph[3];
        ph[0] *= step4;
        ph[1] *= step4;
        ph[2] *= step4;
        ph[3] *= step4;
    }
    for (int l = 0; i < n; i++, l++) { out[i] = T[i] * ph[l]; }
}

void R2IQ::workerLoop(int id) {
    uint64_t seen = 0;
    while (true) {
        int T;
        {
            std::unique_lock<std::mutex> lck(poolMtx);
            poolCv.wait(lck, [&] { return poolExit || generation != seen; });
            if (poolExit) { return; }
            seen = generation;
            T = std::min(nThreads, (int)frames.size());
        }
        if (id >= T) { continue; }
        const size_t n = frames.size();
        for (size_t k = id * n / T; k < (id + 1) * n / T; k++) { runFrame(frames[k], scratch[id]); }
        std::lock_guard<std::mutex> lck(poolMtx);
        if (--pendingWorkers == 0) { doneCv.notify_one(); }
    }
}

void R2IQ::runFrames() {
    const size_t n = frames.size();
    const int T = std::min(nThreads, (int)n);
    if (T <= 1) {
        for (const auto& f : frames) { runFrame(f, scratch[0]); }
        return;
    }
    {
        std::lock_guard<std::mutex> lck(poolMtx);
        pendingWorkers = T - 1;
        generation++;
    }
    poolCv.notify_all();
    for (size_t k = 0; k < n / T; k++) { runFrame(frames[k], scratch[0]); }
    std::unique_lock<std::mutex> lck(poolMtx);
    doneCv.wait(lck, [&] { return pendingWorkers == 0; });
}

size_t R2IQ::process(const int16_t* in, size_t n, bool derand, cf* out) {
    size_t produced = 0;
    int pk = 0;
    uint64_t clip = 0;
    const int nOut = 3 * mfft / 4;
    while (n > 0) {
        size_t take = std::min(n, HIST_CAP - histLen);
        float* dst = hist + histLen;
        if (derand) {
            // LTC2208 RAND: bits 15..1 are XORed with bit 0.
            for (size_t i = 0; i < take; i++) {
                int16_t v = in[i];
                v ^= (int16_t)(-(v & 1)) & (int16_t)0xFFFE;
                dst[i] = (float)v;
            }
        }
        else {
            for (size_t i = 0; i < take; i++) { dst[i] = (float)in[i]; }
        }
        // Stats on the decoded samples.
        float mx = 0.0f, mn = 0.0f;
        double sum = 0.0;
        for (size_t i = 0; i < take; i++) {
            mx = std::max(mx, dst[i]);
            mn = std::min(mn, dst[i]);
            sum += dst[i];
        }
        pk = std::max(pk, (int)std::max(mx, -mn));
        if (mx >= 32735.0f || mn <= -32735.0f) {
            for (size_t i = 0; i < take; i++) {
                if (dst[i] >= 32735.0f || dst[i] <= -32735.0f) { clip++; }
            }
        }
        if (take > 0) {
            // ~25 chunks time constant (tens of ms at typical rates)
            double dc = dcEst.load();
            dc += 0.04 * (sum / (double)take - dc);
            dcEst = dc;
            if (dcRemoval) {
                const float d = (float)dc;
                for (size_t i = 0; i < take; i++) { dst[i] -= d; }
            }
        }

        histLen += take;
        in += take;
        n -= take;

        // Collect the frames that are complete, with all per-frame state.
        applyPendingTuning();
        frames.clear();
        size_t pos = 0;
        while (histLen - pos >= (size_t)FFT_N) {
            frames.push_back({ hist + pos, out + produced, bin, inv, fineCycles, ncoPhase, frameIdx });
            produced += nOut;
            if (fineCycles != 0.0) {
                ncoPhase += fineCycles * nOut;
                ncoPhase -= std::floor(ncoPhase);
            }
            frameIdx++;
            pos += HOP;
        }
        runFrames();
        if (pos) {
            memmove(hist, hist + pos, sizeof(float) * (histLen - pos));
            histLen -= pos;
        }
    }

    int prev = peak.load();
    while (pk > prev && !peak.compare_exchange_weak(prev, pk)) {}
    clipCount += clip;
    return produced;
}

void R2IQ::takeStats(int& pk, uint64_t& clipped) {
    pk = peak.exchange(0);
    clipped = clipCount.exchange(0);
}

} // namespace rx888
