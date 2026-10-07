// Maps a requested RF centre frequency to the DDC tuning for each input path.
#pragma once
#include "rx888_mk2.h"
#include <cmath>
#include <cstdint>

namespace rx888 {

struct DdcTuning {
    double adcHz = 0;    // actual ADC sample rate (ppm corrected)
    double tuneHz = 0;   // frequency in the real ADC spectrum to move to DC
    bool invert = false; // spectrum is inverted in the ADC signal
    uint64_t tunerHz = 0; // VHF: frequency sent to the R828D
    double loHz = 0;     // VHF: actual R828D LO
};

inline DdcTuning computeTuning(RX888mk2::Input input, double centerHz, double adcNominalHz, double ppm) {
    DdcTuning t;
    // ppm is the error of the 27 MHz crystal; the Si5351 synthesis error of
    // each clock is modelled exactly on top of it.
    const double k = 1.0 + ppm * 1e-6;
    t.adcHz = RX888mk2::si5351OutputHz((uint32_t)adcNominalHz) * k;
    if (input == RX888mk2::Input::HF) {
        t.tuneHz = centerHz;
        t.invert = false;
    }
    else {
        // The R828D LO sits above the RF (LO = RF + IF), so the tuner output
        // is inverted; the ADC sees the requested centre at (LO - centre).
        t.tunerHz = (uint64_t)std::llround(centerHz);
        t.loHz = RX888mk2::r828dLoHz(t.tunerHz, RX888mk2::si5351OutputHz(RX888mk2::R828D_REF_HZ) * k);
        t.tuneHz = t.loHz - centerHz;
        t.invert = true;
    }
    return t;
}

} // namespace rx888
