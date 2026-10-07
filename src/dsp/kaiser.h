// Kaiser-windowed sinc low-pass FIR design.
//
// Ported from ExtIO_sddc (Core/fir.cpp), Copyright (c) 2017-2020 Oscar Steila
// ik1xpv, MIT License. See THIRD_PARTY_NOTICES.md.
#pragma once

namespace rx888 {

// Designs a low-pass FIR with unity DC gain.
//   numTaps   > 0: use exactly this many taps; <= 0: estimate (and clamp to -numTaps if < 0)
//   astop     stopband attenuation in dB
//   normFpass passband edge, relative to the sample rate
//   normFstop stopband edge, relative to the sample rate
//   coef      output (may be nullptr to only estimate the tap count)
// Returns the number of taps.
int kaiserLowpass(int numTaps, float astop, float normFpass, float normFstop, float* coef);

} // namespace rx888
