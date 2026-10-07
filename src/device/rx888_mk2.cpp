#include "rx888_mk2.h"
#include <algorithm>
#include <cmath>
#include <thread>

namespace rx888 {

namespace {

// AD8370 code mapping used by ExtIO_sddc for the RX888 mkII.
constexpr int GAIN_SWEET_POINT = 18;
constexpr float HIGH_GAIN_RATIO = 0.409f;
constexpr float LOW_GAIN_RATIO = 0.059f;
constexpr uint8_t AD8370_HIGH_MODE = 0x80;

// R828D gain tables from ExtIO_sddc RX888R2Radio.cpp (dB).
const float VHF_RF_DB[RX888mk2::VHF_RF_STEPS] = {
    0.0f, 0.9f, 1.4f, 2.7f, 3.7f, 7.7f, 8.7f, 12.5f, 14.4f, 15.7f,
    16.6f, 19.7f, 20.7f, 22.9f, 25.4f, 28.0f, 29.7f, 32.8f,
    33.8f, 36.4f, 37.2f, 38.6f, 40.2f, 42.1f, 43.4f, 43.9f,
    44.5f, 48.0f, 49.6f
};
const float VHF_IF_DB[RX888mk2::VHF_IF_STEPS] = {
    -4.7f, -2.1f, 0.5f, 3.5f, 7.7f, 11.2f, 13.6f, 14.9f, 16.3f, 19.5f, 23.1f, 26.5f, 30.0f, 33.7f, 37.2f, 40.8f
};

} // namespace

float RX888mk2::hfVgaDb(int i) {
    i = std::clamp(i, 0, HF_VGA_STEPS - 1);
    if (i > GAIN_SWEET_POINT) { return 20.0f * log10f(HIGH_GAIN_RATIO * (i - GAIN_SWEET_POINT + 3)); }
    return 20.0f * log10f(LOW_GAIN_RATIO * (i + 1));
}

uint8_t RX888mk2::hfVgaCode(int i) {
    i = std::clamp(i, 0, HF_VGA_STEPS - 1);
    if (i > GAIN_SWEET_POINT) { return AD8370_HIGH_MODE | (uint8_t)(i - GAIN_SWEET_POINT + 3); }
    return (uint8_t)(i + 1);
}

float RX888mk2::vhfRfGainDb(int i) {
    return VHF_RF_DB[std::clamp(i, 0, VHF_RF_STEPS - 1)];
}

float RX888mk2::vhfIfGainDb(int i) {
    return VHF_IF_DB[std::clamp(i, 0, VHF_IF_STEPS - 1)];
}

double RX888mk2::si5351OutputHz(uint32_t hz) {
    // Mirrors si5351aSetFrequencyA/B() in SDDC_FX3/driver/Si5351.c: an even
    // integer output divider and a fractional PLL with denominator 1048575
    // whose numerator is truncated, from the 27 MHz reference.
    const uint64_t xtal = 27000000;
    if (hz == 0) { return 0; }
    uint64_t f = hz;
    int rdiv = 1;
    while (f < 1000000) {
        f *= 2;
        rdiv *= 2;
    }
    uint64_t divider = 900000000ull / f;
    if (divider % 2) { divider--; }
    uint64_t pll = divider * f;
    uint64_t mult = pll / xtal;
    uint64_t num = (uint64_t)((double)(pll % xtal) * 1048575.0 / (double)xtal);
    double pllActual = (double)xtal * ((double)mult + (double)num / 1048575.0);
    return pllActual / (double)divider / rdiv;
}

double RX888mk2::r828dLoHz(uint64_t rfHz, double refHz) {
    // Mirrors r82xx_set_freq64() / r82xx_set_pll() in SDDC_FX3/driver/tuner_r82xx.c
    // with sideband = 0 (LO above RF), int_freq = 4.57 MHz and vco_algo = 0.
    // The firmware computes the PLL words for a nominal 16 MHz reference.
    const uint64_t pllRef = R828D_REF_HZ;
    uint64_t lo = rfHz + R828D_IF_HZ;
    uint64_t loKhz = (lo + 500) / 1000;
    const uint64_t vcoMin = 1770000, vcoMax = 2 * vcoMin; // kHz
    uint64_t mixDiv = 2;
    while (mixDiv <= 64) {
        if (loKhz * mixDiv >= vcoMin && loKhz * mixDiv < vcoMax) { break; }
        mixDiv <<= 1;
    }
    if (mixDiv > 64) { mixDiv = 64; }
    uint64_t vcoFreq = lo * mixDiv;
    uint64_t vcoDiv = (pllRef + 65536ull * vcoFreq) / (2 * pllRef); // nint * 65536 + sdm
    double frac = (double)vcoDiv;
    // The firmware leaves the sigma-delta dither on; measured on an RX888 mkII
    // this raises the VCO by a constant 1/4 SDM step (122.07 Hz at the VCO)
    // whenever the fractional part is in use.
    if (vcoDiv % 65536 != 0) { frac += 0.25; }
    double vcoActual = 2.0 * refHz * frac / 65536.0;
    return vcoActual / (double)mixDiv;
}

bool RX888mk2::open(const std::string& path, std::string& err) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    if (!usb.open(path, err)) { return false; }
    uint8_t info[4] = { 0 };
    if (!usb.vendorIn(TESTFX3, 0, 0, info, sizeof(info))) {
        err = "TESTFX3 failed: " + usb.lastError();
        usb.close();
        return false;
    }
    hwModel = info[0];
    fwVersion = (uint16_t)((info[1] << 8) | info[2]);
    sendU8(STOPFX3);
    gpio = 0; // ADC powered, everything else off
    curInput = Input::HF;
    return setGpioBits(0, false);
}

void RX888mk2::close() {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    if (!usb.isOpen()) { return; }
    usb.stopStreaming();
    sendU8(STOPFX3);
    if (curInput == Input::VHF) { sendU8(TUNERSTDBY); }
    // Power down the ADC and turn off the bias-tees, like ExtIO_sddc does on exit.
    gpio = GPIO_SHDWN;
    sendU32(GPIOFX3, gpio);
    usb.close();
}

bool RX888mk2::sendU8(Fx3Command cmd, uint8_t v) {
    return usb.vendorOut(cmd, 0, 0, &v, sizeof(v));
}

bool RX888mk2::sendU32(Fx3Command cmd, uint32_t v) {
    return usb.vendorOut(cmd, 0, 0, &v, sizeof(v));
}

bool RX888mk2::sendU64(Fx3Command cmd, uint64_t v) {
    return usb.vendorOut(cmd, 0, 0, &v, sizeof(v));
}

bool RX888mk2::setArg(ArgumentId id, uint16_t value) {
    uint8_t dummy = 0;
    return usb.vendorOut(SETARGFX3, value, id, &dummy, sizeof(dummy));
}

bool RX888mk2::setGpioBits(uint32_t mask, bool on) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    if (on) { gpio |= mask; }
    else { gpio &= ~mask; }
    return sendU32(GPIOFX3, gpio);
}

bool RX888mk2::setAdcRate(uint32_t hz) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    return sendU32(STARTADC, hz);
}

bool RX888mk2::setInput(Input in) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    if (in == Input::VHF) {
        // Park the HF path at maximum attenuation, then route the tuner to the ADC.
        setArg(ARG_DAT31_ATT, HF_ATT_STEPS - 1);
        if (!setGpioBits(GPIO_VHF_EN, true)) { return false; }
        // AD8370 high-gain mode, code 3 (ExtIO_sddc default for VHF)
        setArg(ARG_AD8340_VGA, AD8370_HIGH_MODE | 3);
        if (!sendU32(TUNERINIT, R828D_REF_HZ)) { return false; }
    }
    else {
        sendU8(TUNERSTDBY);
        if (!setGpioBits(GPIO_VHF_EN, false)) { return false; }
    }
    curInput = in;
    return true;
}

bool RX888mk2::setHfAttenuation(int steps) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    // PE4304 word is the attenuation in 0.5 dB units.
    return setArg(ARG_DAT31_ATT, (uint16_t)std::clamp(steps, 0, HF_ATT_STEPS - 1));
}

bool RX888mk2::setHfVga(int index) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    return setArg(ARG_AD8340_VGA, hfVgaCode(index));
}

bool RX888mk2::setIfVgaCode(uint8_t code) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    return setArg(ARG_AD8340_VGA, code);
}

bool RX888mk2::tune(uint64_t rfHz) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    return sendU64(TUNERTUNE, rfHz);
}

bool RX888mk2::setVhfRfGain(int index) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    return setArg(ARG_R82XX_ATTENUATOR, (uint16_t)std::clamp(index, 0, VHF_RF_STEPS - 1));
}

bool RX888mk2::setVhfIfGain(int index) {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    return setArg(ARG_R82XX_VGA, (uint16_t)std::clamp(index, 0, VHF_IF_STEPS - 1));
}

bool RX888mk2::setDither(bool on) { return setGpioBits(GPIO_DITH, on); }
bool RX888mk2::setRandomizer(bool on) { return setGpioBits(GPIO_RANDO, on); }
bool RX888mk2::setAdcHighGain(bool on) { return setGpioBits(GPIO_PGA_EN, !on); }
bool RX888mk2::setBiasTeeHf(bool on) { return setGpioBits(GPIO_BIAS_HF, on); }
bool RX888mk2::setBiasTeeVhf(bool on) { return setGpioBits(GPIO_BIAS_VHF, on); }
bool RX888mk2::setAdcPower(bool on) { return setGpioBits(GPIO_SHDWN, !on); }

bool RX888mk2::startAdc() {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    return sendU8(STARTFX3);
}

bool RX888mk2::stopAdc() {
    std::lock_guard<std::recursive_mutex> lck(mtx);
    return sendU8(STOPFX3);
}

} // namespace rx888
