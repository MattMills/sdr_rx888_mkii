// RX888 mkII radio control on top of the SDDC_FX3 firmware.
//
// Ported from ExtIO_sddc (Core/radio/RX888R2Radio.cpp, Core/RadioHandler.cpp),
// Copyright (c) 2017-2020 Oscar Steila ik1xpv, MIT License.
// See THIRD_PARTY_NOTICES.md.
#pragma once
#include "fx3_device.h"
#include "fx3_protocol.h"
#include <cstdint>
#include <mutex>
#include <string>

namespace rx888 {

class RX888mk2 {
public:
    enum class Input {
        HF,  // direct sampling: HF antenna -> PE4304 attenuator -> AD8370 VGA -> LTC2208
        VHF, // R828D tuner -> AD8370 -> LTC2208, tuner IF centred at R828D_IF_HZ
    };

    // R828D configuration used by the firmware (8 MHz IF filter).
    static constexpr uint32_t R828D_REF_HZ = 16000000;
    static constexpr uint32_t R828D_IF_HZ = 4570000;
    static constexpr uint64_t VHF_MIN_HZ = 24000000;
    static constexpr uint64_t VHF_MAX_HZ = 1750000000;

    // HF step attenuator (PE4304): 0..31.5 dB in 0.5 dB steps.
    static constexpr int HF_ATT_STEPS = 64;
    // HF VGA (AD8370), ExtIO_sddc index mapping: 0..126.
    static constexpr int HF_VGA_STEPS = 127;
    // R828D LNA+mixer gain index: 0..28; R828D IF VGA index: 0..15.
    static constexpr int VHF_RF_STEPS = 29;
    static constexpr int VHF_IF_STEPS = 16;

    static float hfVgaDb(int index);
    static uint8_t hfVgaCode(int index);
    static float vhfRfGainDb(int index);
    static float vhfIfGainDb(int index);

    // Exact LO the firmware's R828D PLL (vco_algo 0) will produce for an RF
    // frequency, with the tuner reference at refHz. The IF seen by the ADC
    // for a signal at rfHz is (LO - rfHz) and is spectrally inverted.
    static double r828dLoHz(uint64_t rfHz, double refHz);

    // Frequency the firmware's Si5351 driver actually produces when asked
    // for hz (relative to a perfect 27 MHz crystal).
    static double si5351OutputHz(uint32_t hz);

    bool open(const std::string& path, std::string& err);
    void close();
    bool isOpen() const { return usb.isOpen(); }
    Fx3Device& device() { return usb; }

    uint8_t model() const { return hwModel; }
    uint16_t firmwareVersion() const { return fwVersion; }

    bool setAdcRate(uint32_t hz); // blocks ~1 s (firmware waits for the clock)
    bool setInput(Input in);
    Input input() const { return curInput; }

    // HF path
    bool setHfAttenuation(int steps); // 0..63, attenuation = steps * 0.5 dB
    bool setHfVga(int index);         // 0..126
    // VHF path
    bool tune(uint64_t rfHz);
    bool setVhfRfGain(int index);     // 0..28
    bool setVhfIfGain(int index);     // 0..15
    bool setIfVgaCode(uint8_t code);  // raw AD8370 code (bit 7 = high gain mode)

    // ADC and misc
    bool setDither(bool on);
    bool setRandomizer(bool on);
    bool setAdcHighGain(bool on); // LTC2208 PGA: true = 1.5 Vpp range (+3.5 dB)
    bool setBiasTeeHf(bool on);
    bool setBiasTeeVhf(bool on);
    bool setAdcPower(bool on);

    bool startAdc(); // STARTFX3
    bool stopAdc();  // STOPFX3

    std::string lastError() const { return usb.lastError(); }

private:
    bool sendU8(Fx3Command cmd, uint8_t v = 0);
    bool sendU32(Fx3Command cmd, uint32_t v);
    bool sendU64(Fx3Command cmd, uint64_t v);
    bool setArg(ArgumentId id, uint16_t value);
    bool setGpioBits(uint32_t mask, bool on);

    Fx3Device usb;
    std::recursive_mutex mtx;
    uint32_t gpio = 0;
    Input curInput = Input::HF;
    uint8_t hwModel = 0;
    uint16_t fwVersion = 0;
};

} // namespace rx888
