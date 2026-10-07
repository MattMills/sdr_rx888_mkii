// FX3 vendor-request protocol spoken by the SDDC_FX3 firmware.
//
// Ported from ExtIO_sddc (Interface.h), Copyright (c) 2017-2020 Oscar Steila
// ik1xpv, MIT License. See THIRD_PARTY_NOTICES.md.
#pragma once
#include <cstdint>

namespace rx888 {

// USB identifiers. The FX3 ROM bootloader enumerates as 04B4:00F3 until the
// SDDC firmware has been loaded into RAM; the firmware then re-enumerates as
// 04B4:00F1.
constexpr uint16_t FX3_VID = 0x04B4;
constexpr uint16_t FX3_PID_BOOTLOADER = 0x00F3;
constexpr uint16_t FX3_PID_STREAMER = 0x00F1;

// FX3 ROM bootloader "RW_INTERNAL" request used to write RAM / jump to entry.
constexpr uint8_t FX3_BOOT_RW_INTERNAL = 0xA0;

// Firmware vendor requests (bRequest).
enum Fx3Command : uint8_t {
    STARTFX3 = 0xAA,      // start GPIF engine and stream ADC data
    STOPFX3 = 0xAB,       // stop GPIF engine
    TESTFX3 = 0xAC,       // read 4 bytes: model, fw major, fw minor, request count
    GPIOFX3 = 0xAD,       // write uint32 GPIO word
    I2CWFX3 = 0xAE,       // I2C write: wValue = i2c addr, wIndex = register
    I2CRFX3 = 0xAF,       // I2C read:  wValue = i2c addr, wIndex = register
    RESETFX3 = 0xB1,      // reset FX3 back to the ROM bootloader
    STARTADC = 0xB2,      // write uint32 ADC clock frequency (Si5351 CLK0)
    TUNERINIT = 0xB4,     // write uint32 tuner reference frequency, init R82xx
    TUNERTUNE = 0xB5,     // write uint64 RF frequency
    SETARGFX3 = 0xB6,     // wIndex = argument id, wValue = argument value
    TUNERSTDBY = 0xB8,    // put tuner in standby
    READINFODEBUG = 0xBA, // read debug console text
};

// GPIO word bits (GPIOFX3). Mapping to FX3 pins is done by the firmware.
enum GpioBit : uint32_t {
    GPIO_SHDWN = 1u << 5,       // LTC2208 SHDN (1 = ADC powered down)
    GPIO_DITH = 1u << 6,        // LTC2208 DITH (internal dither)
    GPIO_RANDO = 1u << 7,       // LTC2208 RAND (output randomizer)
    GPIO_BIAS_HF = 1u << 8,     // HF input bias-tee
    GPIO_BIAS_VHF = 1u << 9,    // VHF input bias-tee
    GPIO_LED_YELLOW = 1u << 10,
    GPIO_LED_RED = 1u << 11,
    GPIO_LED_BLUE = 1u << 12,
    GPIO_ATT_SEL0 = 1u << 13,
    GPIO_ATT_SEL1 = 1u << 14,
    GPIO_VHF_EN = 1u << 15,     // RX888 mkII: route R828D tuner output to the ADC
    GPIO_PGA_EN = 1u << 16,     // RX888 mkII: firmware drives LTC2208 PGA pin = !PGA_EN
};

// SETARGFX3 argument ids (wIndex).
enum ArgumentId : uint16_t {
    ARG_R82XX_ATTENUATOR = 1, // R82xx combined LNA+mixer gain index, 0..28
    ARG_R82XX_VGA = 2,        // R82xx IF VGA gain index, 0..15
    ARG_R82XX_SIDEBAND = 3,   // R82xx sideband, 0/1
    ARG_R82XX_HARMONIC = 4,
    ARG_DAT31_ATT = 10,       // HF step attenuator (PE4304 on mkII), 0..63 in 0.5 dB
    ARG_AD8340_VGA = 11,      // HF/IF VGA (AD8370 on mkII), 8-bit code
    ARG_PRESELECTOR = 12,
    ARG_VHF_ATTENUATOR = 13,
};

// Hardware model byte returned by TESTFX3.
enum RadioModel : uint8_t {
    MODEL_NORADIO = 0x00,
    MODEL_BBRF103 = 0x01,
    MODEL_HF103 = 0x02,
    MODEL_RX888 = 0x03,
    MODEL_RX888R2 = 0x04, // RX888 mkII
    MODEL_RX999 = 0x05,
    MODEL_RXLUCY = 0x06,
    MODEL_RX888R3 = 0x07,
};

inline const char* modelName(uint8_t model) {
    switch (model) {
    case MODEL_BBRF103: return "BBRF103";
    case MODEL_HF103: return "HF103";
    case MODEL_RX888: return "RX888";
    case MODEL_RX888R2: return "RX888 mkII";
    case MODEL_RX999: return "RX999";
    case MODEL_RXLUCY: return "RX Lucy";
    case MODEL_RX888R3: return "RX888 mkIII";
    default: return "Unknown";
    }
}

} // namespace rx888
