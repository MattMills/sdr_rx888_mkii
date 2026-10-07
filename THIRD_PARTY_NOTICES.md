# Third-party notices

This project is licensed under the GNU General Public License v3.0 (see
`LICENSE`). It is a plugin for [SDR++](https://github.com/AlexandreRouma/SDRPlusPlus),
which is GPL-3.0, and links against its core library.

It contains code and data derived from the projects below.

## ExtIO_sddc (MIT)

Source: <https://github.com/ik1xpv/ExtIO_sddc> (commit `331b35c`, March 2026).

Ported/adapted from ExtIO_sddc:

| This project | ExtIO_sddc origin |
|---|---|
| `src/device/fx3_protocol.h` | `Interface.h` (vendor requests, GPIO bits, argument ids, model ids) |
| `src/device/rx888_mk2.cpp` | `Core/radio/RX888R2Radio.cpp`, `Core/RadioHandler.cpp` (mode switching, attenuator/VGA mapping, R828D gain tables) |
| `src/device/rx888_mk2.cpp` (`r828dLoHz`, `si5351OutputHz`) | models of `SDDC_FX3/driver/tuner_r82xx.c` and `SDDC_FX3/driver/Si5351.c` |
| `src/dsp/r2iq.*` | `Core/fft_mt_r2iq*.cpp` (FFT overlap-save real-to-IQ DDC) |
| `src/dsp/kaiser.*` | `Core/fir.cpp` |
| `firmware/SDDC_FX3.img` | `SDDC_FX3.img` (prebuilt FX3 firmware, v2.02) |

```
The MIT License (MIT)

Copyright (c) 2017-2020 Oscar Steila ik1xpv<at>gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

The firmware image was built by the ExtIO_sddc project from its `SDDC_FX3`
sources with the Cypress (Infineon) EZ-USB FX3 SDK; it runs only on the FX3
in SDDC receivers.

## libusb backend references (GPL)

`src/device/fx3_device.cpp` was written for this project. Its firmware upload
and asynchronous streaming flow follow ExtIO_sddc's Linux libusb backend
(`Core/arch/linux/usb_device.c`, `streaming.c`, Copyright (C) 2020 Franco
Venturi, GPL-3.0-or-later) and fxload's `fx3_load_ram()` (`ezusb.c`,
GPL-2.0-or-later). Both are compatible with this project's GPL-3.0 license.

## Runtime dependencies (not included in source)

- libusb-1.0 (LGPL-2.1)
- FFTW 3 (GPL-2.0-or-later)
- SDR++ core and VOLK (GPL-3.0), as provided by the SDR++ installation
