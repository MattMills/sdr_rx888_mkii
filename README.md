# sdr_rx888_mkii

An [SDR++](https://github.com/AlexandreRouma/SDRPlusPlus) source module for the
**RX888 mkII**, built out-of-tree against the SDR++ core. It supports both
inputs:

- **HF** – direct sampling: the HF antenna port goes through the PE4304 step
  attenuator and the AD8370 VGA straight into the 16-bit LTC2208 ADC;
- **VHF/UHF** – the R828D tuner (24 – 1750 MHz) with its 4.57 MHz IF digitised
  by the same ADC.

The device code is ported from [ExtIO_sddc](https://github.com/ik1xpv/ExtIO_sddc)
(MIT, Oscar Steila IK1XPV and contributors): the FX3 firmware protocol and
firmware image, RX888 mkII control and the FFT overlap-save real-to-IQ
converter. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Features

- Loads the SDDC FX3 firmware automatically (embedded in the plugin) when the
  receiver is still in its USB boot loader; no ExtIO/Cypress driver needed –
  plain libusb/WinUSB.
- Input selection (HF / VHF-UHF) with per-input controls:
  - HF: RF attenuator 0 – 31.5 dB (0.5 dB steps), IF gain (AD8370, −24.6 to
    +33 dB), HF bias-tee.
  - VHF/UHF: R828D LNA+mixer gain (0 – 49.6 dB), R828D IF gain (−4.7 to
    +40.8 dB), AD8370 IF VGA, VHF bias-tee.
  - ADC: PGA (+3.5 dB, 1.5 Vpp range), dither, output randomizer (decoded in
    software), DC-offset removal.
- ADC clock 16 / 20 / 32 / 64 / 128 MS/s; output bandwidth ADC/2 down to
  ADC/128 (e.g. 10 MHz … 156 kHz at 20 MS/s, 32 MHz … 500 kHz at 64 MS/s).
- Real-to-complex conversion that keeps only positive frequencies (no mirror
  image), upright spectrum on both inputs (the R828D's inverted IF is
  conjugated), 1-bin coarse tuning plus a fine NCO so the centre frequency is
  exact.
- Exact frequency model of the firmware's clocking: Si5351 fractional
  synthesis of the ADC and tuner clocks and the R828D PLL including its
  sigma-delta dither offset, plus a ppb crystal correction. SDR++ is given
  the exact corrected sample rate, so the whole waterfall is on frequency.
- Live status: firmware version, USB link speed, measured USB sample rate
  (shows lost samples), ADC peak level and clipping counter.

## Status – what was verified on hardware

Tested on Windows 11 (Intel i7-8705G) with an RX888 mkII, SDR++ built from
source at commit `8c9f5ee8`, using the module inside SDR++ and the
`rx888_tool` utility that shares its code.

| Check | Result |
|---|---|
| Enumeration, firmware upload, re-enumeration | 04B4:00F3 → firmware 2.02 → 04B4:00F1, model 0x04 (RX888 mkII) |
| USB 2.0 High-Speed link (the only link available on the test PC) | 42.4 MB/s max = 21.2 MS/s; 16 and 20 MS/s sustained with 0 drops / 0 errors |
| Sample format | 16-bit little-endian two's complement: 160 M samples with the input attenuated stayed within ±45 LSB of the mean (no byte/sign errors) |
| Randomizer | RAND on: raw mean −0.4 (scrambled); decoded mean/RMS identical to RAND off (−177.7 / 9.24 LSB vs −178.1 / 9.23) |
| ADC PGA | +3.4 … +3.6 dB, as labelled |
| HF gain chain | ADC level follows the VGA index; 31.5 dB attenuation lowers it by 29 dB |
| HF frequency | WWV 5 MHz at −3.1 Hz; AM broadcast carriers on their 10 kHz channels (990 kHz +6 Hz, 1510 kHz −2 Hz) |
| VHF/UHF frequency | Internal clock harmonics within 0.05 Hz of the model; 580 MHz ADC-clock harmonic exactly on frequency inside SDR++ |
| Spectral orientation / no image | ATSC pilots at the *lower* edge of their 6 MHz channels, 39 – 43 dB above the empty mirror position; HF mirror positions 40 – 50 dB down; synthetic test: images 92 – 125 dB down |
| DDC self-test (`rx888_tool selftest`) | 40/40: tone frequency ≤ 0.4 Hz, level ±0.5 dB, odd-bin tuning, inverted path, 1 vs 4 threads bit-identical |
| In SDR++ | module loads, uploads firmware, streams, tunes via rigctl, records baseband without gaps; the UI renders and shows live status. Gain/attenuator setters were verified through `rx888_tool`; changing controls in the GUI while running (including input/clock/bandwidth switching, which restarts the stream) has not been exercised by automation |

**Not verified (no USB 3 link on the test PC):** ADC clocks of 32, 64 and
128 MS/s over USB. The DSP was benchmarked offline at 144 – 195 MS/s per core
(i7-8705G, AVX2 FFTW), so 128 MS/s is within reach, but streaming at those
rates is untested. The macOS and Linux packages are built and DSP-self-tested
by CI but have not been run with hardware.

### Known limitations

- **USB 2.0:** at most ~20 MS/s. That is enough for the VHF/UHF path (the
  R828D IF is ~8 MHz wide) but limits HF to 0 – 10 MHz, and **HF signals
  above the ADC Nyquist frequency alias into the band** at these low clocks
  (the HF input is only low-pass filtered for the full-rate clock) – use an
  external low-pass filter for HF on USB 2.0. Use a USB 3 port *and* a USB 3
  rated cable for 64/128 MS/s.
- **Frequency accuracy is limited by the RX888's 27 MHz crystal.** The test
  unit read +7.5 … +9.6 ppm against ATSC pilots, moving by ~1 ppm between
  sessions (the ADC is powered down when SDR++ stops). Set **Freq. corr.
  (ppb)** from a known signal; residual drift remains.
- The firmware (SDDC_FX3 v2.02) leaves the R828D IF filter at 8 MHz and its
  LNA/mixer are set together (ExtIO_sddc gain table).
- One RX888 per SDR++ instance. Android is not supported (SDR++ plugins must
  be compiled into the APK).
- The SDR++ module ABI is not stable: a plugin binary works with SDR++ builds
  from about the same SDR++ commit it was compiled against (recorded in
  `BUILD_INFO.txt` of each release package).

## Installing

Release packages are built by CI for Windows x64, macOS (Intel/ARM) and
Debian/Ubuntu (amd64/aarch64), matching the SDR++ nightly targets. Use a
package built against an SDR++ version close to yours.

### Windows

1. **USB driver (Zadig), once per PC – needs administrator rights.** Windows
   has no driver for the RX888's Cypress FX3 controller, which shows up under
   two USB IDs:
   - `04B4:00F3` "WestBridge" – the FX3 boot loader, after plugging in;
   - `04B4:00F1` "RX888mk2" – after the firmware has been loaded.

   1. Download Zadig from <https://zadig.akeo.ie/> and run it.
   2. Plug in the RX888. In Zadig choose *Options → List All Devices*, select
      **WestBridge** (USB ID `04B4 00F3`), set the driver to **WinUSB** and
      click **Install Driver** (or *Replace Driver*).
   3. *Device → Create New Device*: name `RX888mk2`, USB ID `04B4` `00F1`
      (leave the third box empty), driver **WinUSB**, **Install Driver**.
      This pre-installs the driver for the firmware device.
      Alternatively, start SDR++ with the RX888 once (it uploads the firmware
      and then reports the missing driver), and in Zadig select **RX888mk2**
      (`04B4 00F1`) and install WinUSB.

   Device Manager should then list *WestBridge* or *RX888mk2* under
   "Universal Serial Bus devices" without a warning sign. If the 04B4:00F1
   device is bound to the Cypress CyUSB3 driver from ExtIO_sddc/HDSDR,
   replace it with WinUSB the same way.
2. Copy `modules\rx888_mkii_source.dll` from the release zip into the
   `modules` folder of SDR++.
3. Start SDR++ and choose **RX888 mkII** in the Source menu. SDR++ only
   creates the module instances listed in its `config.json`, so on first start
   the plugin adds an `RX888 mkII Source` instance there itself; no Module
   Manager step is needed. To hide it, disable that instance in Module Manager
   (a deleted instance is re-added on the next start).

### Linux (Debian/Ubuntu)

```bash
sudo apt install ./rx888_mkii_source_<distro>_<arch>.deb
```

The package installs the plugin to `/usr/lib/sdrpp/plugins`, `rx888_tool` to
`/usr/bin` and a udev rule giving the logged-in user access to the receiver
(replug it after installing).

### macOS

See `INSTALL.txt` in the macOS zip: copy the plugin into
`SDR++.app/Contents/Plugins`, its libraries into `Contents/Frameworks`,
and re-sign the app ad hoc.

## Using the module

- **Input**: HF (direct sampling) or VHF/UHF (R828D). Switching moves the
  tuner to the last frequency used on that input if the current one is out of
  range.
- **ADC clock** and **Bandwidth** (output sample rate) can be changed while
  running; the stream restarts (~1 s, the firmware waits for the clock).
- **ADC peak** shows the highest ADC level in the last second; reduce gain or
  add attenuation when it reports CLIPPING.
- **USB: x MS/s of y**: if the measured rate is below the ADC clock (red),
  the USB link is too slow and samples are being dropped.
- **Freq. corr. (ppb)**: crystal correction; for example +9040 ppb for a
  27 MHz crystal that runs 9.04 ppm fast.

Settings are stored per device in `rx888_mkii_config.json` in the SDR++ root.

## Building from source

The plugin is an out-of-tree CMake project that imports `sdrpp_core` from an
existing SDR++ build tree. Build SDR++ first (its core is enough), then point
this project at it:

- `SDRPP_SOURCE_DIR` – the SDRPlusPlus source tree;
- `SDRPP_BUILD_DIR` – its CMake build tree (contains `core/…/sdrpp_core`).

Without `SDRPP_SOURCE_DIR` only `rx888_tool` is built.

### Windows (Visual Studio 2022)

Dependencies via [vcpkg](https://github.com/microsoft/vcpkg); volk from
PothosSDR (as SDR++'s own CI) or built from source. Example with everything
under `C:\dev`:

```powershell
git clone https://github.com/microsoft/vcpkg C:\dev\vcpkg; C:\dev\vcpkg\bootstrap-vcpkg.bat
C:\dev\vcpkg\vcpkg install fftw3:x64-windows glfw3:x64-windows zstd:x64-windows libusb:x64-windows rtaudio:x64-windows
C:\dev\vcpkg\vcpkg install "fftw3[avx2]:x64-windows-static-md"   # FFTW linked into the plugin
```

volk (needs Python with `mako`: `py -m pip install --user mako`):

```powershell
git clone --recursive --branch v3.1.2 https://github.com/gnuradio/volk C:\dev\volk
cmake -S C:\dev\volk -B C:\dev\volk\build -G "Visual Studio 17 2022" -A x64 -DCMAKE_INSTALL_PREFIX=C:\dev\install -DENABLE_TESTING=OFF
cmake --build C:\dev\volk\build --config Release; cmake --install C:\dev\volk\build --config Release
```

SDR++ (from PowerShell, so the `/I` flags are passed through unchanged):

```powershell
$env:CXXFLAGS = "/IC:/dev/install/include /IC:/dev/vcpkg/installed/x64-windows/include/rtaudio"
$env:CFLAGS = "/IC:/dev/install/include"
$env:LDFLAGS = "/LIBPATH:C:/dev/install/lib /LIBPATH:C:/dev/vcpkg/installed/x64-windows/lib"
git clone https://github.com/AlexandreRouma/SDRPlusPlus C:\dev\SDRPlusPlus
cmake -S C:\dev\SDRPlusPlus -B C:\dev\SDRPlusPlus\build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_TOOLCHAIN_FILE=C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DOPT_BUILD_AIRSPY_SOURCE=OFF -DOPT_BUILD_AIRSPYHF_SOURCE=OFF -DOPT_BUILD_HACKRF_SOURCE=OFF `
  -DOPT_BUILD_PLUTOSDR_SOURCE=OFF -DOPT_BUILD_RTL_SDR_SOURCE=OFF -DOPT_BUILD_DISCORD_PRESENCE=OFF -DOPT_BUILD_AUDIO_SOURCE=OFF
cmake --build C:\dev\SDRPlusPlus\build --config Release
```

(The `do_always_volk` step fails without PothosSDR – it only copies
`volk.dll`; build the remaining targets with `--target`, or ignore it.)

This plugin:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE=C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DSDRPP_SOURCE_DIR=C:/dev/SDRPlusPlus -DSDRPP_BUILD_DIR=C:/dev/SDRPlusPlus/build `
  -DCMAKE_PREFIX_PATH=C:/dev/install `
  -DFFTW3f_DIR=C:/dev/vcpkg/installed/x64-windows-static-md/share/fftw3f
cmake --build build --config Release
```

`scripts\make_sdrpp_bundle.ps1` assembles a runnable SDR++ folder (SDR++
build + DLLs + this plugin), e.g.
`.\scripts\make_sdrpp_bundle.ps1 -SdrppSource C:\dev\SDRPlusPlus -SdrppBuild C:\dev\SDRPlusPlus\build -VcpkgInstalled C:\dev\vcpkg\installed\x64-windows -VolkBin C:\dev\install\bin -Out C:\dev\sdrpp_rx888`,
then run `C:\dev\sdrpp_rx888\sdrpp.exe` (or `sdrpp.exe -r C:\dev\sdrpp_rx888` from any
directory). The script writes `config.json` with absolute module and resource
paths and an RX888 mkII instance. It does this because SDR++'s Windows defaults are
relative to the working directory and do not list this plugin. Close SDR++ normally:
killing it while it saves can leave `config.json` empty, and SDR++ then falls back
to those defaults. Re-run the script to repair it.

### Linux

```bash
sudo apt install build-essential cmake git pkg-config libfftw3-dev libglfw3-dev libvolk-dev libzstd-dev libusb-1.0-0-dev
git clone https://github.com/AlexandreRouma/SDRPlusPlus
cmake -S SDRPlusPlus -B SDRPlusPlus/build -DCMAKE_BUILD_TYPE=Release -DOPT_BUILD_AIRSPY_SOURCE=OFF -DOPT_BUILD_AIRSPYHF_SOURCE=OFF \
  -DOPT_BUILD_HACKRF_SOURCE=OFF -DOPT_BUILD_PLUTOSDR_SOURCE=OFF -DOPT_BUILD_RTL_SDR_SOURCE=OFF -DOPT_BUILD_AUDIO_SOURCE=OFF \
  -DOPT_BUILD_AUDIO_SINK=OFF -DOPT_BUILD_DISCORD_PRESENCE=OFF
cmake --build SDRPlusPlus/build --target sdrpp_core -j
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSDRPP_SOURCE_DIR=$PWD/SDRPlusPlus -DSDRPP_BUILD_DIR=$PWD/SDRPlusPlus/build
cmake --build build -j && sudo cmake --install build --prefix /usr
```

(On older releases the volk package is `libvolk2-dev`.) macOS works the same
way with Homebrew (`brew install libusb fftw glfw zstd pkg-config`) and volk
built from source – see `.github/workflows/build.yml`.

## rx888_tool

Command-line utility built with the same device and DSP code:

| Command | What it does |
|---|---|
| `rx888_tool list` | list FX3/SDDC devices, mode, USB speed, driver state |
| `rx888_tool fwload` | upload the embedded firmware |
| `rx888_tool info` | load firmware if needed; print model, firmware version, link speed |
| `rx888_tool stream --adc 20000000 --seconds 5 [--ddc 2]` | measure USB throughput (and DDC load) |
| `rx888_tool capture --out x --mode hf --adc 20000000 --freq 5000000 --decim 6 --raw` | capture raw ADC (`x.s16`) and DDC output (`x.cf32`), print ADC statistics |
| `rx888_tool scan --mode vhf --start 470e6 --stop 610e6` | step the tuner across a range and list spectral peaks |
| `rx888_tool selftest` | synthetic DDC checks (frequency, level, image, threads) |
| `rx888_tool bench [--threads N]` | DDC throughput |

## How it works

The ADC streams real 16-bit samples over USB bulk transfers (128 KiB, 16 in
flight). A DSP thread converts them to float (undoing the randomizer and the
DC offset) and runs ExtIO_sddc's fast-convolution DDC: 8192-point real FFTs
with 25 % overlap, a circular bin shift as the mixer, a 1025-tap Kaiser
low-pass applied in the frequency domain, and a shorter inverse FFT that
decimates; overlap-save keeps 3/4 of each block. Bins below 0 Hz and above
Nyquist are zeroed, so the analytic signal has no image. Coarse tuning is
one bin (ADC/8192) with a per-frame phase correction; a fine NCO removes the
remainder. On the VHF path the tuner's LO sits 4.57 MHz above the RF, which
inverts the spectrum; the DDC tunes to the exact IF predicted by a model of
the firmware's PLL code and conjugates the output.

## License

GPL-3.0 (see [LICENSE](LICENSE)), as required for an SDR++ plugin. Portions
are derived from ExtIO_sddc under the MIT license – see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
