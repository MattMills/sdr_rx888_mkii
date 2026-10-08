# OpenWebRX+ with the RX888 mkII

`rx888_connector` connects the RX888 mkII to [OpenWebRX+](https://github.com/luarvique/openwebrx).
It uses this repository's device and DDC code (`rx888_core`). The
[owrx_connector](https://github.com/jketterl/owrx_connector) framework supplies the command line,
the IQ socket and the control socket that OpenWebRX drives.

It is installed under the name `sddc_connector` as well. That lets OpenWebRX's existing
"SDDC" device type run it, so OpenWebRX itself needs no changes. Unlike jketterl's
`sddc_connector`, it needs no NVIDIA GPU: the DDC runs on the CPU, using about 2.5 cores
at a 128 MS/s ADC rate.

## Run it

```bash
cd openwebrx
docker compose up -d --build
```

Then open `http://<host>:8073/`.

The image builds on `slechev/openwebrxplus`. The connector is compiled in a stage derived
from that same image, so it links against the libraries that will run it, and the build
fails if `rx888_tool selftest` fails.

The compose file:
- passes the whole USB tree with a device cgroup rule, because the RX888 gets a new device
  node when its firmware loads;
- gives the container `SYS_NICE` and a high CPU weight, so OpenWebRX keeps up on a busy host.

The firmware is embedded in the connector; nothing has to be installed on the host.

`settings.json` seeds a new settings volume with the device and its profiles. A volume that
already holds settings keeps them. To log in to Settings, create an admin user (it prompts
for a password):

```bash
docker exec -it openwebrx-rx888 openwebrx admin adduser admin
```

## Sample rates and profiles

At or below 64 MHz the connector samples the HF input directly. The ADC runs at the highest
rate up to 128 MS/s for which `samp_rate = adc / 2^k` exactly, and the DDC tunes and
decimates.

Use power-of-two rates: 64, 32, 16, 8, 4, 2 or 1 MS/s. These keep the ADC at 128 MS/s,
where the HF input's 64 MHz low-pass filter prevents aliasing. Other rates run the ADC
slower, and signals between its Nyquist frequency and 64 MHz then alias into the band.

Above 64 MHz the connector uses the R828D tuner instead.

The ADC clock is requested at `nominal / (1 + ppm)`, so with the crystal error set as the
device's ppm, the delivered sample rate is exact to about 0.03 ppm. OpenWebRX places every
signal by that rate. Uncorrected, 10 ppm would put a signal 30 MHz from the centre 300 Hz
off.

| profile | span | rate | FFT bin |
|---|---|---|---|
| 0–32 MHz, all of HF (default) | 0.2–31.8 MHz | 32 MS/s | 488 Hz |
| band profiles | 1–4 MHz | 1–4 MS/s | 31–122 Hz |
| 0–64 MHz, HF + 6 m | 0.2–63.8 MHz | 64 MS/s | 1.95 kHz |

**The 64 MS/s profile is beyond what OpenWebRX handles smoothly.** The connector delivers it
without loss: 64.0 MS/s measured at the IQ socket, 0 dropped blocks. OpenWebRX's single
thread that reads the IQ socket then reaches about 93% of a core, and audio stutters. At
32 MS/s the busiest OpenWebRX thread uses about 50%.

The waterfall canvas is one pixel per FFT bin. Chrome draws 65536 bins but not 131072.

## Gain

The device or profile `rf_gain` takes one of:
- **dB.** On HF this is the AD8370 VGA gain, and below the VGA's minimum it engages the step
  attenuator. On VHF it is the R828D LNA gain.
- **`auto`.** HF: attenuator 0, VGA index 30. VHF: LNA 16, IF 8.
- **Explicit settings.** HF: `att=DB,vga=INDEX`. VHF: `rf=INDEX,if=INDEX`.

Every 30 s the connector logs the ADC peak, near-full-scale samples, dropped USB blocks and
USB errors.

To see whether noise comes in through the antenna, switch the attenuator in. If the floor
falls by the full attenuation, the noise arrives at the HF input; the receiver's own floor is
about −117 dBFS per 1 kHz bin.

## Notes

- **The device can be held by only one program.** Stop the container
  (`docker stop openwebrx-rx888`) before using `rx888_tool` or SDR++.
- **A previous session ended abruptly.** For example, a process killed mid-stream. The FX3
  can then time out every control request for some 20 s. The connector reopens the device,
  up to five times, rather than queueing timeouts. On SIGTERM it stops at the next step, so
  OpenWebRX does not have to kill it.
- **Standalone use.** Build with `-DRX888_BUILD_OWRX_CONNECTOR=ON`. This needs
  `libowrx-connector-dev` and `libcsdr-dev`, from the OpenWebRX+ package repository, plus
  pkg-config. Run `rx888_connector --help` for the options.
