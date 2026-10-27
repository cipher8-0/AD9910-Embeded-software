# RaptorWatch: AD9910 DDS Chirp Generator Firmware and Tools

Control firmware, host GUI and measurement analysis code for **RaptorWatch**, a DDS-based linear chirp generation module for an FMCW radar transmit chain. Final year project (EEE4022S), University of Cape Town.

> **Status:** This repository currently contains the software only. The PCB design files (schematic, layout, BOM, Gerbers) and measurement write-up will be added after the project has been assessed.

## Overview

Commercial off-the-shelf FMCW radar front ends offer limited control over chirp parameters. RaptorWatch is a custom board built around the Analog Devices **AD9910** DDS (1 GSPS) that gives full control of the transmit waveform without hardware changes. This repository holds the code that makes the board usable:

- **Firmware** for an STM32L476 processing board that drives the AD9910 over SPI and generates repeated chirp triggers from a hardware timer.
- **Python GUI** that talks to the processing board over a USB CDC virtual serial link and exposes DDS control (initialise, status, single tone, chirp load, trigger and hold).
- **Analysis notebook** that loads spectrum analyser captures (CSV export from a Keysight FieldFox) and produces the plots used in the report.

The wider project also includes a controlled comparison of reference clock architectures (onboard crystal via PLL, external 25 MHz via PLL, and ~1 GHz bypassing the PLL), measured by SFDR and phase noise. See [Limitations](#limitations) for what was and was not completed.

## System architecture

```
 PC (Python GUI)
      |  USB CDC (virtual COM port)
      v
 STM32L476 processing board
      |  SPI2 + IO_UPDATE + DRCTL (timer-driven) + control lines
      v
 AD9910 DDS board  ->  reconstruction filter  ->  balun  ->  SMA output
```

The firmware is split into layers so each can be tested independently:

1. **AD9910 driver:** register-level access over SPI, with a register abstraction so the rest of the code never handles raw bit fields.
2. **DDS command layer:** maps GUI commands (tone, chirp parameters, trigger, hold) onto driver calls. This layer has a dry-run mode, so the full command path can be tested on the processing board without the DDS board attached.
3. **USB CDC command interface:** parses commands from the PC and returns status and error responses.

Repeated sawtooth chirps are triggered by an STM32 hardware timer driving the AD9910 DRCTL pin, rather than by a per-chirp USB command. This keeps the chirp period set by the timer instead of by USB latency.

## Hardware interface (draft pin mapping)

| Signal | STM32 pin | Notes |
|---|---|---|
| SPI2 SCK / MISO / MOSI | PB13 / PB14 / PB15 | AD9910 serial port |
| SPI2 chip select | PB12 | |
| DRCTL | PE9 | TIM1_CH1, timer-driven chirp trigger |
| IO_UPDATE | PC3 | |

> This mapping reflects the current firmware. Check `[path/to/pin_config.h]` for the authoritative version.

## Repository layout

> Adjust this to match the actual folder structure.

```
.
├── firmware/            STM32L476 firmware (driver, command layer, USB CDC)
├── gui/                 Python host GUI
├── analysis/            Jupyter notebook and helper scripts for spectrum captures
├── data/                Example spectrum analyser CSV exports
└── docs/                Supporting notes
```

## Getting started

### Firmware

**Requirements:** `[toolchain, e.g. STM32CubeIDE / arm-none-eabi-gcc, version]`, an STM32L476-based processing board and an ST-LINK programmer.

```bash
# Build
[build command]

# Flash
[flash command or "Run > Debug in STM32CubeIDE"]
```

The processing board enumerates as a USB CDC serial device once flashed.

### Python GUI

**Requirements:** Python 3.`[x]` and the packages in `requirements.txt`.

```bash
pip install -r requirements.txt
python gui/[main_script].py
```

Select the board's COM port, then use the DDS tab to initialise the device, read status, output a single tone, or load and trigger a chirp.

### Analysis notebook

```bash
pip install jupyter numpy pandas matplotlib
jupyter notebook analysis/[notebook_name].ipynb
```

The notebook expects FieldFox CSV exports in `data/`. Update the file paths in the first cell to point at your own captures.

## Testing

- **Dry-run mode:** the DDS command layer passes its tests with no DDS board attached, verifying command parsing and the sequence of driver calls.
- **Hardware bring-up:** single tone, sawtooth and triangle chirps were confirmed on the assembled board using an oscilloscope.
- **Spectrum measurements:** captured with a Keysight FieldFox and analysed with the notebook in `analysis/`.

## Limitations

- The reference clock comparison could not be completed. The highest-frequency signal generator available was 15 MHz, so the external-reference arms (external 25 MHz via PLL, and ~1 GHz bypassing the PLL) could not be run. All measurements reported are for the onboard crystal via PLL configuration. The comparison is documented as a recommendation for further work.
- Oscilloscope bandwidth (50 MHz) limits what can be shown about chirp start and stop timing.
- The pin mapping and timer configuration may change if the processing board interface changes.

## Roadmap

- Add PCB design files and the final report after assessment.
- Run the reference clock architecture comparison once a suitable signal source is available.
- Possible extensions: non-linear FM waveforms, waveform agility, multi-channel synchronisation.

## Acknowledgements

- The STM32L476 processing board platform is by **Justin Pead**.
- Reference material: the AD9910 datasheet, the AD9910/PCBZ evaluation board and UG-207, and the Sinara Urukul open-hardware board.
- Project supervisor: `[supervisor name]`, University of Cape Town.

## Author

**Otsile Diale**, BSc Mechatronics Engineering, University of Cape Town.

## License

`[Choose a licence and add a LICENSE file, e.g. MIT.]` Check that no vendor-provided code (for example STM32 HAL or CubeMX-generated files) is being relicensed incorrectly; those files keep their original licences.