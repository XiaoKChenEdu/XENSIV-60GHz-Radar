# XENSIV-60GHz-Radar Setup Guide

Firmware, configuration, and tooling for capturing raw frames from an Infineon XENSIV™ BGT60TR13C presence radar connected to a `CYSBSYSKIT-DEV-01` Rapid IoT Connect Developer Kit. Use this document to install the required tooling, fetch dependencies, program the kit, and capture data over UART.

## Hardware You Need

- `CYSBSYSKIT-DEV-01` base board with KitProg3 updated to v2.40 or newer
- XENSIV™ BGT60TR13C radar wing board (or equivalent) wired to the board SPI pins (`CYBSP_SPI_*`, `CYBSP_GPIO10`, `CYBSP_GPIO11`, `CYBSP_GPIO5`)
- Micro-USB cable for power/programming
- Host PC (Windows, macOS, or Linux) with a free USB port

## Software Prerequisites

- [ModusToolbox](https://www.infineon.com/modustoolbox) 3.2 or newer (includes GCC Arm toolchain and ModusToolbox shell)
- Git 2.35+ for cloning the repository
- [uv](https://docs.astral.sh/uv/getting-started/installation/) (used by `data_test/serial_logger.py`)
- A serial terminal or logging script capable of 2000000-8-N-1 (e.g. PuTTY, TeraTerm, `serial_logger.py`)

## Clone and Prepare the Project

1. Open a shell/terminal
2. Clone and enter the repository:
   ```sh
   git clone https://github.com/XiaoKChenEdu/XENSIV-60GHz-Radar.git XENSIV-60GHz-Radar
   cd XENSIV-60GHz-Radar
   ```
3. Fetch all required ModusToolbox dependencies (BSP, HAL, radar SDK, etc.):
   ```sh
   # On Windows, use the ModusToolbox shell for full compatibility!
   # This command downloads and synchronizes all dependent libraries.
   make getlibs
   ```
   > **Tip:** If you encounter issues on Windows, launch the "(modus-shell) ModusToolbox Command Prompt" from the Start menu, navigate to this directory, and rerun `make getlibs`.
   This populates the shared library cache at `../mtb_shared` and ensures the local `libs/` directory is in sync.

## Build and Program the Firmware

1. Connect the `CYSBSYSKIT-DEV-01` via USB. Ensure the KitProg3 mode switch is set to `DAPLink`.
2. (Optional) Erase any existing protection on the QSPI flash using the Infineon Firmware Loader.
3. Build the project (default is `Debug`, GCC toolchain):
   ```sh
   make build
   ```
   - For a release image: `make build CONFIG=Release`
4. Program the kit:
   ```sh
   make program
   ```
   The KitProg3 port enumerates as a CDC UART at 115200 baud. Programming also resets the MCU.

## Verify the Serial Interface

1. Open your preferred serial terminal or run the logging script (see below) on the enumerated COM/tty device at 115200 baud.
2. On reset, the firmware prints:
   ```
   XENSIV BGT60TRxx Example
   Ready. Type 'start' [frames] or 'stop' followed by Enter.
   ```
3. Enter commands in the terminal:
   - `start` — begin continuous frame capture
   - `start <n>` — capture exactly `<n>` frames
   - `stop` — end the current capture session

## Log Raw Frames to Disk

The repository ships with a small helper to automate UART capture:
```sh
python data_test/serial_logger.py --port COM8 --output frames.log --frames 10 --stop-on-exit
```
- Use `--frames 0` or omit `--frames` for continuous acquisition; interrupt with `Ctrl+C`.
- The script stops on the firmware’s “Capture completed”/“Capture stopped.” message when a finite frame count is requested.
- Adjust `--baud` if you change `CY_RETARGET_IO_BAUDRATE` in firmware.

## Adjusting Radar Settings

- `presence_radar_settings.h` holds the BGT60TR13C register list generated with the XENSIV configurator. Export new register sets from the configurator and update the header to change chirp parameters, frame repetition, or antenna configuration.
- Update `NUM_SAMPLES_PER_FRAME  ` in `main.c` if you alter the number of antennas, chirps, or samples per chirp.

## Repository Layout

- `src/main.c` – firmware entry point, CLI, and data acquisition loop
- `src/presence_radar_settings.h` – generated radar register configuration
- `data_test/serial_logger.py` – Python helper to capture UART output to a file
- `reference/radar_sdk/` – upstream Infineon radar SDK (for reference examples and documentation)
- `bsps/` – ModusToolbox board support package for `CYSBSYSKIT-DEV-01`

## Updating Libraries or Tool Versions

- Re-run `make getlibs` any time `libs/*.mtb` changes.
- Delete `../mtb_shared` (optional) if you want to force a clean re-fetch of shared libraries.
- Update ModusToolbox using Infineon’s installer when new tool versions are required.
