# BDWidth Klipper Fork

This is a fork of [`markniu/bdwidth`](https://github.com/markniu/bdwidth) with
Klipper-side robustness and diagnostics added for USB BDWidth sensors.

The hardware, firmware files, CAD, and upstream documentation remain based on
the original project. This fork focuses on safer host behavior during long
prints.

## What This Fork Changes

- Hardens USB frame parsing for Klipper.
  - Accepts only the expected 5-byte width/motion frame with the newline
    terminator in the expected position.
  - Rejects malformed frames before they can update flow, motion, or runout
    state.
  - Fixes signed 16-bit motion conversion.

- Adds configurable plausible-width filtering.
  - Default discard range is `1.5mm` to `2.0mm`.
  - Values outside that range are ignored before they update BDWidth state.
  - This is intentionally stricter than a physical maximum filter because this
    setup treats those readings as sensor/serial outliers.

- Adds CCD outlier diagnostics.
  - On an outlier, the plugin can capture a CCD waveform from the same serial
    connection used by Klipper.
  - It saves `raw`, `csv`, and `json` diagnostic files immediately.
  - During a print, PNG graph rendering is deferred to reduce host load.
  - After printing returns to idle/ready, at most one deferred PNG is rendered.

- Prevents duplicate CSV logging handlers after reloads/restarts.

- Keeps the upstream `ENABLE_MOTION`, `ENABLE_WIDTH`, and `ENABLE_ALL` command
  behavior.

## When To Use This Fork

Use this fork if you run BDWidth through Klipper USB mode and want defensive
handling for occasional impossible width readings such as `0mm`, `2.2mm`, or
hundreds of millimeters.

If your sensor is clean and stable, the original project may be enough. This
fork is intended for printers where BDWidth data is useful but must not be
allowed to disturb long prints when a serial or optical outlier appears.

## Installation

```bash
cd ~
git clone https://github.com/SukbeomH/bdwidth.git
chmod +x ~/bdwidth/klipper/install.sh
~/bdwidth/klipper/install.sh
```

If you already installed the original project, switch the repository remote:

```bash
cd ~/bdwidth
git remote set-url origin https://github.com/SukbeomH/bdwidth.git
git remote add upstream https://github.com/markniu/bdwidth.git 2>/dev/null || true
git fetch origin main
git reset --hard origin/main
```

Then restart Klipper.

## Klipper Configuration

Example USB configuration:

```ini
[bdwidth fila_width_0]
port: usb

# Prefer by-path when multiple USB serial devices are present.
serial: /dev/serial/by-path/your-bdwidth-path
# serial: auto

default_nominal_filament_diameter: 1.75
enable: disable
extruder: extruder

min_diameter: 1.0
max_diameter: 2.0

runout_delay_length: 8.0
flowrate_adjust_length: 5
pause_on_runout: False
sample_time: 2
sensor_to_nozzle_length: 870

logging: True
debug_info: False

# Fork-specific hard discard range.
# Readings outside this range are not applied to BDWidth state.
min_plausible_diameter: 1.5
max_plausible_diameter: 2.0

# Fork-specific CCD diagnostics.
ccd_snapshot_on_outlier: True
ccd_snapshot_min_diameter: 1.5
ccd_snapshot_max_diameter: 2.0
ccd_snapshot_dir: ~/printer_data/logs/bdwidth_ccd_auto
ccd_snapshot_min_interval: 600
ccd_snapshot_frames: 3
ccd_snapshot_timeout: 2.0
ccd_snapshot_defer_png_during_print: True
ccd_snapshot_png_limit_per_print: 1
```

### Width Ranges

There are two related ranges:

- `min_diameter` / `max_diameter`: used by the existing flow/runout logic.
- `min_plausible_diameter` / `max_plausible_diameter`: fork-specific hard
  discard range. Values outside this range are captured for diagnostics, then
  ignored.

For this fork's default setup, both CCD diagnostic outliers and hard discard
outliers use:

```ini
min_plausible_diameter: 1.5
max_plausible_diameter: 2.0
ccd_snapshot_min_diameter: 1.5
ccd_snapshot_max_diameter: 2.0
```

## G-code Commands

```gcode
SET_BDWIDTH NAME=fila_width_0 COMMAND=ENABLE_ALL
SET_BDWIDTH NAME=fila_width_0 COMMAND=ENABLE_MOTION
SET_BDWIDTH NAME=fila_width_0 COMMAND=ENABLE_WIDTH
SET_BDWIDTH NAME=fila_width_0 COMMAND=DISABLE
SET_BDWIDTH NAME=fila_width_0 COMMAND=QUERY
```

`COMMAND=ENABLE` remains compatible and enables all functions. For setups where
width compensation is still being validated, prefer `ENABLE_MOTION` first.

## CCD Diagnostics

When `ccd_snapshot_on_outlier: True` is enabled, outlier readings can create
files under `ccd_snapshot_dir`.

During a print:

- `*.raw` stores the raw serial stream.
- `*.csv` stores the decoded CCD pixel amplitudes.
- `*.json` stores metadata such as trigger reason, width, raw width, and file
  paths.
- PNG rendering is skipped when
  `ccd_snapshot_defer_png_during_print: True`.

After the printer returns to idle/ready:

- At most `ccd_snapshot_png_limit_per_print` deferred PNG graph is rendered.
- The default is `1` to avoid post-print CPU spikes if many outliers occurred.

To inspect a deferred diagnostic manually, open the `*.json` and matching
`*.csv` in the snapshot directory. If `png_rendered` is `false`, the PNG was
intentionally deferred or skipped.

## Manual CCD Waveform Capture

The upstream `ccd_data/ccd_data.py` tool still works, but it needs exclusive
access to the BDWidth serial port. Klipper normally keeps that port open, so do
not run the tool against the same port while Klipper is active.

For manual offline inspection:

```bash
python3 ccd_data/ccd_data.py /dev/serial/by-path/your-bdwidth-path
```

Stop Klipper first, then restart Klipper after the test.

## Notes

- Keep `debug_info: False` for long prints unless actively debugging. It can
  generate a lot of console/log traffic.
- `logging: True` writes the BDWidth CSV log. This fork avoids duplicate file
  handlers after repeated reloads.
- CCD cleaning can improve optical waveform quality, but impossible values are
  usually handled better by frame validation and plausible-width filtering.

## Upstream Project

Original project and support resources:

- Repository: <https://github.com/markniu/bdwidth>
- Community and purchase links are maintained by the upstream project.
