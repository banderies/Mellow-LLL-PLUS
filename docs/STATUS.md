# Mellow LLL Plus - Pin & Header Analysis

## Current Wiring (VZ330 Integration)

The board currently uses three connections to the VZ330 controller:

| Wire | LLL Pin | Purpose |
|------|---------|---------|
| Power | 12-24V | Board power supply |
| Ground | GND | Common ground |
| Filament status | PB15 (`DUANLIAO`) | Signals filament presence/absence back to the VZ330 controller |

## Broken-Out Header Pins

### Top Header (Power / Status) - 4 pins

| Pin | Firmware Name | Mode | In Use? | Details |
|-----|--------------|------|---------|---------|
| 12-24V | — | Power | Yes | Board power supply |
| GND | — | Power | Yes | Common ground |
| PB15 | `DUANLIAO` / `DULIAO` | OUTPUT | Yes | Filament break/blockage signal to VZ330 |
| PB14 | `DISTAL_SWITCH` | INPUT_PULLUP | **Yes (custom)** | Repurposed as distal filament switch (hotend side of extruder gears). Was `EXTENSION_PIN7` (unused in stock firmware). |

### 6-Pin Header (MDM / Encoder) - 6 pins

These pins are reserved for the optional MDM blockage detection module. The firmware checks for MDM at boot via `Check_Connet_MDM()` on PA4. **If no MDM module is connected, these pins are left unconfigured after boot** (high impedance).

| Pin | Firmware Name | Mode (no MDM) | Mode (with MDM) | Details |
|-----|--------------|---------------|-----------------|---------|
| 5V | — | Power | Power | 5V supply rail |
| GND | — | Power | Power | Common ground |
| PA4 | `EXTENSION_PIN6` / `MDM_DPIN` | Briefly read at boot, then idle | INPUT | MDM filament detection input. Briefly tested during MDM detection, left alone if not connected |
| PA5 | `EXTENSION_PIN5` / `PULSE1_PIN` | Unconfigured | TIM2 CH1 input | Receives pulses from controller for blockage distance calculation |
| PB10 | `EXTENSION_PIN4` / `PULSE2_PIN` | Unconfigured | INPUT + interrupt | Receives pulses from MDM encoder module |
| PB11 | `EXTENSION_PIN3` / direction | Unconfigured | INPUT + interrupt | Extrusion direction signal (1 = extrude, 0 = retract) |

The schematic shows 10K pull-up resistors on these ENCODE pins, so a connected switch will read HIGH (open) / LOW (closed) without needing an external pull-up.

### 3-Pin Header (Signal Output) - 3 pins

**WARNING: PA2 and PA3 are actively driven as outputs.** Do not use for switch input.

| Pin | Firmware Name | Mode | In Use? | Details |
|-----|--------------|------|---------|---------|
| GND | — | Power | — | Common ground |
| PA2 | `EXTENSION_PIN1` | OUTPUT, driven LOW | Yes | Toggled HIGH on KEY2 single-press (signals controller) |
| PA3 | `EXTENSION_PIN2` | OUTPUT, driven HIGH | Yes | Toggled LOW on KEY1 single-press (signals controller) |

### 2-Pin Headers (Signal Control) - 2 x 2 pins

These are actively read by the firmware. Pulling them LOW triggers motor actions.

| Pin | Firmware Name | Mode | Details |
|-----|--------------|------|---------|
| GND | — | Power | Common ground |
| PB5 | `FRONT_SIGNAL_PIN` | INPUT_PULLUP | Polled in main loop; LOW triggers **forward feed** |
| GND | — | Power | Common ground |
| PB6 | `BACK_SIGNAL_PIN` | INPUT_PULLUP | Polled in main loop; LOW triggers **reverse feed** |

### Internal / Unlabeled Connectors

These connectors are present on the board but are occupied by internal sensors or are not suitable for signal use:

| Connector | Pins | Function | Notes |
|-----------|------|----------|-------|
| Motor (JP4) | 4 | A+, A-, B+, B- stepper coils | Occupied by stepper motor |
| HALL1 | 2 | PB2 + GND/3V3 | Hall effect sensor (buffer position 3) — occupied |
| HALL2 | 2 | PB3 + GND/3V3 | Hall effect sensor (buffer position 2) — occupied |
| HALL3 | 2 | PB4 + GND/3V3 | Hall effect sensor (buffer position 1) — occupied |
| Filament switch | 2 | PB7 (ENDSTOP_3) + GND | "Limit position" connector — occupied by existing proximal filament switch |
| TEMP1 | 2-3 | PA1 (NTC) + GND | Temperature sensor input with voltage divider |
| Debug/SWD | 5-6 | SWDIO, SWCLK, 3V3, GND, NRST | Programming header — not suitable for signal use |

## Distal Limit Switch — Firmware Enhancement Plan

### Problem

During automated filament unloading, the extruder retracts filament back toward the buffer. The stock LLL Plus has a single filament switch (PB7, proximal to the extruder gears) that detects filament presence. When the extruder retracts, the filament tail remains across this switch even after it has cleared the extruder gears. The LLL firmware never sees "filament absent" and therefore cannot determine that unloading is complete. Automated unload fails.

### Solution: Add a Distal Limit Switch

A second limit switch is added to the filament path **distal** to the extruder gears (i.e., on the far side, between the gears and the hotend). This creates two sensing points:

```
                     Filament path direction (loading -->)

    [Buffer] ----> [Proximal Switch (PB7)] ----> [Extruder Gears] ----> [Distal Switch (PB14)] ----> [Hotend]
                    (existing, stock)              (drive mechanism)      (new, custom)
```

### Sensor Terminology

| Switch | Pin | Position | Firmware Name |
|--------|-----|----------|---------------|
| **Proximal** | PB7 | Before extruder gears (buffer side) | `ENDSTOP_3` (existing) |
| **Distal** | PB14 | After extruder gears (hotend side) | `DISTAL_SWITCH` (custom) |

Both are normally-open (NO) switches: LOW = filament present (switch closed), HIGH = filament absent (switch open). PB14 is configured as INPUT_PULLUP in firmware.

### New Filament States

The two switches together define four meaningful states:

| Proximal (PB7) | Distal (PB14) | Filament Position | Action |
|----------------|--------------|-------------------|--------|
| Open (absent) | Open (absent) | **No filament** — filament is fully retracted into the buffer or not loaded | Normal buffer operation (existing logic) |
| Closed (present) | Open (absent) | **Filament parked in gears** — filament tip is between the two switches, captured by the extruder gears | **Stop motor** — unload is complete, filament is parked and ready for next load |
| Closed (present) | Closed (present) | **Filament loaded** — filament extends through gears to hotend | Normal loaded state |
| Open (absent) | Closed (present) | **Anomalous** — filament past gears but not triggering proximal switch | Error / should not happen in normal operation |

### Key Behavior Change

During **unload**: when the distal switch transitions from closed to open while the proximal switch remains closed, the firmware **stops the motor**. The filament is now parked inside the extruder gears — retracted from the hotend but still mechanically captured. The `DUANLIAO` output signals filament-absent to the VZ330, completing the automated unload. On the next load cycle, feeding can begin immediately since the filament hasn't fully left the gears.

### Wiring

The new distal switch connects to **PB14** on the top header (power/status):

```
Distal Limit Switch (NO)
     |         |
    COM       NO
     |         |
    GND       PB14
  (top hdr)  (top hdr)
```

Solder the switch wires to PB14 and GND on the 4-pin top header (same header as 12-24V, GND, PB15).

### Pin Choice: PB14

PB14 was `EXTENSION_PIN7` — the only broken-out pin never used by stock firmware. It was chosen because:
- **No functional impact** — not used by any stock feature (MDM, signal control, etc.)
- All other features (PB5/PB6 signal control, PA4 MDM detection, etc.) remain fully operational
- On the top header alongside power and PB15, physically accessible
- Configured as `INPUT_PULLUP` — clean signal for a NO switch
