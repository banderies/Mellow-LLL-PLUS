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
    Filament path (loading direction -->)

    [User inserts] → [Proximal Switch (PB7)] → [Extruder Gears] → [Distal Switch (PB14)] → [Bowden to buffer/hotend]
                      (stock, detects entry)    (drive mechanism)   (custom, detects gear capture)
```

**Physical layout detail:** The user inserts filament at the proximal switch first. The filament must be pushed slightly past the proximal switch to reach the extruder gears. Once the gears grab it, the motor drives it forward past the distal switch, through the bowden tube, and ultimately to the hotend. The buffer's hall effect sensors detect backpressure when the filament reaches the hotend.

### Sensor Terminology

| Switch | Pin | Position | Firmware Name |
|--------|-----|----------|---------------|
| **Proximal** | PB7 | Before extruder gears (loading side) | `ENDSTOP_3` (existing) |
| **Distal** | PB14 | After extruder gears (hotend side) | `DISTAL_SWITCH` (custom) |

Both are normally-open (NO) switches wired COM→GND, NO→pin. LOW = filament present (switch closed), HIGH = filament absent (switch open). PB14 is configured as INPUT_PULLUP in firmware.

### Device States

The firmware implements a state machine with three primary states and several transition states:

| State | Proximal | Distal | Motor | DUANLIAO | Description |
|-------|----------|--------|-------|----------|-------------|
| **Empty** | Open | Open | Off | Absent | No filament in device |
| **Primed** | Closed* | Closed | Off | Absent | Filament captured in extruder gears, not at hotend |
| **Loaded** | Closed* | Closed | Hall sensors | Present | Filament at hotend, normal buffer operation |
| PrimingForward | Closed | Open | Forward | Absent | Advancing filament into gears (5s timeout) |
| Retracting | Closed | Closed→Open | Back | Absent | Pulling filament back from Loaded (no timeout) |
| Repriming | Closed | Open→Closed | Forward | Absent | Re-advancing filament after retraction (5s timeout) |
| Unloading | Closed→Open | — | Back | Absent | Removing filament from Primed (5s timeout) |
| Halted | — | — | Off | Absent | Stopped (button halt, timeout, or error) |
| StartupProbe | Closed | Closed | Back | Absent | Boot-time retraction to determine Primed vs Loaded (10s timeout) |

\* **Runout rule:** the device only becomes non-functional (**Empty**) when *both* switches are open. Proximal open with distal still closed means the spool has run out and the tail is still gripped by the extruder gears — Primed and Loaded keep operating normally so the remaining filament can be fed through (and a fresh spool can be hot-refilled behind it). See *Spool Runout* below.

### Button Behavior

| Button | In Stable State | During Transition | Held 2 Seconds |
|--------|----------------|-------------------|----------------|
| **Forward (KEY2)** | Empty→Primed, Primed→Loaded | Halt (stop motor) | Deadman: motor runs forward until released |
| **Back (KEY1)** | Loaded→Retracting, Primed→Empty | Halt (stop motor) | Deadman: motor runs backward until released |

After deadman release, the pre-deadman state is restored if sensors are consistent (both switches still match). If sensors contradict (e.g., filament removed), state is determined from sensor reality (Empty if both open, Halted otherwise).

### Loading Sequence (Empty → Primed → Loaded)

1. **Empty**: Motor off, waiting for filament
2. User inserts filament → proximal triggers → motor auto-advances forward (**PrimingForward**)
3. Filament passes through extruder gears → distal triggers → motor stops → **Primed**
4. User presses **forward button** → **Loaded** (hall sensor buffer operation begins)
5. Motor feeds filament through bowden tube to hotend
6. Backpressure builds → buffer hall sensors control motor → normal printing operation

### Unloading Sequence (Loaded → Primed)

1. **Loaded**: Normal printing operation
2. User presses **back button** → motor retracts (**Retracting**)
3. Filament retracts until distal switch opens (filament pulled past gears)
4. Motor stops briefly, then **reverses** forward (**Repriming**)
5. Filament re-advances until distal triggers → motor stops → **Primed**
6. DUANLIAO signals absent throughout — VZ330 knows filament is not at hotend

### Full Removal (Primed → Empty)

1. **Primed**: Filament in gears, motor off
2. User presses **back button** → motor retracts (**Unloading**)
3. Filament pulled past proximal switch → proximal opens → motor stops → **Empty**
4. User pulls filament out

### Re-loading After Unload (Primed → Loaded)

1. **Primed**: Filament parked in gears
2. User presses **forward button** → **Loaded** (hall sensor operation)
3. Motor feeds filament to hotend automatically
4. No manual re-insertion needed — filament never left the gears

### Spool Runout (Loaded → Empty)

1. **Loaded**: Normal printing operation
2. Spool runs out → tail passes proximal switch → proximal opens, distal still closed
3. **Buffer keeps running** — hall sensors continue to drive the motor, DUANLIAO still signals present. The tail is still gripped by the gears, so the printer can consume the remaining filament. Inserting a new spool at this point (hot refill) re-closes proximal; the new filament follows the tail through the gears.
4. Tail clears the extruder gears → distal opens → both switches open → motor stops → **Empty**
5. DUANLIAO signals absent → Klipper runout / PAUSE. Filament between the buffer's distal switch and the hotend is no longer under buffer control.

If the tail drifts back over the distal switch (e.g. a printer retraction), or the device boots with proximal open / distal closed, it goes to **Halted**: forward → Loaded (feed the tail through), back → Retracting.

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
