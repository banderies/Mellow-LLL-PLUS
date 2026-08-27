# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a custom fork of the Mellow Fly LLL (Long Link Line) buffer board firmware, for a filament buffer system on a VZ330 3D printer. The device manages filament feeding and detection using a TMC2209 stepper driver and multiple sensors.

**Target Hardware:** STM32F072C8T6 microcontroller (Cortex-M0, 48MHz, 64KB flash, 16KB RAM)

## Git / Fork Structure

This repo is a fork of `Fly3DTeam/Buffer` (the official Mellow firmware).

- **`origin`** (`banderies/Mellow-LLL-PLUS`) — our GitHub repo
- **`upstream`** (`Fly3DTeam/Buffer`) — Mellow's official repo (read-only reference)
- **Branch `custom`** — our working branch with all customizations
- **Branch `dev_v1.1.0`** — tracks upstream `dev-1.1.x` (most active upstream branch)

To check for upstream changes: `git fetch upstream && git log HEAD..upstream/dev-1.1.x --oneline`

### Custom Modifications (vs upstream)
- **Distal filament switch** on PB14: detects filament captured in extruder gears
- **State machine rewrite**: 3 primary states (Empty/Primed/Loaded) with button-driven transitions
- **Button behavior**: short press = state transition or halt; 2s hold = deadman override (restores pre-deadman state on release)
- **Motor control improvements**: coast stop for hall sensor cycling (avoids EN toggling clicks), delayed EN disable after settling
- **Startup safety**: motor starts fully de-energized (EN HIGH); state determined from EEPROM + sensors
- See `docs/STATUS.md` for full pin analysis, state table, and sequence documentation

## Build System

**Platform:** PlatformIO with Arduino framework

### Build Commands

```bash
# Build the project
pio run

# Build and upload via DFU (requires device in DFU mode)
pio run --target upload

# Clean build files
pio run --target clean

# Open serial monitor (115200 baud)
pio device monitor
```

### Upload Methods
- Primary: DFU (Device Firmware Update) mode
- Alternatives: ST-Link, J-Link, Black Magic Probe, serial bootloader

## Project Structure

```
src/main.cpp              # Entry point, calls buffer_init() and buffer_loop()
lib/buffer/               # Core buffer management logic
  buffer.h                # Pin definitions, constants, data structures
  buffer.cpp              # Main control loop and sensor/motor logic
boards/                   # Custom board definition
variants/F072C8/          # Hardware-specific pin mappings
klipper/                  # Klipper configuration files for integration
helper/buffer_control.py  # Python serial control interface (interactive + CLI)
```

## Core Architecture

The firmware operates as a state machine controlling filament buffer position:

### Sensor System
- **3 Hall Effect Sensors** (HALL1/2/3 on PB2/3/4): Detect filament position in buffer
  - Position 1 (HALL3): Trigger forward feed
  - Position 2 (HALL2): Stop motor
  - Position 3 (HALL1): Trigger reverse feed
- **Proximal Filament Switch** (PB7): Material present/absent detection (buffer side of extruder gears)
- **Distal Filament Switch** (PB14, custom): Detects filament on hotend side of extruder gears
- **2 Manual Buttons** (KEY1/KEY2 on PB13/12): Manual forward/reverse control

### Motor Control
- TMC2209 stepper driver with UART control (9600 baud on PB1)
- Speed control via VACTUAL register (default 260 RPM)
- 64 microsteps per step
- Three motor command functions:
  - `cmd_motor_forward()` / `cmd_motor_back()`: EN LOW + set direction + VACTUAL with retry
  - `cmd_motor_coast()`: VACTUAL=0 but EN stays LOW (coils energized with hold current, silent restart)
  - `cmd_motor_stop()`: coast + EN HIGH (full de-energization, used for state transitions/safety)
- Hall sensor buffer cycling (DS_Loaded) uses coast stop to avoid audible clicks from EN toggling
- Delayed EN disable: after coast stop, EN goes HIGH after `coast_delay` ms (default 500, configurable)

### Optional MDM Module
If connected, adds blockage detection by comparing:
- Expected pulse count from controller (via TIM2)
- Actual movement pulses from MDM encoder
- Triggers blockage alarm when error exceeds threshold

### Safety Features
- Independent watchdog timer (IWDG) with 2s timeout
- Timeout detection for continuous feeding (default 120s)
- Blockage detection with configurable error tolerance
- EEPROM storage for persistent configuration

## Serial Commands

Connect via USB serial at 115200 baud:

```
timeout <ms>         # Set forward feed timeout (default 120000ms)
rt                   # Read current timeout value
steps <value>        # Set steps per mm (default 916)
encoder <value>      # Set MDM encoder length in mm/pulse (default 1.73)
scale <value>        # Set blockage error scale factor (default 2)
speed <rpm>          # Set motor speed in RPM (default 260)
I <mA>               # Set motor current in mA (0-3000, default 500)
out <0|1>            # Set DUANLIAO output polarity (filament absent signal)
coast <ms>           # Set coast delay in ms (0-10000, default 500)
info                 # Display all current parameters + switch states
clear                # Reset blockage detection counters
version              # Show firmware version
```

## Pin Mapping Reference

Critical pins defined in `lib/buffer/buffer.h`:
- Motor: EN=PA6, DIR=PA7, STEP=PC13, UART=PB1
- Indicators: ERR_LED=PA15, START_LED=PA8, DUANLIAO(filament runout)=PB15, DULIAO(blockage)=PB15
- Extension pins: PA2/3/4/5, PB10/11 (used for blockage detection interface)
- Distal switch: PB14 (custom — filament detection on hotend side of extruder gears)
- Signal control: FRONT_SIGNAL_PIN=PB5, BACK_SIGNAL_PIN=PB6 (external control inputs)

## Klipper Integration

The `klipper/` directory contains configuration files for integrating this buffer with Klipper firmware. Key features:
- Filament runout sensor configuration
- Manual load/retract macros
- Configurable extrusion parameters (temperature, length, speed)

To use: Include the appropriate .cfg file in your printer.cfg and adjust pin assignments to match your mainboard connections.

## Development Notes

### Interrupt Priority
The firmware uses multiple interrupt sources with specific priorities (set in `buffer_init()`):
- Priority 0: TIM6 (timeout + watchdog)
- Priority 1: EXTI4_15 (buttons, direction signal, MDM pulses)

### State Machine Flow
The firmware uses an explicit state machine (`DeviceState` enum in `buffer.cpp`):
1. Read switches + detect button edges (short press vs 2s deadman hold)
2. Handle deadman override (saves pre-deadman state; motor runs in direction while held; restores original state on release unless sensors contradict)
3. Handle PB5/PB6 external signal control (blocking deadman)
4. Global sensor validation: both switches open → force to Empty. This is the *only* non-functional condition — proximal open with distal closed means a spool tail is still gripped by the gears (runout) and every state keeps operating normally
5. Check timeout (`is_error` from timer ISR) → transition to Halted (active in forward states + Loaded)
6. Execute state-specific logic:
   - **Empty**: auto-advance when proximal triggers (→ PrimingForward)
   - **PrimingForward**: advance until distal triggers (→ Primed), 5s timeout
   - **Primed**: wait for button. Forward → Loaded, Back → Unloading. Distal open → re-prime (500ms debounce)
   - **Loaded**: hall sensor buffer logic (upstream behavior: no-sensor = continue direction). Uses coast stop. Forward timeout active. Back → Retracting. Spool runout (proximal opens) does NOT stop the buffer — it keeps feeding until the tail clears the distal switch → Empty
   - **Retracting**: motor runs back until distal opens, then auto-reverse (→ Repriming). No timeout.
   - **Repriming**: forward until distal triggers (→ Primed), 5s timeout
   - **Unloading**: back until proximal opens (→ Empty), 5s timeout
   - **Halted**: wait for button, determine next state from switch positions (distal closed = filament in gears: Forward → Loaded, Back → Retracting, regardless of proximal)
   - **StartupProbe**: retract on boot to determine Primed vs Loaded (10s timeout)
7. Persist state to EEPROM on Primed/Loaded entry
8. Process serial commands

### TMC2209 Communication
Communication failures are handled with automatic retries (up to 9 attempts). The IFCNT register is monitored to verify command transmission.

### Important Variables
- `device_state`: Current state machine state (`DeviceState` enum)
- `motor_state`: Current motor state (Forward/Stop/Back)
- `is_error`: Global error flag set by timer ISR on forward timeout
- `is_front`: Tracks forward movement for timeout monitoring
- `coast_delay`: Configurable ms delay before EN pin disable after coast stop (EEPROM-persisted)
- `blockage_detect`: Structure holding blockage detection data
