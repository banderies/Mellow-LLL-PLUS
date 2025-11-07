# Mellow Fly LLL Buffer Control Helper

Python script for interacting with the Mellow Fly LLL filament buffer board via serial connection.

## Installation

```bash
# Install dependencies
pip install -r requirements.txt

# Make script executable (Unix-like systems)
chmod +x buffer_control.py
```

## Usage

### Interactive Mode (Recommended)

```bash
# Auto-detect port and enter interactive mode
python buffer_control.py

# Specify port manually
python buffer_control.py -p /dev/ttyUSB0    # Linux
python buffer_control.py -p COM3            # Windows
```

In interactive mode, you can use commands like:
```
buffer> info                    # Show all parameters
buffer> speed 300               # Set speed to 300 RPM
buffer> speed                   # Read current speed
buffer> timeout 45000           # Set timeout to 45 seconds
buffer> rt                      # Read current timeout
buffer> steps 920               # Set steps per mm
buffer> encoder 1.75            # Set encoder length
buffer> scale 2.5               # Set error scale factor
buffer> clear                   # Clear blockage counters
buffer> monitor 10              # Monitor output for 10 seconds
buffer> raw <custom_cmd>        # Send raw command
buffer> quit                    # Exit
```

### Single Command Mode

Execute one command and exit:

```bash
python buffer_control.py -c "info"
python buffer_control.py -c "speed 280"
python buffer_control.py -c "rt"
```

### Monitor Mode

Continuously display serial output from the device:

```bash
python buffer_control.py --monitor
```

## Available Commands

| Command | Description | Example |
|---------|-------------|---------|
| `info` | Display all current parameters | `info` |
| `speed [rpm]` | Set/get motor speed (0-1000 RPM) | `speed 260` |
| `timeout <ms>` | Set forward feed timeout | `timeout 60000` |
| `rt` | Read current timeout value | `rt` |
| `steps <value>` | Set steps per mm (0-51200) | `steps 916` |
| `encoder <length>` | Set MDM encoder length (mm/pulse) | `encoder 1.73` |
| `scale <factor>` | Set blockage error scale factor | `scale 2` |
| `clear` | Reset blockage detection counters | `clear` |
| `monitor [duration]` | Monitor serial output | `monitor 30` |

## Serial Connection

- **Baud Rate:** 115200 (default)
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
- **Flow Control:** None

## Examples

### Quick Status Check
```bash
python buffer_control.py -c "info"
```

### Configure for Different Filament
```bash
python buffer_control.py
buffer> speed 280
buffer> timeout 45000
buffer> steps 920
buffer> info
```

### Monitor During Operation
```bash
python buffer_control.py --monitor
```

### Troubleshooting Connection

If auto-detection fails, find your port manually:

**Linux/macOS:**
```bash
ls /dev/tty*    # Look for /dev/ttyUSB*, /dev/ttyACM*, or /dev/cu.*
```

**Windows:**
- Check Device Manager → Ports (COM & LPT)
- Look for "USB Serial Device" or similar

Then specify the port:
```bash
python buffer_control.py -p /dev/ttyUSB0
```

## Python API Example

You can also import and use the `BufferController` class in your own scripts:

```python
from buffer_control import BufferController

# Create controller
controller = BufferController(port="/dev/ttyUSB0")
controller.connect()

# Send commands
print(controller.get_info())
controller.set_speed(280)
controller.set_timeout(45000)

# Cleanup
controller.disconnect()
```

## Notes

- The device must be connected via USB
- Settings are persisted to EEPROM on the device
- Changes take effect immediately
- Use `Ctrl+C` to interrupt monitoring mode
