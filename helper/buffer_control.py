#!/usr/bin/env python3
"""
Mellow Fly LLL Buffer Control Interface
Interactive serial interface for the Fly LLL filament buffer board.
"""

import serial
import serial.tools.list_ports
import time
import sys
import argparse
from typing import Optional


class BufferController:
    """Interface for Mellow Fly LLL Buffer Board"""

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        """
        Initialize connection to buffer board.

        Args:
            port: Serial port path (e.g., /dev/ttyUSB0, COM3). Auto-detect if None.
            baudrate: Serial baud rate (default 115200)
        """
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None

        if port:
            self.port = port
        else:
            self.port = self._auto_detect_port()

        if not self.port:
            raise Exception("No serial port specified and auto-detection failed")

    def _auto_detect_port(self) -> Optional[str]:
        """Attempt to auto-detect the buffer board serial port"""
        ports = serial.tools.list_ports.comports()

        print("\nAvailable serial ports:")
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")

        if not ports:
            print("  No serial ports found")
            return None

        if len(ports) == 1:
            print(f"\nAuto-selecting only available port: {ports[0].device}")
            return ports[0].device

        # Let user select
        try:
            choice = input("\nSelect port number (or press Enter to skip): ").strip()
            if choice == "":
                return None
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx].device
        except (ValueError, IndexError):
            pass

        return None

    def connect(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=2.0,
                write_timeout=2.0
            )
            time.sleep(0.5)  # Allow connection to stabilize
            # Clear any initial data
            self.serial.reset_input_buffer()
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")

    def send_command(self, cmd: str, wait_response: bool = True) -> str:
        """
        Send command to buffer board and optionally wait for response.

        Args:
            cmd: Command string
            wait_response: Whether to wait for and return response

        Returns:
            Response string if wait_response=True, empty string otherwise
        """
        if not self.serial or not self.serial.is_open:
            return "Error: Not connected"

        try:
            # Send command with newline
            self.serial.write(f"{cmd}\n".encode('utf-8'))
            self.serial.flush()

            if not wait_response:
                return ""

            # Read response (may be multiple lines)
            response = ""
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response += line + "\n"
                    time.sleep(0.01)
                else:
                    # If we have a response and no more data, we're done
                    if response:
                        break
                    time.sleep(0.01)

            return response.strip()

        except Exception as e:
            return f"Error: {e}"

    # Command wrappers

    def get_info(self) -> str:
        """Get all configuration parameters"""
        return self.send_command("info")

    def set_speed(self, rpm: float) -> str:
        """Set motor speed in RPM (0-1000)"""
        return self.send_command(f"speed {rpm}")

    def get_speed(self) -> str:
        """Get current motor speed"""
        return self.send_command("speed")

    def set_timeout(self, ms: int) -> str:
        """Set forward feed timeout in milliseconds"""
        return self.send_command(f"timeout {ms}")

    def get_timeout(self) -> str:
        """Read current timeout value"""
        return self.send_command("rt")

    def set_steps(self, steps: int) -> str:
        """Set steps per mm (0-51200)"""
        return self.send_command(f"steps {steps}")

    def set_encoder_length(self, length: float) -> str:
        """Set MDM encoder length in mm/pulse"""
        return self.send_command(f"encoder {length}")

    def set_error_scale(self, scale: float) -> str:
        """Set blockage error scale factor"""
        return self.send_command(f"scale {scale}")

    def clear_blockage(self) -> str:
        """Clear blockage detection counters"""
        return self.send_command("clear")

    def monitor(self, duration: Optional[int] = None):
        """
        Monitor serial output from device.

        Args:
            duration: Monitoring duration in seconds (None = indefinite)
        """
        if not self.serial or not self.serial.is_open:
            print("Error: Not connected")
            return

        print(f"\nMonitoring serial output (Ctrl+C to stop)...")
        print("-" * 50)

        start_time = time.time()
        try:
            while True:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(line)

                if duration and (time.time() - start_time) > duration:
                    break

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n" + "-" * 50)
            print("Monitoring stopped")


def interactive_mode(controller: BufferController):
    """Run interactive command prompt"""

    print("\n" + "="*60)
    print("  Mellow Fly LLL Buffer - Interactive Mode")
    print("="*60)
    print("\nCommands:")
    print("  info                    - Show all parameters")
    print("  speed [rpm]             - Set/get motor speed (RPM)")
    print("  timeout <ms>            - Set forward feed timeout")
    print("  rt                      - Read current timeout")
    print("  steps <value>           - Set steps per mm")
    print("  encoder <length>        - Set encoder length (mm/pulse)")
    print("  scale <factor>          - Set error scale factor")
    print("  clear                   - Clear blockage counters")
    print("  monitor [duration]      - Monitor serial output")
    print("  raw <command>           - Send raw command")
    print("  help                    - Show this help")
    print("  quit / exit             - Exit program")
    print()

    while True:
        try:
            user_input = input("buffer> ").strip()

            if not user_input:
                continue

            parts = user_input.split(maxsplit=1)
            cmd = parts[0].lower()
            args = parts[1] if len(parts) > 1 else ""

            if cmd in ['quit', 'exit', 'q']:
                break

            elif cmd == 'help':
                # Re-print help
                print("\nCommands:")
                print("  info, speed, timeout, rt, steps, encoder, scale,")
                print("  clear, monitor, raw, help, quit/exit")

            elif cmd == 'info':
                print(controller.get_info())

            elif cmd == 'speed':
                if args:
                    print(controller.set_speed(float(args)))
                else:
                    print(controller.get_speed())

            elif cmd == 'timeout':
                if args:
                    print(controller.set_timeout(int(args)))
                else:
                    print("Usage: timeout <milliseconds>")

            elif cmd == 'rt':
                print(controller.get_timeout())

            elif cmd == 'steps':
                if args:
                    print(controller.set_steps(int(args)))
                else:
                    print("Usage: steps <value>")

            elif cmd == 'encoder':
                if args:
                    print(controller.set_encoder_length(float(args)))
                else:
                    print("Usage: encoder <mm_per_pulse>")

            elif cmd == 'scale':
                if args:
                    print(controller.set_error_scale(float(args)))
                else:
                    print("Usage: scale <factor>")

            elif cmd == 'clear':
                print(controller.clear_blockage())

            elif cmd == 'monitor':
                duration = int(args) if args else None
                controller.monitor(duration)

            elif cmd == 'raw':
                if args:
                    response = controller.send_command(args)
                    print(response)
                else:
                    print("Usage: raw <command>")

            else:
                print(f"Unknown command: {cmd}")
                print("Type 'help' for available commands")

        except KeyboardInterrupt:
            print()
            break
        except ValueError as e:
            print(f"Invalid argument: {e}")
        except Exception as e:
            print(f"Error: {e}")

    print("\nGoodbye!")


def main():
    parser = argparse.ArgumentParser(
        description='Control interface for Mellow Fly LLL Buffer Board'
    )
    parser.add_argument(
        '-p', '--port',
        help='Serial port (e.g., /dev/ttyUSB0, COM3). Auto-detect if not specified.'
    )
    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    parser.add_argument(
        '-c', '--command',
        help='Execute single command and exit'
    )
    parser.add_argument(
        '-m', '--monitor',
        action='store_true',
        help='Monitor mode - display serial output'
    )

    args = parser.parse_args()

    try:
        # Initialize controller
        controller = BufferController(port=args.port, baudrate=args.baudrate)

        # Connect
        if not controller.connect():
            sys.exit(1)

        # Execute based on mode
        if args.monitor:
            controller.monitor()
        elif args.command:
            response = controller.send_command(args.command)
            print(response)
        else:
            interactive_mode(controller)

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        if controller.serial:
            controller.disconnect()


if __name__ == "__main__":
    main()
