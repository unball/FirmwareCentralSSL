from __future__ import annotations

import threading
import time
import argparse
from pynput import keyboard
import serial


class KeyboardController:

    def __init__(self, port='COM8', baudrate=115200):
        self._lock = threading.Lock()
        
        self._key_map = {
            "w": 0,
            "s": 0,
            "a": 0,
            "d": 0,
            "q": 0,
            "e": 0,
        }
        self.x = 0
        self.y = 0
        self.rotation_clockwise = 0
        
        # Initialize serial connection
        try:
            self._serial = serial.Serial(port, baudrate, timeout=0.01)
            self._serial.read
            print("Connected to serial port")
        except serial.SerialException as e:
            print(f"Warning: Could not open serial port {port}")
            print(e)
            self._serial = None

        self.changed = False
        self._running = False
        self._listener: keyboard.Listener | None = None
        self._sender_thread: threading.Thread | None = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._sender_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._sender_thread.start()

        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()

    def stop(self) -> None:
        self._running = False
        if self._listener:
            try:
                self._listener.stop()
            except Exception:
                pass
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass

    def _on_press(self, key):
        k = self._key_to_str(key)
        if k == "esc":
            self.stop()
            return
        with self._lock:
            if k in self._key_map:
                self._key_map[k] = 1
                self._recompute()

    def _on_release(self, key):
        k = self._key_to_str(key)
        with self._lock:
            if k in self._key_map:
                self._key_map[k] = 0
                self._recompute()

    def _recompute(self) -> None:
        lx = 0
        ly = 0
        rotation_clockwise = 0
        ly += 1 if self._key_map["w"] else 0
        ly -= 1 if self._key_map["s"] else 0
        lx += 1 if self._key_map["d"] else 0
        lx -= 1 if self._key_map["a"] else 0
        rotation_clockwise += 1 if self._key_map["e"] else 0
        rotation_clockwise -= 1 if self._key_map["q"] else 0

        self.x = lx
        self.y = ly
        self.rotation_clockwise = rotation_clockwise
        self.changed = True

    def _send_loop(self) -> None:
        while self._running:
            try:
                # Convert -1/0/1 to 0/1/2 for encoding
                x_enc = (self.x + 1) & 0x3  # 2 bits
                y_enc = (self.y + 1) & 0x3  # 2 bits
                rot_enc = (self.rotation_clockwise + 1) & 0x3  # 2 bits
                
                # Pack into single byte: 00RRYYXX
                byte = (rot_enc << 4) | (y_enc << 2) | x_enc
                
                if self._serial:
                    self._serial.write(bytes([byte]))
                    if self._serial.readable():
                        response = self._serial.readline()
                        if response != b'':
                            print(response)
                if self.changed:
                    print(f"Sending byte: {bin(byte)[2:]:0>8}")
                    self.changed = False
            except Exception as e:
                print(f"Error sending: {e}")
            time.sleep(0.001)

    @staticmethod
    def _key_to_str(key: keyboard.Key | keyboard.KeyCode) -> str:
        # Normalize key to string names used in _key_map
        try:
            if isinstance(key, keyboard.KeyCode):
                return (key.char or "").lower()
            else:
                # special keys like Key.up have .name
                return key.name
        except Exception:
            return ""


def main() -> None:
    parser = argparse.ArgumentParser(description='Robot Keyboard Controller')
    parser.add_argument('--port', type=str, default='COM8',
                      help='Serial port to connect to (default: COM8)')
    args = parser.parse_args()

    kc = KeyboardController(port=args.port)
    print(
        "Keyboard controller started. Use W/A/S/D (or arrows), Q/E to rotate. Press ESC to exit."
    )
    kc.start()

    try:
        while kc._running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        kc.stop()


if __name__ == "__main__":
    main()
