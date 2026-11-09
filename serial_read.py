#!/usr/bin/env python3
"""Read serial data from /dev/ttyACM0 at 9600 baud.
Reads raw bytes, converts to ASCII, prints with timestamps.
Usage: python3 serial_read.py [seconds]
  seconds: how long to read (default: 10)
"""

import serial
import sys
import time

PORT = '/dev/ttyACM0'
BAUD = 9600
DURATION = int(sys.argv[1]) if len(sys.argv) > 1 else 10

ser = serial.Serial(PORT, BAUD, timeout=0.5)
ser.reset_input_buffer()

print(f"[serial_read] Listening on {PORT} @ {BAUD} for {DURATION}s ...")
start = time.time()
buf = b''

while (time.time() - start) < DURATION:
    data = ser.read(256)
    if data:
        buf += data
        # Decode and print line by line
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            text = line.decode('ascii', errors='replace').rstrip('\r')
            elapsed = time.time() - start
            print(f"[{elapsed:6.2f}] {text}")

# Flush remaining buffer
if buf:
    text = buf.decode('ascii', errors='replace').rstrip('\r')
    elapsed = time.time() - start
    print(f"[{elapsed:6.2f}] {text}")

ser.close()
print(f"[serial_read] Done ({DURATION}s)")
