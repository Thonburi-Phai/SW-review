#!/usr/bin/env python3
"""Serial command tool for BMS TAAP.
Sends a command, reads response, filters out binary binary protocol packets.
Usage: python3 serial_cmd.py [command] [seconds]
  command: triple-char command like SSS, FFF, TTT (default: SSS)
  seconds: how long to read after sending (default: 8)
"""

import serial
import sys
import time

PORT = '/dev/ttyACM0'
BAUD = 9600

cmd = sys.argv[1] if len(sys.argv) > 1 else 'SSS'
duration = int(sys.argv[2]) if len(sys.argv) > 2 else 8

ser = serial.Serial(PORT, BAUD, timeout=0.5)
ser.reset_input_buffer()

# Send command characters one at a time with small delays
print(f"[cmd] Sending '{cmd}' on {PORT} @ {BAUD}, reading for {duration}s")
for ch in cmd:
    ser.write(ch.encode('ascii'))
    time.sleep(0.05)

start = time.time()
buf = b''
lines = []

while (time.time() - start) < duration:
    data = ser.read(512)
    if data:
        buf += data
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            text = line.decode('ascii', errors='replace').rstrip('\r')
            # Filter: only print lines that are mostly printable ASCII
            printable = sum(1 for c in text if 32 <= ord(c) <= 126 or c in '\t')
            if len(text) > 0 and printable > len(text) * 0.7:
                elapsed = time.time() - start
                print(f"[{elapsed:6.2f}] {text}")
                lines.append(text)

# Flush remaining
if buf:
    text = buf.decode('ascii', errors='replace').rstrip('\r')
    printable = sum(1 for c in text if 32 <= ord(c) <= 126 or c in '\t')
    if len(text) > 0 and printable > len(text) * 0.7:
        print(f"[{time.time()-start:6.2f}] {text}")

ser.close()
print(f"\n[cmd] Done. {len(lines)} readable lines captured.")
