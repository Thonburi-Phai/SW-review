#!/usr/bin/env python3
"""Serial tool for BMS TAAP.
Usage:
  python3 serial_tool.py read [seconds]       - Read ASCII lines
  python3 serial_tool.py cmd SSS [seconds]    - Send command then read
  python3 serial_tool.py raw [seconds]        - Read raw hex dump
"""

import serial
import os
import sys
import time
import termios
import struct

PORT = '/dev/ttyACM0'
BAUD = 9600

def setup_raw_write(port):
    """Open port for raw non-blocking write."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    return fd

def send_chars(fd, text):
    """Send characters one at a time with delay."""
    for ch in text:
        try:
            os.write(fd, ch.encode('ascii'))
        except BlockingIOError:
            time.sleep(0.1)
            os.write(fd, ch.encode('ascii'))
        time.sleep(0.05)

def extract_ascii_lines(raw_data):
    """Extract ASCII-readable lines from mixed binary/text data."""
    lines = []
    segments = raw_data.split(b'\x0a')
    for seg in segments:
        text = seg.replace(b'\x0d', b'').decode('ascii', errors='replace')
        if len(text) < 3:
            continue
        printable = sum(1 for c in text if 32 <= ord(c) <= 126)
        if printable > len(text) * 0.7:
            lines.append(text)
    return lines

def main():
    if len(sys.argv) < 2:
        print("Usage: serial_tool.py {read|cmd|raw} [args...]")
        return

    action = sys.argv[1]

    ser = serial.Serial(PORT, BAUD, timeout=2)
    wfd = setup_raw_write(PORT)

    if action == 'cmd':
        cmd = sys.argv[2] if len(sys.argv) > 2 else 'SSS'
        duration = int(sys.argv[3]) if len(sys.argv) > 3 else 8

        # Drain input buffer
        ser.reset_input_buffer()

        print(f'> Sending: {cmd}', flush=True)
        send_chars(wfd, cmd)

        # Read for duration seconds
        raw = b''
        start = time.time()
        while (time.time() - start) < duration:
            d = ser.read(2048)
            if d:
                raw += d

        lines = extract_ascii_lines(raw)
        for line in lines:
            print(line, flush=True)
        print(f'\n[{len(lines)} lines, {len(raw)} bytes total]', flush=True)

    elif action == 'read' or action == 'ascii':
        duration = int(sys.argv[2]) if len(sys.argv) > 2 else 8

        raw = b''
        start = time.time()
        while (time.time() - start) < duration:
            d = ser.read(2048)
            if d:
                raw += d

        lines = extract_ascii_lines(raw)
        for line in lines:
            print(line, flush=True)
        print(f'\n[{len(lines)} lines, {len(raw)} bytes total]', flush=True)

    elif action == 'raw':
        duration = int(sys.argv[2]) if len(sys.argv) > 2 else 3

        start = time.time()
        while (time.time() - start) < duration:
            d = ser.read(256)
            if d:
                for i in range(0, len(d), 32):
                    chunk = d[i:i+32]
                    hexpart = ' '.join(f'{b:02X}' for b in chunk)
                    ascpart = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
                    print(f'  {hexpart}  {ascpart}', flush=True)

    os.close(wfd)
    ser.close()

if __name__ == '__main__':
    main()
