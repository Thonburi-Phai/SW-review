#!/usr/bin/env python3
"""
decode_ntc_fault.py -- decode binary protocol packet 132 (MCU NTC band verdict
and FET request) from a raw UART capture.

Companion to decode_wire_fault.py; same frame handling, different payload. The
layout is in src/protocol/link_bms_ntc_fault.h and must be kept in step with it.

Frame:  07 | op | len16 LE | sender | receiver | pkt_id | count | payload | crc32
CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, no reflection, no final xor)
over bytes 1 .. 8+len-1.

Usage:  decode_ntc_fault.py CAPTURE.bin [--all] [--csv]
        --all   print every frame, not just the ones where something changed
        --csv   machine-readable output
"""
import struct, sys, argparse

from decode_wire_fault import frames

PKT_ID = 132
FMT = "<IBBBBBBBBhhhBBB"
assert struct.calcsize(FMT) == 21

STATE = ["ok", "degraded", "tripped", "service"]
CHG_BAND = ["off-cold", "pre-charge", "normal", "extended", "off-hot"]
DSG_BAND = ["off-cold", "normal", "extended", "off-hot"]
FET_BAND = ["off-cold", "ok", "off-hot"]
FET_SRC = ["none", "NTC_CH4", "BQ TS1"]


def _name(table, i):
    return table[i] if i < len(table) else f"?{i}"


def fets(bits):
    """FETControl host OFF bits, as src/bq7690x/bq7690x_fet.h defines them."""
    if bits == 0:
        return "-"
    out = []
    if bits & 0x04:
        out.append("DSG_OFF")
    if bits & 0x08:
        out.append("CHG_OFF")
    extra = bits & ~0x0C
    if extra:
        out.append(f"0x{extra:02X}")
    return "+".join(out)


def decode(payload):
    f = struct.unpack(FMT, payload[:21])
    return dict(
        t_ms=f[0],
        enabled=bool(f[1]),
        state=_name(STATE, f[2]),
        usable=f[3],
        open_wire=f[4],
        chg=_name(CHG_BAND, f[5]),
        dsg=_name(DSG_BAND, f[6]),
        fet=_name(FET_BAND, f[7]),
        fet_src=_name(FET_SRC, f[8]),
        batt_min_c=f[9],
        batt_max_c=f[10],
        fet_temp_c=f[11],
        want=f[12],
        applied=f[13],
        union=f[14])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--csv", action="store_true")
    a = ap.parse_args()

    with open(a.capture, "rb") as fh:
        buf = fh.read()

    rows = [decode(p) for pid, p in frames(buf) if pid == PKT_ID and len(p) >= 21]
    if not rows:
        print(f"no valid packet {PKT_ID} frames in {len(buf)} bytes", file=sys.stderr)
        return 1

    # A 600 ms heartbeat of unchanged bands is noise; a change is the event.
    key = lambda r: (r["enabled"], r["state"], r["usable"], r["open_wire"],
                     r["chg"], r["dsg"], r["fet"], r["fet_src"],
                     r["want"], r["applied"], r["union"])
    if a.csv:
        cols = list(rows[0])
        print(",".join(cols))
        for r in rows:
            print(",".join(str(r[c]) for c in cols))
        return 0

    print(f"{len(rows)} frames, {rows[0]['t_ms']/1000:.1f}s .. "
          f"{rows[-1]['t_ms']/1000:.1f}s")
    prev = None
    for r in rows:
        if not a.all and prev is not None and key(r) == key(prev):
            prev = r
            continue
        print(f"t={r['t_ms']/1000:8.1f}s  {'EN ' if r['enabled'] else 'off'} "
              f"{r['state']:<8} usable=0x{r['usable']:02X} open=0x{r['open_wire']:02X} | "
              f"batt {r['batt_min_c']:>4}..{r['batt_max_c']:<4}C "
              f"chg={r['chg']:<10} dsg={r['dsg']:<9} | "
              f"fet {r['fet_temp_c']:>4}C ({r['fet_src']}) {r['fet']:<8} | "
              f"want={fets(r['want']):<15} applied={fets(r['applied']):<15} "
              f"bus={fets(r['union'])}")
        prev = r
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        try:
            sys.stdout.close()
        except Exception:
            pass
        sys.exit(0)
