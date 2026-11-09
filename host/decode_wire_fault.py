#!/usr/bin/env python3
"""
decode_wire_fault.py -- decode binary protocol packet 131 (open-wire / wire-fault
telemetry) from a raw UART capture.

Why this exists as a file rather than an inline one-liner: V2 has no console RX
and the binary protocol broadcast shares the debug UART, so `cat /dev/ttyXXX` on a bench
run returns binary. Every Phase E result was read back through an ad-hoc version
of this script, which meant re-deriving the frame layout each time. The layout
is in src/protocol/link_bms_wire_fault.h and must be kept in step with it.

Frame:  07 | op | len16 LE | sender | receiver | pkt_id | count | payload | crc32
CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, no reflection, no final xor)
over bytes 1 .. 8+len-1.

Usage:  decode_wire_fault.py CAPTURE.bin [--all] [--csv]
        --all   print every frame, not just the ones where something changed
        --csv   machine-readable output
"""
import struct, sys, argparse

PKT_ID = 131
HDR = 8
FMT = "<IBBBBBHhBBBBIHH"
assert struct.calcsize(FMT) == 25

OW_KIND = ["none", "mid-tap", "top-tap", "bottom-tap", "multi", "real-cell",
           "ambiguous"]
OW_STATE = ["idle", "suspect", "confirmed", "recovery"]
WF_STATE = ["inactive", "limp", "service"]


def _crc32_mpeg2(data):
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 \
                  else (crc << 1) & 0xFFFFFFFF
    return crc


def frames(buf):
    """Yield (pkt_id, payload) for every CRC-valid frame, resyncing on junk."""
    i = 0
    while True:
        i = buf.find(b"\x07", i)
        if i < 0 or i + HDR + 4 > len(buf):
            return
        length = struct.unpack_from("<H", buf, i + 2)[0]
        end = i + HDR + length + 4
        if length > 512 or end > len(buf):
            i += 1
            continue
        want = struct.unpack_from("<I", buf, i + HDR + length)[0]
        if _crc32_mpeg2(buf[i + 1:i + HDR + length]) != want:
            i += 1
            continue
        yield buf[i + 6], buf[i + HDR:i + HDR + length]
        i = end


def decode(payload):
    f = struct.unpack(FMT, payload[:25])
    return dict(
        t_ms=f[0],
        ow_kind=OW_KIND[f[1]] if f[1] < len(OW_KIND) else f"?{f[1]}",
        ow_tap=f[2], ow_cell=f[3], settled=bool(f[4]),
        ow_state=OW_STATE[f[5]] if f[5] < len(OW_STATE) else f"?{f[5]}",
        ow_events=f[6], sum_minus_stack_mv=f[7],
        wf_state=WF_STATE[f[8]] if f[8] < len(WF_STATE) else f"?{f[8]}",
        bal_locked=bool(f[9]), soc_suppressed=bool(f[10]),
        dsg_map=f[11], grace_left_ms=f[12], reapply=f[13], uv_trips=f[14])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--csv", action="store_true")
    a = ap.parse_args()

    with open(a.capture, "rb") as fh:
        buf = fh.read()

    rows = [decode(p) for pid, p in frames(buf) if pid == PKT_ID and len(p) >= 25]
    if not rows:
        print(f"no valid packet {PKT_ID} frames in {len(buf)} bytes", file=sys.stderr)
        return 1

    # The interesting quantity is a change of state, not the 600 ms heartbeat.
    key = lambda r: (r["ow_kind"], r["ow_tap"], r["ow_state"], r["wf_state"],
                     r["dsg_map"], r["uv_trips"], r["ow_events"])
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
        print(f"t={r['t_ms']/1000:8.1f}s  ow={r['ow_kind']:<10} tap={r['ow_tap']} "
              f"cell={r['ow_cell']} {'settled' if r['settled'] else 'drift  '} "
              f"st={r['ow_state']:<9} ev={r['ow_events']:<3} "
              f"sum-stack={r['sum_minus_stack_mv']:>6} mV | "
              f"wf={r['wf_state']:<8} dsg=0x{r['dsg_map']:02X} "
              f"grace={r['grace_left_ms']/1000:6.1f}s "
              f"reapply={r['reapply']} uv_trips={r['uv_trips']}")
        prev = r
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        # `| head` closes the pipe early; that is not an error worth a traceback.
        try:
            sys.stdout.close()
        except Exception:
            pass
        sys.exit(0)
