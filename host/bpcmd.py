#!/usr/bin/env python3
"""
bpcmd.py -- host-side binary-protocol console for the BMS over the wireless probe's UART.

Why this exists: every protection threshold, delay, enable mask and FET map on the
BQ76907 is a runtime parameter, and SET_PARAMETER only touches a RAM cache until an
explicit apply. That makes almost the whole open-wire experiment plan runnable from
here -- see docs/openwire_experiment_plan.md -- but only with a tool that can build
CRC-protected frames. An earlier copy of this lived in a session scratchpad and was
lost with it, which is why it is in the repo now.

Transport is the probe's MQTT UART bridge: publish to <prefix>/uart/tx, subscribe to
<prefix>/uart/rx. Credentials come from the proxy repo's devices.toml so none live
here.

Two facts about the firmware that shape every call:

  * SET_PARAMETER has no acknowledgement packet. handle_set_parameter() only prints
    to the console, and on V2 the console is off by default, so a write that falls
    outside the parameter's bounds is silently discarded. Every set is therefore
    read back with REQ_PARAMETER, and `set` refuses to report success without it.

  * SAVE_TO_ROM does not write ROM. It applies the whole RAM cache to the chip in
    one CONFIG_UPDATE, which clears latched faults and resets PASSQ. So a fault can
    only be set up after an apply, never before.

Usage:
    bpcmd.py status [--secs N]                 decode the broadcast stream
    bpcmd.py get  <PARAM_NAME|id> [...]        read parameters
    bpcmd.py set  <PARAM_NAME|id> <value>      write cache + read back (no apply)
    bpcmd.py apply                             SAVE_TO_ROM: cache -> chip
    bpcmd.py reset                             RESET_ROM: defaults -> cache -> chip
    bpcmd.py cmd  <0xNN|name> [param]          BMS_COMMAND (BPCMD_*)
    bpcmd.py watch [--secs N] [--csv FILE]     log status at the broadcast rate
    bpcmd.py params [filter]                   list known parameter names

All names are resolved from the firmware headers at run time, so this tool cannot
drift from the build it is talking to.
"""

import argparse
import re
import struct
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PARAM_H = REPO / "src/controller/parameter/parameter.h"
PARAM_C = REPO / "src/controller/parameter/parameter.c"
MAIN_C = REPO / "src/controller/main_controller.c"
DEVICES_TOML = Path.home() / "workspace/cmsis_dap_remote_flashing/openocd-mqtt-proxy/devices.toml"

START = 0x07
OP_COMMAND, OP_RESPONSE, OP_BROADCAST = 0xAA, 0xAB, 0xAC
ID_PC, ID_BMS = 0xFE, 0x10

PKT_REQ_PARAM, PKT_SET_PARAM, PKT_SAVE_TO_ROM = 1, 2, 3
PKT_RESET_ROM, PKT_BMS_COMMAND = 25, 20
PKT_BMS_STATUS = 1
PKT_RESP_REQ_PARAM = 1          # response op disambiguates it from BMS_STATUS

TYPE_NAMES = {0: "u8", 1: "i8", 2: "u16", 3: "i16", 4: "u32", 5: "i32", 6: "f32"}
TYPE_SIGNED = {1, 3, 5}


# --------------------------------------------------------------------------- CRC
def crc32_mpeg2(data: bytes) -> int:
    """CRC-32/MPEG-2: poly 0x04C11DB7, init 0xFFFFFFFF, no reflect, no xorout.

    The firmware feeds the hardware CRC peripheral 32-bit words MSB-first and
    zero-pads the tail, which is what the word loop below reproduces. Validated
    against 333 live frames with zero mismatches.
    """
    crc = 0xFFFFFFFF
    padded = data + b"\x00" * (-len(data) % 4)
    for i in range(0, len(padded), 4):
        word = int.from_bytes(padded[i:i + 4], "big")
        crc ^= word
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 \
                else (crc << 1) & 0xFFFFFFFF
    return crc


def build(op, packet_id, payload, sender=ID_PC, receiver=ID_BMS, count=0):
    body = bytes([op]) + struct.pack("<H", len(payload)) + \
        bytes([sender, receiver, packet_id, count]) + payload
    return bytes([START]) + body + struct.pack("<I", crc32_mpeg2(body))


def frames(buf):
    """Yield (op, packet_id, payload, consumed_upto) for every CRC-valid frame."""
    i = 0
    while i < len(buf) - 11:
        if buf[i] != START or buf[i + 1] not in (OP_COMMAND, OP_RESPONSE, OP_BROADCAST):
            i += 1
            continue
        ln = buf[i + 2] | (buf[i + 3] << 8)
        total = 8 + ln + 4
        if ln > 504 or i + total > len(buf):
            i += 1
            continue
        body = buf[i + 1:i + 8 + ln]
        if crc32_mpeg2(body) != int.from_bytes(buf[i + 8 + ln:i + 8 + ln + 4], "little"):
            i += 1
            continue
        yield buf[i + 1], buf[i + 6], buf[i + 8:i + 8 + ln]
        i += total


# ------------------------------------------------------- firmware introspection
def param_ids():
    """PARAM_* name -> id, read from the enum in parameter.h."""
    txt = PARAM_H.read_text()
    m = re.search(r"typedef enum \{(.*?)\} param_id_t;", txt, re.S)
    if not m:
        sys.exit(f"cannot find param_id_t enum in {PARAM_H}")
    out, nxt = {}, 0
    for line in m.group(1).splitlines():
        line = re.sub(r"/\*.*?\*/", "", line).strip().rstrip(",")
        mm = re.match(r"^(PARAM_[A-Z0-9_]+)\s*(?:=\s*(\d+))?$", line)
        if not mm:
            continue
        if mm.group(2) is not None:
            nxt = int(mm.group(2))
        out[mm.group(1)] = nxt
        nxt += 1
    out.pop("PARAM_COUNT", None)
    return out


def param_meta():
    """PARAM_* name -> (min, max, type) from the PARAM_ADD table in parameter.c."""
    txt = PARAM_C.read_text()
    out = {}
    for mm in re.finditer(
        r"PARAM_ADD\(\s*(PARAM_[A-Z0-9_]+)\s*,\s*(-?\w+)\s*,\s*(-?\w+)\s*,[^,]+,\s*(PARAM_TYPE_\w+)",
        txt,
    ):
        name, lo, hi, ty = mm.groups()
        try:
            out[name] = (int(lo, 0), int(hi, 0), ty)
        except ValueError:
            out[name] = (lo, hi, ty)
    return out


def bpcmds():
    """BPCMD_* name -> code, from main_controller.c."""
    txt = MAIN_C.read_text()
    return {m.group(1): int(m.group(2), 0)
            for m in re.finditer(r"#define\s+(BPCMD_[A-Z0-9_]+)\s+(0x[0-9A-Fa-f]+|\d+)", txt)}


def creds(entry="dap-017-serial"):
    """Pull broker + credentials for a devices.toml entry, inheriting [defaults]."""
    if not DEVICES_TOML.exists():
        sys.exit(f"no devices.toml at {DEVICES_TOML}")
    txt = DEVICES_TOML.read_text()
    sections, cur = {}, None
    for line in txt.splitlines():
        line = line.split("#", 1)[0].strip()
        if line.startswith("["):
            cur = line.strip("[]")
            sections[cur] = {}
        elif cur and "=" in line:
            k, v = (x.strip() for x in line.split("=", 1))
            sections[cur][k] = v.strip('"')
    d = dict(sections.get("defaults", {}))
    d.update(sections.get(f"devices.{entry}", {}))
    if "topic_prefix" not in d:
        sys.exit(f"devices.toml has no [devices.{entry}]")
    return d


# ------------------------------------------------------------------- transport
class Link:
    def __init__(self, entry, client_suffix=""):
        import paho.mqtt.client as mqtt
        c = creds(entry)
        self.prefix = c["topic_prefix"]
        self.rx = bytearray()
        cid = c.get("client_id", "$internal-bpcmd") + client_suffix
        try:
            self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=cid)
        except (AttributeError, TypeError):
            self.c = mqtt.Client(client_id=cid)
        if c.get("username"):
            self.c.username_pw_set(c["username"], c.get("password", ""))
        self.c.on_message = lambda cl, ud, msg: self.rx.extend(msg.payload)
        self.c.connect(c.get("broker", "mqtt.monigrow.com"), int(c.get("mqtt_port", 1883)), 30)
        self.c.loop_start()
        self.c.subscribe(f"{self.prefix}/uart/rx", 0)
        time.sleep(1.0)        # let SUBACK land before anything is published
        self._prime()

    def _prime(self):
        """Burn one throwaway command, because the first one is always lost.

        The MCU sleeps in VLPS between activities and wakes on UART RX. The bytes
        that do the waking are consumed doing it, so the opening bytes of the
        first frame after an idle period never reach the parser and the frame is
        simply dropped. The firmware is not at fault and nothing is wrong with the
        request -- it never arrives.

        Measured on dap-017, 2026-11-02. Asking for PARAM_WF_GRACE_MS (id 123)
        standalone reported NO RESPONSE three times in a row, while the same read
        inside `set` always worked, because the SET ahead of it did the waking.
        Reversing the order proved the id was irrelevant: id 4 failed when asked
        first and succeeded when asked second. `fetctl` showed the same thing as
        "resp None" on a cold start.

        This is worth a second of startup because the failure mode is a confident
        lie: NO RESPONSE reads as "the firmware has lost this parameter", which is
        exactly the wrong conclusion to hand someone debugging a BMS.

        A read of id 0 is harmless and changes nothing.
        """
        try:
            self.send(build(OP_COMMAND, PKT_REQ_PARAM, struct.pack("<H", 0)))
            time.sleep(0.5)
        except Exception:
            pass           # priming is best-effort; never fail a session over it
        self.rx.clear()

    def send(self, pkt):
        # QoS 1: QoS 0 publishes to this broker drop silently under load.
        self.c.publish(f"{self.prefix}/uart/tx", pkt, qos=1).wait_for_publish()

    def collect(self, secs):
        self.rx.clear()
        time.sleep(secs)
        return bytes(self.rx)

    def close(self):
        self.c.loop_stop()
        self.c.disconnect()


STATUS_FMT = "<IhhhhhiibbbbHHBBBBBBbbi"
STATUS_FIELDS = ("ts", "c1", "c2", "c3", "c4", "stack", "current_ma", "power_dw",
                 "die", "ts1", "ext1", "ext2", "soc", "soh", "chg", "dsg", "bal",
                 "flags", "ssa", "ssb", "ext3", "ext4", "raw_ma")


def decode_status(buf):
    """Newest BMS_STATUS in buf, as a dict."""
    out = None
    for op, pid, pl in frames(buf):
        if op == OP_BROADCAST and pid == PKT_BMS_STATUS and len(pl) == struct.calcsize(STATUS_FMT):
            out = dict(zip(STATUS_FIELDS, struct.unpack(STATUS_FMT, pl)))
    return out


def fmt_status(s):
    return (f"t={s['ts']/1000:8.1f}s cells {s['c1']:5d} {s['c2']:5d} {s['c3']:5d} {s['c4']:5d} "
            f"stack {s['stack']:6d} I {s['current_ma']:6d}mA SOC {s['soc']/100:5.1f}% "
            f"die {s['die']:3d} ts1 {s['ts1']:3d} "
            f"CHG {'ON ' if s['chg'] else 'OFF'} DSG {'ON ' if s['dsg'] else 'OFF'} "
            f"bal {s['bal']} SSA 0x{s['ssa']:02X} SSB 0x{s['ssb']:02X}")


# ------------------------------------------------------------------- operations
def decode_param(raw, ty):
    """Sign-extend from the parameter's OWN width, not from 32 bits.

    The firmware keeps every parameter in a 4-byte union without sign-extending
    the narrow signed types, so an int16 of -30 arrives as 0x0000FFE2. Reading
    that as int32 gives 65506, which is how PARAM_UTD_THRESHOLD_C came back as
    65506 instead of -30 on 2026-11-01. This is the same class of mistake as the
    unsigned cell-voltage decode that made a -241 mV open-wire node display as
    65.295 V in the app.
    """
    if ty == 1:                                  # int8
        return struct.unpack("<b", struct.pack("<B", raw & 0xFF))[0]
    if ty == 3:                                  # int16
        return struct.unpack("<h", struct.pack("<H", raw & 0xFFFF))[0]
    if ty == 5:                                  # int32
        return struct.unpack("<i", struct.pack("<I", raw & 0xFFFFFFFF))[0]
    if ty == 6:                                  # float
        return struct.unpack("<f", struct.pack("<I", raw))[0]
    # Unsigned types must be MASKED to their declared width, not just returned.
    # param_cache is a 4-byte union and the firmware writes only the member for
    # the parameter's type, so the upper bytes keep whatever was there before.
    # The response carries all four. Seen on 2026-11-02:
    # PARAM_DSG_FET_PROTECTIONS_A, a u8, read back as 38645 = 0x96F5 -- the low
    # byte 0xF5 correct, the rest left over from an earlier write.
    if ty == 0:                                  # uint8
        return raw & 0xFF
    if ty == 2:                                  # uint16
        return raw & 0xFFFF
    return raw & 0xFFFFFFFF


def resolve(token, ids):
    if token in ids:
        return ids[token]
    up = token.upper()
    if up in ids:
        return ids[up]
    if not up.startswith("PARAM_") and f"PARAM_{up}" in ids:
        return ids[f"PARAM_{up}"]
    try:
        return int(token, 0)
    except ValueError:
        sys.exit(f"unknown parameter {token!r} (try: bpcmd.py params)")


def do_get(link, tokens, ids, meta, by_id, secs=3.0):
    rows = []
    for tok in tokens:
        pid = resolve(tok, ids)
        link.rx.clear()
        link.send(build(OP_COMMAND, PKT_REQ_PARAM, struct.pack("<H", pid)))
        got, deadline = None, time.time() + secs
        while time.time() < deadline:
            for op, p, pl in frames(bytes(link.rx)):
                if op == OP_RESPONSE and p == PKT_RESP_REQ_PARAM and len(pl) == 8:
                    raw, rid, ty, ack = struct.unpack("<IHBB", pl)
                    if rid == pid:
                        got = (raw, ty, ack)
            if got:
                break
            time.sleep(0.1)
        name = by_id.get(pid, f"id {pid}")
        if not got:
            rows.append((name, pid, None, None, None))
            continue
        raw, ty, ack = got
        val = decode_param(raw, ty)
        rows.append((name, pid, val, TYPE_NAMES.get(ty, ty), ack))
    for name, pid, val, ty, ack in rows:
        bounds = meta.get(name)
        b = f"  [{bounds[0]}..{bounds[1]}]" if bounds else ""
        if val is None:
            print(f"{name:34s} (id {pid:3d})  NO RESPONSE")
        else:
            print(f"{name:34s} (id {pid:3d}) = {val:<12} {ty or '':4s} ack={ack}{b}")
    return rows


def do_set(link, name_tok, value, ids, meta, by_id):
    pid = resolve(name_tok, ids)
    name = by_id.get(pid, f"id {pid}")
    ty_name = (meta.get(name) or (None, None, "PARAM_TYPE_UINT32"))[2]
    ty = {"PARAM_TYPE_UINT8": 0, "PARAM_TYPE_INT8": 1, "PARAM_TYPE_UINT16": 2,
          "PARAM_TYPE_INT16": 3, "PARAM_TYPE_UINT32": 4, "PARAM_TYPE_INT32": 5,
          "PARAM_TYPE_FLOAT": 6}[ty_name]
    raw = struct.unpack("<I", struct.pack("<i", int(value)))[0] if int(value) < 0 else int(value)
    link.send(build(OP_COMMAND, PKT_SET_PARAM, struct.pack("<IHB", raw, pid, ty)))
    time.sleep(0.4)
    rows = do_get(link, [str(pid)], ids, meta, by_id)
    got = rows[0][2]
    if got is None:
        print(f"!! {name}: no read-back; cannot confirm")
        return False
    if got != int(value):
        bounds = meta.get(name)
        extra = f" (bounds {bounds[0]}..{bounds[1]})" if bounds else ""
        print(f"!! {name}: wrote {value}, reads {got} -- REJECTED, likely out of bounds{extra}")
        return False
    print(f"ok {name} = {got} (cache only; run `apply` to push to the chip)")
    return True


def main():
    ap = argparse.ArgumentParser(description="binary protocol host console for the BMS")
    ap.add_argument("--entry", default="dap-017-serial", help="devices.toml entry")
    ap.add_argument("--suffix", default="", help="ClientID suffix, to avoid session takeover")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("status"); s.add_argument("--secs", type=float, default=3.0)
    s = sub.add_parser("get");    s.add_argument("names", nargs="+")
    s = sub.add_parser("set");    s.add_argument("name"); s.add_argument("value")
    sub.add_parser("apply")
    sub.add_parser("reset")
    s = sub.add_parser("cmd");    s.add_argument("which"); s.add_argument("param", nargs="?", default="0")
    s = sub.add_parser("watch");  s.add_argument("--secs", type=float, default=20.0)
    s.add_argument("--csv")
    s = sub.add_parser("params"); s.add_argument("filter", nargs="?", default="")

    a = ap.parse_args()
    ids, meta = param_ids(), param_meta()
    by_id = {v: k for k, v in ids.items()}

    if a.cmd == "params":
        f = a.filter.upper()
        for n, i in sorted(ids.items(), key=lambda kv: kv[1]):
            if f in n:
                b = meta.get(n)
                print(f"{i:4d}  {n:34s} {('[%s..%s] %s' % (b[0], b[1], b[2][11:].lower())) if b else ''}")
        return

    link = Link(a.entry, a.suffix)
    try:
        if a.cmd == "status":
            st = decode_status(link.collect(a.secs))
            print(fmt_status(st) if st else "no BMS_STATUS seen -- is the app running?")

        elif a.cmd == "get":
            do_get(link, a.names, ids, meta, by_id)

        elif a.cmd == "set":
            sys.exit(0 if do_set(link, a.name, a.value, ids, meta, by_id) else 1)

        elif a.cmd in ("apply", "reset"):
            if a.cmd == "apply":
                pkt = build(OP_COMMAND, PKT_SAVE_TO_ROM, bytes([0]))
            else:
                pkt = build(OP_COMMAND, PKT_RESET_ROM, struct.pack("<HB", 0xFFFF, 0))
            t0 = time.time()
            link.rx.clear()
            link.send(pkt)
            want = PKT_SAVE_TO_ROM if a.cmd == "apply" else PKT_RESET_ROM
            ack, deadline = None, time.time() + 20
            while time.time() < deadline:
                for op, p, pl in frames(bytes(link.rx)):
                    if op == OP_RESPONSE and p == want and len(pl) == 1:
                        ack = pl[0]
                if ack is not None:
                    break
                time.sleep(0.2)
            dt = time.time() - t0
            print(f"{a.cmd}: ack={ack} in {dt*1000:.0f} ms (host-observed)"
                  if ack is not None else f"{a.cmd}: NO ACK after {dt:.1f}s")
            sys.exit(0 if ack == 0 else 1)

        elif a.cmd == "cmd":
            table = bpcmds()
            if a.which in table:
                code = table[a.which]
            elif f"BPCMD_{a.which.upper()}" in table:
                code = table[f"BPCMD_{a.which.upper()}"]
            else:
                code = int(a.which, 0)
            name = next((k for k, v in table.items() if v == code), f"0x{code:02X}")
            link.send(build(OP_COMMAND, PKT_BMS_COMMAND,
                            struct.pack("<BBi", code, 0, int(a.param, 0))))
            print(f"sent {name} (0x{code:02X}) param={a.param}")
            time.sleep(1.5)
            st = decode_status(link.collect(2.0))
            if st:
                print(fmt_status(st))

        elif a.cmd == "watch":
            f = open(a.csv, "w") if a.csv else None
            if f:
                f.write(",".join(("host_ms",) + STATUS_FIELDS) + "\n")
            t0, seen, n = time.time(), None, 0
            while time.time() - t0 < a.secs:
                time.sleep(0.25)
                st = decode_status(bytes(link.rx))
                if st and st["ts"] != seen:
                    seen = st["ts"]
                    n += 1
                    print(fmt_status(st))
                    if f:
                        f.write(",".join(str(int((time.time() - t0) * 1000)))
                                + "," + ",".join(str(st[k]) for k in STATUS_FIELDS) + "\n")
                    link.rx.clear()
            if f:
                f.close()
                print(f"-- {n} samples -> {a.csv}")

    finally:
        link.close()


if __name__ == "__main__":
    main()
