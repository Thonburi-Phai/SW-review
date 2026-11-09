#!/usr/bin/env python3
"""Append-only CSV logger for a BMS session: polls /timeseries every INTERVAL s with a
wider WINDOW and appends only rows whose ts has not been written yet."""
import json, os, sys, time, urllib.request
API = "http://localhost:8081/timeseries"
INTERVAL, WINDOW = 60, 300
OUT = os.path.expanduser("~/bms_logs")
TAG = sys.argv[1] if len(sys.argv) > 1 else "session"
SERIES = {
    "status": ["ts","packV","packI","c1","c2","c3","c4","soc","soh","t1","t2","t3","t4",
               "chgFet","dsgFet","balanceActive","safetyA","safetyB","batteryStatus","powerDw"],
    "energy": ["ts","currentA","passedChargeMah","remainingCapacityMah","socRemainingMah"],
    "balance": ["ts","enabled","cmdMask","hwActiveMask","blockFlags","vminMv","vmaxMv","deltaMv",
                "currentMa","internalTempC","tickCount","writeErrCount"],
    "fault": ["ts","isConnected","cuvTripped","safetyStatusA","safetyAlertA","safetyStatusB",
              "safetyAlertB","batteryStatus","alarmStatus","extTempFaultMask","systemHealthFlags"],
}
os.makedirs(OUT, exist_ok=True)
stamp = time.strftime("%Y%m%d_%H%M%S")
files, last_ts = {}, {}
for name, cols in SERIES.items():
    path = os.path.join(OUT, f"{TAG}_{stamp}_{name}.csv")
    f = open(path, "w", buffering=1); f.write("wall_iso," + ",".join(cols) + "\n")
    files[name] = (f, cols); last_ts[name] = -1.0; print("writing", path, flush=True)
while True:
    try:
        req = urllib.request.Request(API, data=json.dumps({"seconds": WINDOW}).encode(),
                                     headers={"Content-Type": "application/json"})
        data = json.loads(urllib.request.urlopen(req, timeout=15).read())
        wall = time.strftime("%Y-%m-%dT%H:%M:%S"); total = 0
        for name, (f, cols) in files.items():
            for s in (data.get(name) or {}).get("samples", []):
                ts = s.get("ts")
                if ts is None or ts <= last_ts[name]: continue
                last_ts[name] = ts
                f.write(wall + "," + ",".join(str(s.get(c, "")) for c in cols) + "\n"); total += 1
        print(time.strftime("%H:%M:%S"), "appended", total, "rows", flush=True)
    except Exception as e:
        print(time.strftime("%H:%M:%S"), "poll error:", e, flush=True)
    time.sleep(INTERVAL)
