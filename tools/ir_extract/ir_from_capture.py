#!/usr/bin/env python3
"""Extract per-cell internal resistance from a captured BMS log.

Deliberately offline.  Nothing here runs on the target: the point is to find out what the
pack's resistance actually is, and how it varies with state of charge and direction, before
deciding whether the firmware should measure it for itself.

Method: find steps where the current changes sharply between consecutive samples and is
steady on both sides, then take dV/dI per cell across the step.  No deliberate pulse is
needed; ordinary charger ramps, load changes and protection trips supply the steps.

Two caveats the numbers inherit and the report states:

  * The voltage samples lag the current samples by about one frame in this logger, so the
    pair either side of a step must be chosen with that offset or every result is wrong by
    the size of the step.  Run with --lag 0 and --lag 1 to see it.
  * A step measured 0.4 s apart is not pure ohmic resistance; it already contains the fast
    part of the diffusion response.  It is "resistance at the sampling interval", which is
    the right quantity for a 10 s decision and the wrong one for "where would this cell
    rest".  The slow figure needs a steady-current decay, not a step, and is not measured
    here.

Usage:
    python3 ir_from_capture.py <status.csv[.gz]> [more...] [--min-step 8] [--lag 1]
"""
import csv
import gzip
import io
import sys
from collections import defaultdict

NCELL = 4


def load(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as fh:
        rows = []
        for r in csv.DictReader(fh):
            try:
                rows.append({
                    "ts": float(r["ts"]),
                    "i": float(r["packI"]),
                    "v": [float(r[f"c{k+1}"]) * 1000.0 for k in range(NCELL)],
                    "soc": float(r["soc"]),
                })
            except (KeyError, ValueError):
                continue
    rows.sort(key=lambda x: x["ts"])
    # the logger appends in batches and can repeat a timestamp; keep the first of each
    out, seen = [], set()
    for r in rows:
        key = round(r["ts"], 3)
        if key in seen:
            continue
        seen.add(key)
        out.append(r)
    return out


def steady(rows, lo, hi, tol):
    """True if the current holds within tol amps across rows[lo:hi]."""
    if lo < 0 or hi > len(rows) or hi - lo < 2:
        return False
    vals = [r["i"] for r in rows[lo:hi]]
    return max(vals) - min(vals) <= tol


def find_steps(rows, min_step, lag, settle=3, tol=3.0, max_gap_s=1.0):
    """Yield (index, dI, [dV per cell], soc, direction) for each usable current step."""
    for n in range(settle + 1, len(rows) - settle - lag - 1):
        a, b = rows[n - 1], rows[n]
        di = b["i"] - a["i"]
        if abs(di) < min_step:
            continue
        if b["ts"] - a["ts"] > max_gap_s:
            continue
        # steady before the step and after it has settled
        if not steady(rows, n - 1 - settle, n, tol):
            continue
        if not steady(rows, n + lag, n + lag + settle, tol):
            continue
        va = rows[n - 1]["v"]
        vb = rows[n + lag]["v"]
        dv = [vb[k] - va[k] for k in range(NCELL)]
        # re-measure dI over the same pair actually used for the voltages
        di_used = rows[n + lag]["i"] - rows[n - 1]["i"]
        if abs(di_used) < min_step:
            continue
        direction = "charge" if di_used > 0 else "discharge"
        yield n, di_used, dv, a["soc"], direction


def soc_bin(soc):
    for lo in (0, 10, 25, 40, 55, 70, 85):
        if soc < lo + 15:
            return f"{lo}-{lo+15}%"
    return "85-100%"


def main():
    args = [a for a in sys.argv[1:]]
    min_step, lag = 8.0, 1
    paths = []
    i = 0
    while i < len(args):
        if args[i] == "--min-step":
            min_step = float(args[i + 1]); i += 2
        elif args[i] == "--lag":
            lag = int(args[i + 1]); i += 2
        else:
            paths.append(args[i]); i += 1
    if not paths:
        print(__doc__)
        return 1

    buckets = defaultdict(list)     # (direction, socbin) -> [ [r per cell] ]
    total = 0
    for p in paths:
        rows = load(p)
        n_here = 0
        for _, di, dv, soc, direction in find_steps(rows, min_step, lag):
            r = [abs(dv[k] / di) for k in range(NCELL)]     # mV / A = mOhm
            if max(r) > 20.0:        # nonsense, usually a sample spike
                continue
            buckets[(direction, soc_bin(soc))].append(r)
            n_here += 1
        print(f"{p}: {len(rows)} samples, {n_here} usable steps "
              f"(>= {min_step:.0f} A, lag {lag})")
        total += n_here

    if not total:
        print("\nno usable steps found")
        return 1

    print(f"\nper-cell resistance in mOhm, median of each bin  (lag {lag} sample)\n")
    print(f"{'direction':10s} {'SOC':10s} {'n':>4s}   " +
          "  ".join(f"cell{k+1:d}" for k in range(NCELL)))
    for key in sorted(buckets, key=lambda k: (k[0], k[1])):
        rows_ = buckets[key]
        med = []
        for k in range(NCELL):
            vals = sorted(r[k] for r in rows_)
            med.append(vals[len(vals) // 2])
        print(f"{key[0]:10s} {key[1]:10s} {len(rows_):4d}   " +
              "  ".join(f"{m:5.2f}" for m in med))

    allr = [r for rows_ in buckets.values() for r in rows_]
    print()
    for k in range(NCELL):
        vals = sorted(r[k] for r in allr)
        n = len(vals)
        print(f"cell{k+1}: median {vals[n//2]:.2f} mOhm, "
              f"10th {vals[n//10]:.2f}, 90th {vals[9*n//10]:.2f}, n={n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
