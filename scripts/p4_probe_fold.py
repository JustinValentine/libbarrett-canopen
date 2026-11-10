#!/usr/bin/env python3
"""Fold the columns of an SDO probe CSV (iq_probe / v_probe / p4_enable_probe
logs) by the drive's reported electrical angle, using the documented scale
(0x60EA: 65536 counts per electrical revolution, P4 manual RevBA p.26), and
report bin coverage so an incomplete sweep is not read as a full waveform.

    p4_probe_fold.py FILE.csv [--bins 36] [--cols ud,uq,ifb]

Replaces iq_analyze.py / v_analyze.py, which guessed the angle scale from the
data range and filled unvisited bins with zero.
"""
import argparse
import csv
import math

THETA_FULL_SCALE = 65536.0


def fold(rows, col, nbins):
    bins = [[] for _ in range(nbins)]
    for r in rows:
        try:
            th = float(r["theta_e"]); v = float(r[col])
        except (KeyError, ValueError):
            continue
        bins[int((th % THETA_FULL_SCALE) / THETA_FULL_SCALE * nbins) % nbins].append(v)
    vals = [float(r[col]) for r in rows if r.get(col) not in (None, "")]
    mean = sum(vals) / len(vals) if vals else 0.0
    means = [sum(b) / len(b) - mean if len(b) >= 2 else None for b in bins]
    valid = [(i, m) for i, m in enumerate(means) if m is not None]
    coverage = len(valid) / nbins
    out = dict(coverage=coverage, counts=[len(b) for b in bins], means=means, pp=None, harmonics={})
    if len(valid) >= nbins // 2:
        out["pp"] = max(m for _, m in valid) - min(m for _, m in valid)
        # harmonics over the VISITED bins only (least-squares on the covered set)
        for k in (1, 2, 3, 6):
            c = sum(m * math.cos(2 * math.pi * k * (i + .5) / nbins) for i, m in valid) * 2 / len(valid)
            s = sum(m * math.sin(2 * math.pi * k * (i + .5) / nbins) for i, m in valid) * 2 / len(valid)
            out["harmonics"][k] = math.hypot(c, s)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--bins", type=int, default=36)
    ap.add_argument("--cols", default=None, help="comma-separated columns; default: every numeric column except t/pos/theta_e")
    args = ap.parse_args()
    rows = list(csv.DictReader(open(args.csv)))
    if not rows:
        raise SystemExit("empty file")
    cols = args.cols.split(",") if args.cols else [c for c in rows[0].keys() if c not in ("t", "pos", "theta_e", "label")]
    th = [float(r["theta_e"]) for r in rows if r.get("theta_e") not in (None, "")]
    print("%d rows over %.1f s; theta_e range %d..%d of %d (scale fixed by the manual)" % (
        len(rows), float(rows[-1]["t"]) - float(rows[0]["t"]), min(th), max(th), int(THETA_FULL_SCALE)))
    for col in cols:
        f = fold(rows, col, args.bins)
        if f["pp"] is None:
            print("%-8s coverage %3.0f%% of bins - too sparse for a waveform" % (col, 100 * f["coverage"]))
            continue
        h = f["harmonics"]
        print("%-8s coverage %3.0f%%  p-p %8.2f  harmonics 1x %.2f 2x %.2f 3x %.2f 6x %.2f  (units of the column)" % (
            col, 100 * f["coverage"], f["pp"], h[1], h[2], h[3], h[6]))
        print("         bins: " + " ".join("%+6.1f" % m if m is not None else "   ---" for m in f["means"]))


if __name__ == "__main__":
    main()
