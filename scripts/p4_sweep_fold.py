#!/usr/bin/env python3
"""Does the cogging repeat at a fixed MOTOR ANGLE or at a fixed TIME after the
safety board's keepalive?  Offline, from an existing candump capture of a
hand-sweep test (doc/p4-cogging-review.md, "What to test first").

    p4_sweep_fold.py LOG --start T0 --end T1 --node 4

For the selected motor the script takes the position TPDO of every cycle,
differentiates it to a velocity, removes the slow hand motion with a running
mean, and folds the residual three ways:

  angle_e   by electrical angle  (position mod counts-per-electrical-cycle)
  angle_m   by mechanical angle  (position mod counts-per-revolution)
  keepalive by time since the last keepalive frame (0x28A or 0x300+node)

For each fold it reports the modulation (peak-to-peak of the bin means), the
bin coverage, and a signal-to-noise figure (peak-to-peak divided by the typical
standard error of a bin mean); a fold that is real should stay above ~3 in
both sweep directions and at both speeds, and only ONE of the folds should.
A coarse spectrum of the residual is printed as well, with the electrical
frequency predicted from the mean speed, its harmonics, and the keepalive rate
marked.  Bus receive timestamps bound the timing; nothing here measures torque.
"""
import argparse
import collections
import math
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from analyze_p4_cogging_capture import parse_frame
except ImportError:  # keep this file usable on its own
    FRAME = re.compile(r"^\s*\((\d+(?:\.\d+)?)\)\s+\S+\s+([0-9a-fA-F]{3,8})"
                       r"(?:\s+\[(\d+)\]\s*(.*?)\s*|#([0-9a-fA-F]*))$")

    def parse_frame(line):
        m = FRAME.match(line)
        if not m:
            return None
        ts, cob, dlc, spaced, compact = m.groups()
        try:
            data = bytes.fromhex(compact if compact is not None else spaced)
        except ValueError:
            return None
        if len(data) > 8 or (dlc is not None and len(data) != int(dlc)):
            return None
        return float(ts), int(cob, 16), data


def load_samples(lines, node, start, end, keepalive_cobs):
    """-> list of (t, pos, t_since_keepalive or None), in time order."""
    samples = []
    last_ka = None
    for line in lines:
        f = parse_frame(line)
        if f is None:
            continue
        t, cob, data = f
        if (start is not None and t < start) or (end is not None and t > end):
            continue
        if cob in keepalive_cobs:
            last_ka = t
        elif cob == 0x180 + node and len(data) == 4:
            samples.append((t, struct.unpack("<i", data)[0], None if last_ka is None else t - last_ka))
    return samples


def residual_velocity(samples, half=2, mean_window=125):
    """velocity by central difference over +-half samples, minus a running mean."""
    n = len(samples)
    vel = [None] * n
    for i in range(half, n - half):
        dt = samples[i + half][0] - samples[i - half][0]
        if dt > 0:
            vel[i] = (samples[i + half][1] - samples[i - half][1]) / dt
    res = [None] * n
    smooth = [None] * n
    csum = [0.0]
    cnt = [0]
    for v in vel:
        csum.append(csum[-1] + (v or 0.0))
        cnt.append(cnt[-1] + (1 if v is not None else 0))
    for i in range(n):
        if vel[i] is None:
            continue
        lo, hi = max(0, i - mean_window), min(n, i + mean_window + 1)
        k = cnt[hi] - cnt[lo]
        if k:
            smooth[i] = (csum[hi] - csum[lo]) / k
            res[i] = vel[i] - smooth[i]
    return vel, smooth, res


def fold(values, keys, nbins, span):
    bins = [[] for _ in range(nbins)]
    for v, k in zip(values, keys):
        if v is None or k is None:
            continue
        bins[int((k % span) / span * nbins) % nbins].append(v)
    means, ses = [], []
    for b in bins:
        if len(b) >= 2:
            m = sum(b) / len(b)
            sd = math.sqrt(sum((x - m) ** 2 for x in b) / (len(b) - 1))
            means.append(m); ses.append(sd / math.sqrt(len(b)))
        else:
            means.append(None); ses.append(None)
    valid = [m for m in means if m is not None]
    if len(valid) < 2:
        return dict(pp=None, snr=None, coverage=len(valid) / nbins, means=means, counts=[len(b) for b in bins])
    pp = max(valid) - min(valid)
    se = sorted(s for s in ses if s is not None)[len(valid) // 2]
    return dict(pp=pp, snr=(pp / (2 * se) if se > 0 else float("inf")), coverage=len(valid) / nbins,
                means=means, counts=[len(b) for b in bins])


def correlation(a, b):
    """Pearson r over bins present in both folds (None where a fold is empty)."""
    pairs = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
    if len(pairs) < 4:
        return None
    ma = sum(x for x, _ in pairs) / len(pairs); mb = sum(y for _, y in pairs) / len(pairs)
    sa = math.sqrt(sum((x - ma) ** 2 for x, _ in pairs)); sb = math.sqrt(sum((y - mb) ** 2 for _, y in pairs))
    if sa == 0 or sb == 0:
        return None
    return sum((x - ma) * (y - mb) for x, y in pairs) / (sa * sb)


def spectrum(values, dt, freqs):
    xs = [v for v in values if v is not None]
    n = len(xs)
    out = {}
    for f in freqs:
        re_ = im_ = 0.0
        w = 2 * math.pi * f * dt
        for i, x in enumerate(xs):
            re_ += x * math.cos(w * i); im_ += x * math.sin(w * i)
        out[f] = 2 * math.hypot(re_, im_) / max(1, n)
    return out


def report(samples, cpe, cpr, keepalive_ms, nbins=24, label="all"):
    vel, smooth, res = residual_velocity(samples)
    moving = [i for i, s in enumerate(smooth) if s is not None and abs(s) > 0.05 * cpe]
    if len(moving) < 200:
        print("[%s] too little motion (%d moving samples)" % (label, len(moving)))
        return None
    idx = moving
    r = [res[i] for i in idx]
    pos = [samples[i][1] for i in idx]
    tka = [samples[i][2] for i in idx]
    mean_speed = sum(abs(smooth[i]) for i in idx) / len(idx)
    fe = mean_speed / cpe
    print("[%s] %d moving samples, mean |motor speed| %.0f counts/s = %.2f electrical cycles/s"
          % (label, len(idx), mean_speed, fe))
    fa = fold(r, pos, nbins, cpe)
    fm = fold(r, pos, nbins, cpr)
    fk = fold(r, tka, 10, keepalive_ms / 1000.0) if any(k is not None for k in tka) else None
    for name, fd in (("electrical angle", fa), ("mechanical angle", fm), ("time since keepalive", fk)):
        if fd is None:
            print("   %-22s no keepalive frames in this window" % name)
        elif fd["pp"] is None:
            print("   %-22s insufficient coverage (%.0f%% of bins)" % (name, 100 * fd["coverage"]))
        else:
            print("   %-22s p-p %6.0f counts/s   snr %5.1f   coverage %3.0f%%" % (name, fd["pp"], fd["snr"], 100 * fd["coverage"]))
            print("      bins: " + " ".join("%+5.0f" % m if m is not None else "  ---" for m in fd["means"]))
    dt = (samples[idx[-1]][0] - samples[idx[0]][0]) / max(1, len(idx) - 1)
    marks = {"keepalive": 1000.0 / keepalive_ms, "1x elec": fe, "2x elec": 2 * fe, "6x elec": 6 * fe, "1x mech": mean_speed / cpr}
    grid = [float(f) for f in range(5, 101)]
    sp = spectrum(r, dt, grid + sorted(set(round(v, 2) for v in marks.values() if 0.5 < v < 250)))
    floor = sorted(sp[f] for f in grid)[len(grid) // 2]
    print("   spectrum of the residual (amplitude / median 5-100 Hz level; below ~5 Hz the hand's own motion leaks in):")
    for name, f in sorted(marks.items(), key=lambda kv: kv[1]):
        if 0.5 < f < 250:
            print("      %-10s %6.2f Hz   x%.1f" % (name, f, sp[round(f, 2)] / floor if floor else float("nan")))
    top = sorted(((sp[f] / floor if floor else 0, f) for f in grid), reverse=True)[:3]
    print("      strongest grid peaks: " + ", ".join("%.0f Hz x%.1f" % (f, a) for a, f in top))
    return dict(angle_e=fa, angle_m=fm, keepalive=fk)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="candump -t a capture, or (with --csv) a p4_enable_probe --fastlog CSV")
    ap.add_argument("--csv", action="store_true", help="read t,pos from a probe CSV instead of a candump (no keepalive fold)")
    ap.add_argument("--start", type=float)
    ap.add_argument("--end", type=float)
    ap.add_argument("--node", type=int, default=4)
    ap.add_argument("--cpr", type=float, default=4096.0, help="encoder counts per motor revolution")
    ap.add_argument("--pole-pairs", type=int, default=6)
    ap.add_argument("--keepalive-ms", type=float, default=50.0)
    ap.add_argument("--bins", type=int, default=24)
    args = ap.parse_args()
    cpe = args.cpr / args.pole_pairs
    if args.csv:
        import csv as _csv
        samples = []
        for r in _csv.DictReader(open(args.log)):
            try:
                t = float(r["t"]); pos = int(r["pos"])
            except (KeyError, ValueError):
                continue
            if (args.start is not None and t < args.start) or (args.end is not None and t > args.end):
                continue
            samples.append((t, pos, None))
    else:
        with open(args.log) as f:
            samples = load_samples(f, args.node, args.start, args.end, {0x28A, 0x300 + args.node})
    if len(samples) < 300:
        sys.exit("only %d position samples for node %d in the window" % (len(samples), args.node))
    print("node %d: %d position samples over %.1f s; counts/elec cycle %.2f" % (
        args.node, len(samples), samples[-1][0] - samples[0][0], cpe))
    report(samples, cpe, args.cpr, args.keepalive_ms, args.bins, "all")
    # per direction, then per speed half, so a real fold can be checked for consistency
    vel, smooth, _ = residual_velocity(samples)
    for sign, name in ((1, "moving +"), (-1, "moving -")):
        sub = [s for s, v in zip(samples, smooth) if v is not None and v * sign > 0.05 * cpe]
        if len(sub) > 300:
            report(sub, cpe, args.cpr, args.keepalive_ms, args.bins, name)
    speeds = sorted(abs(v) for v in smooth if v is not None and abs(v) > 0.05 * cpe)
    halves = {}
    if len(speeds) > 600:
        med = speeds[len(speeds) // 2]
        for cond, name in ((lambda v: abs(v) <= med, "slower half"), (lambda v: abs(v) > med, "faster half")):
            sub = [s for s, v in zip(samples, smooth) if v is not None and abs(v) > 0.05 * cpe and cond(v)]
            if len(sub) > 300:
                halves[name] = report(sub, cpe, args.cpr, args.keepalive_ms, args.bins, name)
    if len(halves) == 2 and all(halves.values()):
        a, b = halves["slower half"], halves["faster half"]
        print("consistency of each fold between the slower and faster halves (Pearson r of the bin means):")
        print("   a disturbance locked to angle keeps its angle pattern at both speeds; one locked to the")
        print("   keepalive keeps its keepalive pattern.  At ONE steady speed a time-locked bump also looks")
        print("   periodic in angle, so only this cross-speed agreement, not a single fold, decides it.")
        for key, name in (("angle_e", "electrical angle"), ("angle_m", "mechanical angle"), ("keepalive", "time since keepalive")):
            if a[key] is None or b[key] is None:
                print("   %-22s n/a" % name); continue
            rr = correlation(a[key]["means"], b[key]["means"])
            print("   %-22s r = %s" % (name, "%.2f" % rr if rr is not None else "n/a"))


if __name__ == "__main__":
    main()
