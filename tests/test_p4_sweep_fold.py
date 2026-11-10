#!/usr/bin/env python3
"""p4_sweep_fold must tell an angle-locked disturbance from a keepalive-locked one
on a synthetic hand sweep with realistic encoder dither.  Run: python3 tests/test_p4_sweep_fold.py"""
import io
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import p4_sweep_fold as sf

CPE = 4096.0 / 6


def synth(kind, seconds=24.0, seed=1):
    """candump -t a lines: SYNC every 2 ms, keepalive every 50 ms, node-4 position TPDO per cycle.
    kind: 'angle' -> velocity bump once per electrical cycle; 'time' -> bump 4-10 ms after each keepalive; 'none'."""
    rnd = random.Random(seed)
    lines = []
    t = 1789700000.0
    pos = 0.0
    last_ka = None
    n = int(seconds / 0.002)
    for k in range(n):
        # hand sweep: triangle wave of speed, both directions, 2 speeds (slow first half, fast second half)
        phase = (k * 0.002) % 6.0
        base = 1500.0 if k < n // 2 else 3000.0
        speed = base if phase < 3.0 else -base
        speed *= 0.8 + 0.4 * math.sin(k * 0.002 * 0.7)      # slow variation of the hand
        lines.append(" (%.6f)  can20  080   [0] " % t)
        if last_ka is None or t - last_ka >= 0.050 - 1e-9:
            lines.append(" (%.6f)  can20  28A   [3]  0F 00 0A" % (t + 0.0003))
            last_ka = t + 0.0003
        disturb = 0.0
        if kind == "angle":
            disturb = 600.0 * math.cos(2 * math.pi * (pos % CPE) / CPE)
        elif kind == "time":
            dtk = (t + 0.00126) - last_ka
            disturb = 900.0 if 0.004 <= dtk < 0.010 else 0.0
        pos += (speed + disturb) * 0.002
        reported = int(round(pos + rnd.uniform(-2, 2)))
        v = reported & 0xFFFFFFFF
        lines.append(" (%.6f)  can20  184   [4]  %02X %02X %02X %02X" % (t + 0.00126, v & 255, (v >> 8) & 255, (v >> 16) & 255, (v >> 24) & 255))
        lines.append(" (%.6f)  can20  201   [8]  00 00 00 00 00 00 00 00" % (t + 0.00135))
        t += 0.002
    return lines


def folds(kind, seconds=24.0):
    samples = sf.load_samples(synth(kind, seconds), 4, None, None, {0x28A, 0x304})
    vel, smooth, res = sf.residual_velocity(samples)
    idx = [i for i, s in enumerate(smooth) if s is not None and abs(s) > 0.05 * CPE]
    r = [res[i] for i in idx]
    fa = sf.fold(r, [samples[i][1] for i in idx], 24, CPE)
    fk = sf.fold(r, [samples[i][2] for i in idx], 10, 0.050)
    return fa, fk


def cross_speed(kind):
    """fold each speed half separately (the synthetic sweep is slow for the first half, fast for the second)."""
    samples = sf.load_samples(synth(kind), 4, None, None, {0x28A, 0x304})
    n = len(samples)
    out = []
    for sub in (samples[: n // 2], samples[n // 2:]):
        vel, smooth, res = sf.residual_velocity(sub)
        idx = [i for i, s in enumerate(smooth) if s is not None and abs(s) > 0.05 * CPE]
        r = [res[i] for i in idx]
        out.append((sf.fold(r, [sub[i][1] for i in idx], 24, CPE), sf.fold(r, [sub[i][2] for i in idx], 10, 0.050)))
    (fa1, fk1), (fa2, fk2) = out
    return sf.correlation(fa1["means"], fa2["means"]), sf.correlation(fk1["means"], fk2["means"])


class SweepFoldTest(unittest.TestCase):
    def test_angle_locked_shows_in_angle_fold_only(self):
        fa, fk = folds("angle")
        self.assertGreater(fa["snr"], 6.0, fa)
        self.assertLess(fk["snr"], 3.0, fk)
        self.assertGreater(fa["pp"], 3 * fk["pp"])

    def test_time_locked_shows_in_keepalive_fold_only(self):
        fa, fk = folds("time")
        self.assertGreater(fk["snr"], 6.0, fk)
        self.assertLess(fa["snr"], 3.0, fa)
        self.assertGreater(fk["pp"], 3 * fa["pp"])

    def test_no_disturbance_shows_in_neither(self):
        fa, fk = folds("none")
        self.assertLess(fa["snr"], 3.0, fa)
        self.assertLess(fk["snr"], 3.0, fk)

    def test_angle_pattern_agrees_across_speeds(self):
        r_angle, r_keepalive = cross_speed("angle")
        self.assertGreater(r_angle, 0.8, (r_angle, r_keepalive))
        self.assertLess(abs(r_keepalive), 0.6, (r_angle, r_keepalive))

    def test_keepalive_pattern_agrees_across_speeds(self):
        r_angle, r_keepalive = cross_speed("time")
        self.assertGreater(r_keepalive, 0.8, (r_angle, r_keepalive))
        self.assertLess(abs(r_angle), 0.6, (r_angle, r_keepalive))

    def test_report_runs(self):
        samples = sf.load_samples(synth("angle", seconds=8.0), 4, None, None, {0x28A, 0x304})
        out = io.StringIO(); old = sys.stdout; sys.stdout = out
        try:
            sf.report(samples, CPE, 4096.0, 50.0, 24, "test")
        finally:
            sys.stdout = old
        self.assertIn("electrical angle", out.getvalue())
        self.assertIn("keepalive", out.getvalue())


if __name__ == "__main__":
    unittest.main()
