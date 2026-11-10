#!/usr/bin/env python3
"""Offline regression tests for the cogging capture analysis."""

import csv
import importlib.util
import io
from pathlib import Path
import struct
import unittest


path = Path(__file__).resolve().parents[1] / "scripts" / "analyze_p4_cogging_capture.py"
spec = importlib.util.spec_from_file_location("capture", path)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def frame(t, cob, data=b"", compact=False):
    if compact:
        return "(%.6f) can20 %03X#%s\n" % (t, cob, data.hex())
    return "(%.6f) can20 %03X [%d] %s\n" % (t, cob, len(data), data.hex(" "))


class CaptureTests(unittest.TestCase):
    def test_formats_and_signed_payloads(self):
        data = struct.pack("<4h", -1, 0, 10, -32768)
        for compact in (False, True):
            self.assertEqual(capture.parse_frame(frame(1, 0x201, data, compact)),
                             (1, 0x201, data))
            self.assertEqual(capture.parse_frame(frame(1, 0x80, compact=compact)),
                             (1, 0x80, b""))
        self.assertIsNone(capture.parse_frame("(1.0) can20 201 [8] 00\n"))
        self.assertIsNone(capture.parse_frame("(1.0) can20 201##100\n"))

    def test_complete_cycles_jitter_and_feedback_loss(self):
        lines = [frame(1, 0x80)]
        for n in range(1, 5):
            lines.append(frame(1.001, 0x180 + n, struct.pack("<i", -n)))
        lines += [frame(1.0015, 0x201, bytes(8)), frame(1.002, 0x80),
                  frame(1.003, 0x184, struct.pack("<i", 30)),
                  frame(1.0031, 0x184, struct.pack("<i", 31)),
                  frame(1.006, 0x80)]
        report = capture.analyze(lines)
        self.assertEqual(report["complete_cycles"], 2)
        self.assertAlmostEqual(report["sync_period_ms"]["max"], 4)
        self.assertEqual(report["sync_gaps_over_3ms"], 1)
        self.assertEqual(report["position_feedback"]["1"]["missing_cycles"], 1)
        self.assertEqual(report["position_feedback"]["4"]["duplicate_cycles"], 1)
        self.assertEqual(report["position_feedback"]["4"]["missing_cycles"], 0)
        self.assertEqual(report["torque_frames_per_complete_cycle"], {1: 1, 0: 1})
        self.assertAlmostEqual(report["torque_command_age_at_sync_ms"]["p50"], .5)

    def test_window_does_not_claim_unknown_commands_are_zero(self):
        lines = [frame(1, 0x201, struct.pack("<4h", 0, 0, 0, 25)),
                 frame(2, 0x80), frame(2.002, 0x80)]
        report = capture.analyze(lines, start=2, end=2.002)
        self.assertEqual(report["complete_cycles"], 1)
        self.assertEqual(report["torque_frames_total"], 0)
        self.assertIsNone(report["torque_command_age_at_sync_ms"])
        self.assertEqual(report["torque_ranges_permille"]["4"], [None, None])

    def test_zero_and_nonzero_commands(self):
        lines = [frame(1, 0x201, bytes(8)),
                 frame(1.001, 0x201, struct.pack("<4h", -3, 2, 0, 8))]
        report = capture.analyze(lines)
        self.assertEqual(report["torque_frames_all_zero"], 1)
        self.assertEqual(report["torque_frames_total"], 2)
        self.assertEqual(report["torque_ranges_permille"]["1"], [-3, 0])
        self.assertEqual(report["torque_ranges_permille"]["4"], [0, 8])

    def test_causal_csv_and_keepalive_status(self):
        lines = [frame(.99, 0x28A, bytes.fromhex("0f000a")),
                 frame(.999, 0x201, struct.pack("<4h", 0, 0, 0, -2)),
                 frame(1, 0x80),
                 frame(1.0005, 0x284, struct.pack("<Hb", 0x27, 10)),
                 frame(1.001, 0x184, struct.pack("<i", -123)),
                 frame(1.0011, 0x28A, bytes.fromhex("0f000a")),
                 frame(1.0012, 0x201, struct.pack("<4h", 0, 0, 0, 5)),
                 frame(1.002, 0x80)]
        out = io.StringIO()
        names = ["sync_t", "position_t", "position_counts", "position_frames",
                 "command_before_sync", "command_age_ms", "ms_since_keepalive_at_position"]
        writer = csv.DictWriter(out, fieldnames=names)
        writer.writeheader()
        report = capture.analyze(lines, cycle_writer=writer)
        row, = list(csv.DictReader(io.StringIO(out.getvalue())))
        self.assertEqual(row["command_before_sync"], "-2")
        self.assertEqual(row["position_counts"], "-123")
        self.assertAlmostEqual(float(row["ms_since_keepalive_at_position"]), 11)
        self.assertEqual(report["keepalive_payloads"], {"0F000A": 2})
        self.assertEqual(report["selected_node_status_frames"],
                         {"sw=0x0027 mode=10 enabled=True": 1})

    def test_bad_timestamps_and_wrong_mapping(self):
        with self.assertRaisesRegex(ValueError, "timestamps decrease"):
            capture.analyze([frame(2, 0x80), frame(1, 0x80)])
        report = capture.analyze([frame(1, 0x184, bytes(8)),
                                  frame(1, 0x201, bytes(2))])
        self.assertEqual(report["unexpected_frame_lengths"], 2)


if __name__ == "__main__":
    unittest.main()
