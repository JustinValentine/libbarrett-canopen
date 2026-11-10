#!/usr/bin/env python3
"""Analyze an existing WAM4 candump capture without connecting to hardware.

Accepts absolute-timestamp candump text (-t a) and compact logs (-L).
Select a steady ex01/ex04 interval AFTER libbarrett installs its PDO mapping.
This measures bus timing and commands, not actual motor torque. Only complete
SYNC-to-SYNC intervals contribute to per-cycle statistics.
"""

import argparse
import collections
import csv
import json
import math
import re
import struct
import sys


FRAME = re.compile(
    r"^\s*\((\d+(?:\.\d+)?)\)\s+\S+\s+([0-9a-fA-F]{3,8})"
    r"(?:\s+\[(\d+)\]\s*(.*?)\s*|#([0-9a-fA-F]*))$"
)


def parse_frame(line):
    match = FRAME.match(line)
    if not match:
        return None
    timestamp, cob, dlc, spaced, compact = match.groups()
    try:
        data = bytes.fromhex(compact if compact is not None else spaced)
    except ValueError:
        return None
    if len(data) > 8 or (dlc is not None and len(data) != int(dlc)):
        return None
    return float(timestamp), int(cob, 16), data


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)

    def percentile(fraction):
        index = (len(ordered) - 1) * fraction
        low = int(index)
        high = min(low + 1, len(ordered) - 1)
        return ordered[low] + (ordered[high] - ordered[low]) * (index - low)

    return dict(count=len(ordered), min=ordered[0], p50=percentile(.5),
                p99=percentile(.99), max=ordered[-1])


def analyze(lines, start=None, end=None, node=4, cycle_writer=None):
    sync = None
    previous_time = None
    latest_torque = None
    torque_at_sync = None
    last_keepalive = None
    previous_keepalive = None
    positions = collections.defaultdict(list)
    torque_frames_in_cycle = 0
    periods = []
    keepalive_periods = []
    keepalive_payloads = collections.Counter()
    status = collections.Counter()
    ids = collections.Counter()
    skipped_lines = 0
    malformed_frames = 0
    all_zero_frames = 0
    torque_ranges = [[None, None] for _ in range(4)]
    missing = collections.Counter()
    duplicate = collections.Counter()
    latencies = collections.defaultdict(list)
    torque_cycle_counts = collections.Counter()
    target_age_ms = []

    for line in lines:
        frame = parse_frame(line)
        if frame is None:
            if line.strip():
                skipped_lines += 1
            continue
        t, cob, data = frame
        if not math.isfinite(t):
            raise ValueError("non-finite CAN timestamp")
        if previous_time is not None and t < previous_time:
            raise ValueError("timestamps decrease; use absolute timestamps, not candump -t d")
        previous_time = t
        if (start is not None and t < start) or (end is not None and t > end):
            continue
        ids["0x%03X" % cob] += 1

        if cob == 0x080:
            if data:
                malformed_frames += 1
                continue
            if sync is not None:
                periods.append((t - sync) * 1000)
                torque_cycle_counts[torque_frames_in_cycle] += 1
                for n in range(1, 5):
                    samples = positions[n]
                    missing[n] += not samples
                    duplicate[n] += len(samples) > 1
                    if samples:
                        latencies[n].append((samples[0][0] - sync) * 1000)
                if torque_at_sync is not None:
                    target_age_ms.append((sync - torque_at_sync[0]) * 1000)
                if cycle_writer is not None:
                    samples = positions[node]
                    pos_t, pos, ka_t = samples[0] if samples else (None, None, None)
                    cycle_writer.writerow(dict(
                        sync_t="%.6f" % sync,
                        position_t="%.6f" % pos_t if pos_t is not None else "",
                        position_counts=pos if pos is not None else "",
                        position_frames=len(samples),
                        command_before_sync=(torque_at_sync[1][node - 1]
                                             if torque_at_sync is not None else ""),
                        command_age_ms=((sync - torque_at_sync[0]) * 1000
                                        if torque_at_sync is not None else ""),
                        ms_since_keepalive_at_position=((pos_t - ka_t) * 1000
                                                       if ka_t is not None else ""),
                    ))
            sync = t
            torque_at_sync = latest_torque
            positions = collections.defaultdict(list)
            torque_frames_in_cycle = 0
        elif cob == 0x201:
            if len(data) != 8:
                malformed_frames += 1
                continue
            targets = struct.unpack("<4h", data)
            latest_torque = (t, targets)
            if sync is not None:
                torque_frames_in_cycle += 1
            all_zero_frames += not any(targets)
            for bounds, value in zip(torque_ranges, targets):
                bounds[0] = value if bounds[0] is None else min(bounds[0], value)
                bounds[1] = value if bounds[1] is None else max(bounds[1], value)
        elif 0x181 <= cob <= 0x184:
            if len(data) != 4:
                malformed_frames += 1
                continue
            if sync is not None:
                positions[cob - 0x180].append(
                    (t, struct.unpack("<i", data)[0], last_keepalive))
        elif cob == 0x28A:
            if len(data) not in (2, 3):
                malformed_frames += 1
                continue
            keepalive_payloads[data.hex().upper()] += 1
            last_keepalive = t
            if previous_keepalive is not None:
                keepalive_periods.append((t - previous_keepalive) * 1000)
            previous_keepalive = t
        elif cob == 0x280 + node:
            if len(data) != 3:
                malformed_frames += 1
                continue
            sw, mode = struct.unpack("<Hb", data)
            label = "sw=0x%04X mode=%d enabled=%s" % (sw, mode, (sw & 0x6F) == 0x27)
            status[label] += 1

    return dict(
        complete_cycles=len(periods),
        sync_period_ms=distribution(periods),
        sync_gaps_over_3ms=sum(p > 3 for p in periods),
        torque_frames_per_complete_cycle=dict(torque_cycle_counts),
        torque_command_age_at_sync_ms=distribution(target_age_ms),
        torque_frames_all_zero=all_zero_frames,
        torque_frames_total=ids["0x201"],
        torque_ranges_permille={str(n + 1): bounds for n, bounds in enumerate(torque_ranges)},
        position_feedback={str(n): dict(
            first_arrival_after_sync_ms=distribution(latencies[n]),
            missing_cycles=missing[n], duplicate_cycles=duplicate[n]) for n in range(1, 5)},
        keepalive_period_ms=distribution(keepalive_periods),
        keepalive_payloads=dict(keepalive_payloads),
        selected_node_status_frames=dict(status),
        frame_counts=dict(ids), ignored_or_unparsed_lines=skipped_lines,
        unexpected_frame_lengths=malformed_frames,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--start", type=float, help="inclusive Unix timestamp")
    parser.add_argument("--end", type=float, help="inclusive Unix timestamp")
    parser.add_argument("--node", type=int, choices=range(1, 5), default=4)
    parser.add_argument("--cycles-csv", help="create per-cycle CSV for the selected motor")
    args = parser.parse_args()
    if any(v is not None and not math.isfinite(v) for v in (args.start, args.end)):
        parser.error("time bounds must be finite")
    if args.start is not None and args.end is not None and args.end <= args.start:
        parser.error("--end must be greater than --start")
    output = None
    try:
        writer = None
        if args.cycles_csv:
            # Exclusive creation also prevents accidentally truncating the input log.
            output = open(args.cycles_csv, "x", newline="")
            writer = csv.DictWriter(output, fieldnames=[
                "sync_t", "position_t", "position_counts", "position_frames",
                "command_before_sync", "command_age_ms", "ms_since_keepalive_at_position"])
            writer.writeheader()
        with open(args.log) as source:
            report = analyze(source, args.start, args.end, args.node, writer)
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
        if not report["complete_cycles"]:
            print("No complete SYNC intervals; verify the format and selected window.", file=sys.stderr)
            return 1
        if report["unexpected_frame_lengths"]:
            print("Unexpected PDO lengths: verify the window uses the WAM4 host mapping.", file=sys.stderr)
            return 1
        return 0
    except (OSError, ValueError) as error:
        parser.exit(2, "%s\n" % error)
    finally:
        if output is not None:
            output.close()


if __name__ == "__main__":
    sys.exit(main())
