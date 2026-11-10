#!/usr/bin/env python3
"""Send SYNC frames and count what comes back, with candump as the counter.

Every SYNC makes all configured pucks transmit their synchronous TPDOs at
once, so this exercises simultaneous transmitters and burst delivery
through the host adapter.  Sends nothing but SYNC (0x080, no data): no
torque, no controlword, no configuration.  Run it with the arm E-stopped or
idle and no WAM program on the bus.

Counting is done by a candump subprocess (a C consumer that keeps up with
a saturated 1 Mbit/s bus); an earlier version counted in Python with a
per-cycle window and reported losses that did not exist.  The report gives,
per puck, how many SYNC-triggered frames arrived versus SYNCs sent, any
boot-up messages (a puck that rebooted during the run), CAN error frames
(controller state changes), and the host's error-state counters.

    python3 sync_probe.py --iface can1 --rate 500 --count 2500 --nodes 1-8
"""
import argparse
import collections
import re
import shutil
import subprocess
import sys
import tempfile
import time

import can


def counters(iface):
    out = subprocess.run(['ip', '-details', '-statistics', 'link', 'show', iface],
                         capture_output=True, text=True).stdout
    state = re.search(r'can state ([A-Z-]+)', out)
    m = re.search(r're-started.*?\n\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)', out)
    rx = re.search(r'RX:.*?\n\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)', out)
    return {
        'state': state.group(1) if state else '?',
        'bus_errors': int(m.group(2)) if m else -1,
        'error_warning': int(m.group(4)) if m else -1,
        'error_passive': int(m.group(5)) if m else -1,
        'bus_off': int(m.group(6)) if m else -1,
        'rx_packets': int(rx.group(2)) if rx else -1,
        'rx_dropped': int(rx.group(4)) if rx else -1,
    }


def parse_nodes(text):
    nodes = set()
    for part in text.split(','):
        if '-' in part:
            a, b = part.split('-')
            nodes.update(range(int(a), int(b) + 1))
        else:
            nodes.add(int(part))
    return sorted(nodes)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--iface', default='can1')
    ap.add_argument('--rate', type=float, default=500.0, help='SYNC rate, Hz')
    ap.add_argument('--count', type=int, default=2500, help='number of SYNCs')
    ap.add_argument('--nodes', default='1-8', help='pucks expected to answer, e.g. 1-8 or 1,2,3')
    ap.add_argument('--keep', metavar='FILE', help='keep the candump log at this path')
    args = ap.parse_args()
    nodes = parse_nodes(args.nodes)
    period = 1.0 / args.rate

    busy = subprocess.run(['pgrep', '-a', 'lbc_|ex0|bt-wam'], capture_output=True, text=True).stdout.strip()
    if busy:
        sys.exit('A WAM program is on the bus; refusing to send SYNC:\n' + busy)
    if shutil.which('candump') is None:
        sys.exit('candump (can-utils) is required')

    before = counters(args.iface)
    print('before:', before)

    log = open(args.keep or tempfile.mktemp(prefix='sync_probe-', suffix='.log'), 'w')
    dump = subprocess.Popen(['candump', '-t', 'a', '-e', args.iface], stdout=log, stderr=subprocess.DEVNULL)
    time.sleep(0.3)

    bus = can.interface.Bus(channel=args.iface, interface='socketcan')
    sync = can.Message(arbitration_id=0x080, is_extended_id=False, data=[])
    t_next = time.perf_counter()
    t0 = t_next
    for _ in range(args.count):
        bus.send(sync)
        t_next += period
        delay = t_next - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
    elapsed = time.perf_counter() - t0
    bus.shutdown()

    time.sleep(0.5)                      # let the last replies land
    dump.terminate()
    dump.wait()
    log.close()

    after = counters(args.iface)
    print('after: ', after)

    syncs = 0
    tpdo = collections.Counter()          # (cob_base, node) -> frames
    bootups = []
    error_frames = []
    other = collections.Counter()
    first_ts = None
    for line in open(log.name):
        m = re.search(r'\((\d+\.\d+)\)\s+\S+\s+([0-9A-F]{3,8})\s+\[(\d+)\]\s*(.*)', line)
        if not m:
            continue
        ts, cob, data = float(m.group(1)), m.group(2), m.group(4).strip()
        if first_ts is None:
            first_ts = ts
        if len(cob) == 8:                # error frame (candump -e)
            error_frames.append(('%.3f' % (ts - first_ts), cob, data))
            continue
        cob = int(cob, 16)
        if cob == 0x080:
            syncs += 1
        elif 0x181 <= cob <= 0x1FF or 0x281 <= cob <= 0x2FF or 0x381 <= cob <= 0x3FF or 0x481 <= cob <= 0x4FF:
            tpdo[(cob & 0x780, cob & 0x7F)] += 1
        elif 0x701 <= cob <= 0x77E:
            bootups.append(('%.3f' % (ts - first_ts), cob - 0x700, data))
        else:
            other[cob] += 1

    print('SYNCs sent %d in %.2fs (%.0f Hz asked); SYNCs seen on the wire: %d'
          % (args.count, elapsed, args.rate, syncs))
    for base, name in ((0x180, 'TPDO1'), (0x280, 'TPDO2'), (0x380, 'TPDO3'), (0x480, 'TPDO4')):
        row = [tpdo[(base, n)] for n in nodes]
        if any(row):
            print('  %s frames per puck %s: %s' % (name, nodes, row))
    missing = {n: syncs - tpdo[(0x180, n)] for n in nodes if tpdo[(0x180, n)] < syncs}
    if missing:
        print('  TPDO1 frames short of one per SYNC:', missing)
    else:
        print('  every puck answered every SYNC with TPDO1')
    if bootups:
        print('  BOOT-UP/heartbeat-state frames from pucks (t, node, state):', bootups[:16],
              '... total %d' % len(bootups))
        if any(d.strip() == '00' for _, _, d in bootups):
            print('  >>> at least one puck REBOOTED during the run (boot-up state 00)')
    if error_frames:
        print('  CAN error frames (t, id, data):', error_frames[:8], '... total %d' % len(error_frames))
    if other:
        print('  other frames:', {hex(k): v for k, v in sorted(other.items())})
    for key in ('error_warning', 'error_passive', 'bus_off', 'rx_dropped', 'bus_errors'):
        if after[key] != before[key]:
            print('  host CAN %s changed: %d -> %d' % (key, before[key], after[key]))
    print('host CAN state now:', after['state'])
    print('candump log:', log.name)


if __name__ == '__main__':
    main()
