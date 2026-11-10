#!/usr/bin/env python3
"""Watch joint directions with the drives DISABLED: read every puck's position
over SDO, map motor counts to joint angles with the j2mp / sign arrays of a
wam8g.conf, and print the change since start.  Nothing is enabled or
commanded; the safety board only needs to be in Shift-Idle (bus powered) and
no host program may be running.

    p4_wrist_watch.py --conf ~/sbflash/wam8g.conf [--can can20] [--nodes 5 6 7 8]

Move ONE joint at a time by hand and check:
  * which motors move and with which relative signs (the transmission), and
  * whether the resulting joint angle grows in the direction your DH table
    calls positive (right-hand rule about that joint's z axis).
Fix a wrong joint sign with canopen_position_sign (and the matching
canopen_torque_sign) for that MOTOR; fix a wrong sum/difference assignment in
j2mp.  Ctrl-C to stop.
"""
import argparse
import math
import os
import re
import sys
import time

import canopen


def parse_matrix(text, name, n=8):
    m = re.search(r"\b%s\s*=\s*\(\s*(\(.*?\))\s*\)\s*;" % re.escape(name), text, re.S)
    if not m:
        return None
    rows = re.findall(r"\(([^()]*)\)", m.group(1))
    mat = [[float(x) for x in row.replace("\n", " ").split(",")] for row in rows]
    if len(mat) != n or any(len(r) != n for r in mat):
        raise SystemExit("%s: expected %dx%d, got %s" % (name, n, n, [len(r) for r in mat]))
    return mat


def parse_vector(text, name, n=8):
    # only an UNcommented setting counts
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("#"):
            continue
        m = re.match(r"%s\s*=\s*\(([^)]*)\)\s*;" % re.escape(name), s)
        if m:
            v = [float(x) for x in m.group(1).split(",")]
            if len(v) != n:
                raise SystemExit("%s: expected %d entries" % (name, n))
            return v
    return [1.0] * n


def invert(a):
    n = len(a)
    m = [row[:] + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise SystemExit("j2mp is singular")
        m[c], m[p] = m[p], m[c]
        f = m[c][c]; m[c] = [x / f for x in m[c]]
        for r in range(n):
            if r != c and m[r][c] != 0.0:
                g = m[r][c]; m[r] = [x - g * y for x, y in zip(m[r], m[c])]
    return [row[n:] for row in m]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conf", default=os.path.expanduser("~/sbflash/wam8g.conf"))
    ap.add_argument("--can", default="can20")
    ap.add_argument("--nodes", type=int, nargs="*", default=[1, 2, 3, 4, 5, 6, 7, 8])
    ap.add_argument("--rate", type=float, default=4.0, help="lines per second")
    args = ap.parse_args()

    text = open(args.conf).read()
    j2mp = parse_matrix(text, "j2mp")
    if j2mp is None:
        raise SystemExit("no j2mp in %s" % args.conf)
    m2jp = invert(j2mp)
    psign = parse_vector(text, "canopen_position_sign")
    print("j2mp read from %s; position signs %s" % (args.conf, ["%+.0f" % s for s in psign]))
    for i in range(8):
        drives = ["m%d" % (r + 1) for r in range(8) if abs(j2mp[r][i]) > 1e-9]
        print("  J%d moves %s" % (i + 1, ", ".join("%s(%+.3f)" % (d, j2mp[int(d[1:]) - 1][i]) for d in drives)))

    net = canopen.Network()
    net.connect(bustype="socketcan", channel=args.can, bitrate=1000000)
    nodes = {}
    cpr = {}
    for n in args.nodes:
        nd = canopen.RemoteNode(n, canopen.ObjectDictionary()); net.add_node(nd); nodes[n] = nd
        try:
            cpr[n] = int.from_bytes(nd.sdo.upload(0x3013, 1), "little") or 4096
        except Exception as e:
            print("node %d: no answer (%s) - treated as stationary" % (n, e)); nodes.pop(n)

    def read_counts():
        c = [0] * 8
        for n, nd in nodes.items():
            try:
                c[n - 1] = int.from_bytes(nd.sdo.upload(0x6064, 0)[:4], "little", signed=True)
            except Exception:
                pass
        return c

    start = read_counts()
    print("counts/rev: %s" % {n: cpr[n] for n in sorted(cpr)})
    print("Move one joint by hand.  Columns: motor turns since start (m1..m8), then joint change since start J1..J8 in degrees.")
    print("%-6s %s   %s" % ("t", " ".join("%7s" % ("m%d" % (i + 1)) for i in range(8)), " ".join("%7s" % ("J%d" % (i + 1)) for i in range(8))))
    t0 = time.time()
    try:
        while True:
            c = read_counts()
            mrad = [psign[i] * (c[i] - start[i]) * 2 * math.pi / cpr.get(i + 1, 4096) for i in range(8)]
            jrad = [sum(m2jp[i][k] * mrad[k] for k in range(8)) for i in range(8)]
            print("%5.1fs %s   %s" % (time.time() - t0,
                  " ".join("%+7.3f" % (m / (2 * math.pi)) for m in mrad),
                  " ".join("%+7.2f" % math.degrees(j) for j in jrad)))
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        net.disconnect()


if __name__ == "__main__":
    main()
