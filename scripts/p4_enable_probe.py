#!/usr/bin/env python3
"""Enable ONE Puck 4 by SDO alone, then optionally replay, one at a time, the
pieces of the normal host/safety-board traffic, so the drive can be felt in
isolation.  Written to the requirements in doc/p4-cogging-review.md.

Preconditions: safety board in Shift-Idle (bus powered), NO libbarrett program
running, the arm supported so the joint under test can be moved by hand.

  p4_enable_probe.py 4                          enabled, CST, zero torque, nothing else on the bus
  p4_enable_probe.py 4 --keepalive cwmode       + controlword AND mode-of-operation rewritten every 50 ms
  p4_enable_probe.py 4 --keepalive cw           + controlword only, every 50 ms (RPDO2 remapped to CW only)
  p4_enable_probe.py 4 --keepalive cwmode --sync   + the host's 2 ms SYNC and zero torque quads
  p4_enable_probe.py 4 --pi 0.5                 current-loop Kp/Ki x0.5 for this run (RAM only)
  p4_enable_probe.py 4 --velfilter 5            drive velocity filter 5 Hz for this run (RAM only)
  p4_enable_probe.py 4 --sync --bias-beta -6    Beta current-sense bias shifted by -6 counts for this run (RAM only)
  p4_enable_probe.py 4 --sync --also 1 2 3      the other three drives enabled too (CST, zero), as under the host

Position TPDOs are only transmitted on SYNC, so runs that will be analysed
with p4_sweep_fold.py from a candump need --sync.

What it guarantees (and checks):
  * the mode is written AFTER the drive confirms Operation Enabled, and 0x6061
    is read back = 10 (fw 4.3.2 only engages a mode written while enabled);
  * 0x6071 (target torque) and 0x60B2 (torque offset) are written 0 and read back;
  * the keepalive goes to a node-private COB-ID (0x300 + node), so the other
    drives never see it; their statuswords are logged at start and end;
  * RPDO2 mapping, Kp/Ki and the velocity filter are captured first and
    restored on exit, in RAM only (nothing is saved to EEPROM);
  * the transmitters start only after the enable + mode are confirmed;
  * statusword, mode, target, offset, position and electrical angle are
    logged to ~/sbflash/probe-<node>-<time>.csv at ~20 Hz for the whole run.
Enter (or Ctrl-C) ends the run; the drive is disabled before anything is restored.
"""
import argparse
import os
import select
import struct
import sys
import threading
import time

import can
import canopen

CW_FAULT_RESET, CW_SHUTDOWN, CW_ENABLE, CW_DISABLE = 0x80, 0x06, 0x0F, 0x00
MODE_CST = 10
SW_MASK = 0x6F


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("node", type=int, choices=range(1, 9))
    ap.add_argument("--can", default="can20")
    ap.add_argument("--keepalive", choices=["cwmode", "cw"], help="replay the safety board's 50 ms refresh, node-private COB")
    ap.add_argument("--keepalive-ms", type=float, default=50.0)
    ap.add_argument("--sync", action="store_true", help="also send SYNC + a zero torque quad every 2 ms, like the host")
    ap.add_argument("--pi", type=float, help="scale the current-loop Kp and Ki (0x2380:1/2) for this run, RAM only")
    ap.add_argument("--velfilter", type=int, help="drive velocity filter cutoff Hz (0x2100:1) for this run, RAM only")
    ap.add_argument("--bias-alpha", type=int, default=0, help="ADD this many ADC counts to the Alpha current-sense bias (0x3008:3) for this run, RAM only")
    ap.add_argument("--bias-beta", type=int, default=0, help="ADD this many ADC counts to the Beta current-sense bias (0x3009:3) for this run, RAM only")
    ap.add_argument("--gain-beta", type=int, default=0, help="ADD this to the Beta current-sense gain factor (0x3009:6, 4096 = 1.0) for this run, RAM only")
    ap.add_argument("--also", type=int, nargs="*", default=[], metavar="NODE",
                    help="also enable these other drives (CST, zero torque, no keepalive) so the arm is in the same electrical state as under the host")
    ap.add_argument("--fastlog", action="store_true",
                    help="log only t,pos,theta_e as fast as SDO allows (~150-200 Hz) so a run WITHOUT --sync can still be folded from its CSV (p4_sweep_fold.py --csv)")
    ap.add_argument("--restlog", action="store_true",
                    help="log t,theta_e,ud,uq,ifb,alpha_filt,beta_filt as fast as SDO allows: for an AT-REST estimate of the "
                         "current-sense offset from the voltage the loop applies to hold zero measured current")
    ap.add_argument("--seconds", type=float, default=0, help="stop automatically after this long (0 = until Enter)")
    args = ap.parse_args()

    net = canopen.Network()
    net.connect(bustype="socketcan", channel=args.can, bitrate=1000000)
    node = canopen.RemoteNode(args.node, canopen.ObjectDictionary())
    net.add_node(node)
    others = {}
    for n in range(1, 5):
        if n != args.node:
            others[n] = canopen.RemoteNode(n, canopen.ObjectDictionary())
            net.add_node(others[n])

    def up(nd, idx, sub, fmt):
        b = nd.sdo.upload(idx, sub)
        return struct.unpack(fmt, b[:struct.calcsize(fmt)].ljust(struct.calcsize(fmt), b"\0"))[0]

    def down(nd, idx, sub, fmt, value):
        nd.sdo.download(idx, sub, struct.pack(fmt, value))

    def sw(nd=node):
        return up(nd, 0x6041, 0, "<H")

    def wait_sw(want, timeout=1.5):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = sw()
            if (s & SW_MASK) == want:
                return s
            time.sleep(0.02)
        raise RuntimeError("statusword 0x%04x, wanted (sw & 0x6F) == 0x%02x" % (sw(), want))

    stop = threading.Event()
    restore = []          # (description, callable) in the order to run at exit
    threads = []
    log = None

    def snapshot(label):
        row = dict(t=time.time(), sw=sw(), mode=up(node, 0x6061, 0, "<b"),
                   target=up(node, 0x6071, 0, "<h"), offset=up(node, 0x60B2, 0, "<h"),
                   pos=up(node, 0x6064, 0, "<i"), theta_e=up(node, 0x60EA, 0, "<h"),
                   ifb=up(node, 0x6078, 0, "<h"))
        if log is not None:
            log.write("%.4f,%s,0x%04x,%d,%d,%d,%d,%d,%d\n" % (
                row["t"], label, row["sw"], row["mode"], row["target"], row["offset"],
                row["pos"], row["theta_e"], row["ifb"]))
        return row

    def others_state(when):
        for n, nd in sorted(others.items()):
            try:
                print("  other node %d %s: sw=0x%04x mode=%d" % (n, when, sw(nd), up(nd, 0x6061, 0, "<b")))
            except Exception as e:
                print("  other node %d %s: read failed (%s)" % (n, when, e))

    try:
        # --- record the state we start from -------------------------------------
        print("node %d start: sw=0x%04x mode=%d target=%d offset=%d" % (
            args.node, sw(), up(node, 0x6061, 0, "<b"), up(node, 0x6071, 0, "<h"), up(node, 0x60B2, 0, "<h")))
        others_state("before")

        orig_cob = up(node, 0x1401, 1, "<I")
        orig_type = up(node, 0x1401, 2, "<B")
        orig_count = up(node, 0x1601, 0, "<B")
        orig_map = [up(node, 0x1601, i, "<I") for i in range(1, orig_count + 1)]
        print("RPDO2 as found: cob=0x%08x type=%d map=%s" % (orig_cob, orig_type, ["0x%08x" % m for m in orig_map]))

        def set_rpdo2(cob, rtype, mapping):
            down(node, 0x1401, 1, "<I", 0x80000000 | (cob & 0x7FF))
            down(node, 0x1601, 0, "<B", 0)
            for i, m in enumerate(mapping, 1):
                down(node, 0x1601, i, "<I", m)
            down(node, 0x1601, 0, "<B", len(mapping))
            down(node, 0x1401, 2, "<B", rtype)
            down(node, 0x1401, 1, "<I", cob & 0x7FF)
            got = (up(node, 0x1401, 1, "<I"), up(node, 0x1401, 2, "<B"),
                   [up(node, 0x1601, i, "<I") for i in range(1, up(node, 0x1601, 0, "<B") + 1)])
            if got != (cob & 0x7FF, rtype, list(mapping)):
                raise RuntimeError("RPDO2 readback mismatch: %r" % (got,))

        # --- optional RAM-only tuning changes (captured first) --------------------
        if args.pi is not None:
            kp_raw = node.sdo.upload(0x2380, 1)[:4]
            ki_raw = node.sdo.upload(0x2380, 2)[:4]
            kp, ki = struct.unpack("<f", kp_raw)[0], struct.unpack("<f", ki_raw)[0]
            restore.append(("current-loop Kp/Ki", lambda: (node.sdo.download(0x2380, 1, kp_raw), node.sdo.download(0x2380, 2, ki_raw))))
            node.sdo.download(0x2380, 1, struct.pack("<f", kp * args.pi))
            node.sdo.download(0x2380, 2, struct.pack("<f", ki * args.pi))
            print("Kp %.4f -> %.4f, Ki %.1f -> %.1f (RAM)" % (kp, struct.unpack("<f", node.sdo.upload(0x2380, 1)[:4])[0],
                                                             ki, struct.unpack("<f", node.sdo.upload(0x2380, 2)[:4])[0]))
        if args.velfilter is not None:
            vf = up(node, 0x2100, 1, "<H")
            restore.append(("velocity filter", lambda: down(node, 0x2100, 1, "<H", vf)))
            down(node, 0x2100, 1, "<H", args.velfilter)
            print("velocity filter %d -> %d Hz (RAM)" % (vf, up(node, 0x2100, 1, "<H")))

        for idx, delta, name in ((0x3008, args.bias_alpha, "Alpha"), (0x3009, args.bias_beta, "Beta")):
            if delta:
                orig = up(node, idx, 3, "<H")
                restore.append(("%s bias" % name, lambda idx=idx, orig=orig: down(node, idx, 3, "<H", orig)))
                down(node, idx, 3, "<H", orig + delta)
                print("%s bias %d -> %d counts (RAM)" % (name, orig, up(node, idx, 3, "<H")))
        if args.gain_beta:
            orig_g = up(node, 0x3009, 6, "<H")
            restore.append(("Beta gain factor", lambda: down(node, 0x3009, 6, "<H", orig_g)))
            down(node, 0x3009, 6, "<H", orig_g + args.gain_beta)
            print("Beta gain factor %d -> %d (RAM)" % (orig_g, up(node, 0x3009, 6, "<H")))
        print("bias now: Alpha %d Beta %d; filtered at rest: Alpha %d Beta %d" % (
            up(node, 0x3008, 3, "<H"), up(node, 0x3009, 3, "<H"),
            (up(node, 0x3008, 2, "<h") >> 4), (up(node, 0x3009, 2, "<h") >> 4)))

        # --- keepalive mapping on a node-private COB ------------------------------
        ka_cob = 0x300 + args.node
        if args.keepalive:
            mapping = [0x60400010, 0x60600008] if args.keepalive == "cwmode" else [0x60400010]
            restore.append(("RPDO2 mapping", lambda: set_rpdo2(orig_cob & 0x7FF, orig_type, orig_map)))
            set_rpdo2(ka_cob, 255, mapping)
            print("RPDO2 -> cob=0x%03x type=255 map=%s (verified)" % (ka_cob, ["0x%08x" % m for m in mapping]))

        # TPDOs (position on SYNC, statusword) are only produced in NMT Operational;
        # start this node only (a plain NMT start, no reset, so RAM settings survive).
        net.send_message(0x000, bytes([0x01, args.node]))
        time.sleep(0.05)

        # --- enable, then mode, then zero targets, all confirmed ------------------
        path = os.path.expanduser("~/sbflash/probe-%d-%s.csv" % (args.node, time.strftime("%Y%m%d-%H%M%S")))
        log = open(path, "w")
        log.write("t,label,sw,mode,target,offset,pos,theta_e,ifb,ud,uq,ifb2,alpha_filt,beta_filt\n" if args.restlog else "t,label,sw,mode,target,offset,pos,theta_e,ifb\n")
        snapshot("start")
        down(node, 0x6071, 0, "<h", 0)
        down(node, 0x60B2, 0, "<h", 0)
        down(node, 0x6040, 0, "<H", CW_FAULT_RESET); time.sleep(0.15)
        down(node, 0x6040, 0, "<H", CW_SHUTDOWN); wait_sw(0x21)
        down(node, 0x6040, 0, "<H", CW_ENABLE); wait_sw(0x27)
        restore.insert(0, ("drive disabled", lambda: (down(node, 0x6060, 0, "<b", 0), down(node, 0x6040, 0, "<H", CW_DISABLE))))
        down(node, 0x6060, 0, "<b", MODE_CST)
        t0 = time.time()
        while up(node, 0x6061, 0, "<b") != MODE_CST:
            if time.time() - t0 > 1.5:
                raise RuntimeError("mode display did not reach 10 (CST) after enable")
            time.sleep(0.02)
        down(node, 0x6071, 0, "<h", 0)
        down(node, 0x60B2, 0, "<h", 0)
        row = snapshot("enabled")
        if row["target"] != 0 or row["offset"] != 0 or (row["sw"] & SW_MASK) != 0x27 or row["mode"] != MODE_CST:
            raise RuntimeError("unexpected state after enable: %r" % row)
        print("node %d ENABLED in CST, target 0, offset 0: sw=0x%04x mode=%d" % (args.node, row["sw"], row["mode"]))

        # --- other drives, same sequence, no keepalive ----------------------------
        for n in args.also:
            nd = others.get(n) or canopen.RemoteNode(n, canopen.ObjectDictionary())
            if n not in others:
                net.add_node(nd); others[n] = nd
            net.send_message(0x000, bytes([0x01, n])); time.sleep(0.05)
            down(nd, 0x6071, 0, "<h", 0); down(nd, 0x60B2, 0, "<h", 0)
            down(nd, 0x6040, 0, "<H", CW_FAULT_RESET); time.sleep(0.15)
            down(nd, 0x6040, 0, "<H", CW_SHUTDOWN); time.sleep(0.3)
            down(nd, 0x6040, 0, "<H", CW_ENABLE)
            t1 = time.time()
            while (sw(nd) & SW_MASK) != 0x27:
                if time.time() - t1 > 1.5: raise RuntimeError("node %d did not enable" % n)
                time.sleep(0.02)
            restore.insert(0, ("node %d disabled" % n, lambda nd=nd: (down(nd, 0x6060, 0, "<b", 0), down(nd, 0x6040, 0, "<H", CW_DISABLE))))
            down(nd, 0x6060, 0, "<b", MODE_CST)
            t1 = time.time()
            while up(nd, 0x6061, 0, "<b") != MODE_CST:
                if time.time() - t1 > 1.5: raise RuntimeError("node %d mode did not reach CST" % n)
                time.sleep(0.02)
            down(nd, 0x6071, 0, "<h", 0)
            print("node %d also ENABLED in CST, target 0: sw=0x%04x" % (n, sw(nd)))

        # --- transmitters, only now ----------------------------------------------
        if args.keepalive:
            data = bytes([CW_ENABLE, 0x00, MODE_CST]) if args.keepalive == "cwmode" else bytes([CW_ENABLE, 0x00])
            msg = can.Message(arbitration_id=ka_cob, data=data, is_extended_id=False)
            def keepalive():
                while not stop.is_set():
                    net.bus.send(msg); time.sleep(args.keepalive_ms / 1000.0)
            threads.append(threading.Thread(target=keepalive, daemon=True)); threads[-1].start()
            print("keepalive %s: 0x%03x %s every %.0f ms" % (args.keepalive, ka_cob, data.hex(" "), args.keepalive_ms))
        if args.sync:
            sync = can.Message(arbitration_id=0x080, data=b"", is_extended_id=False)
            quad = can.Message(arbitration_id=0x201, data=bytes(8), is_extended_id=False)
            def cyclic():
                nxt = time.perf_counter()
                while not stop.is_set():
                    net.bus.send(sync); net.bus.send(quad)
                    nxt += 0.002
                    d = nxt - time.perf_counter()
                    if d > 0: time.sleep(d)
            threads.append(threading.Thread(target=cyclic, daemon=True)); threads[-1].start()
            print("SYNC + zero torque quad every 2 ms")

        # --- run: log state, watch for drop-outs, wait for Enter ------------------
        print("Move the joint by hand.  Enter to stop.  Log: %s" % path)
        started = time.time(); last_sw = row["sw"]; last_mode = row["mode"]; k = 0
        dropped = False
        while not stop.is_set():
            if select.select([sys.stdin], [], [], 0.0 if args.fastlog else 0.05)[0]:
                sys.stdin.readline(); break
            if args.seconds and time.time() - started > args.seconds:
                break
            if args.restlog:
                k += 1
                t = time.time(); th = up(node, 0x60EA, 0, "<h"); ud = up(node, 0x3010, 4, "<h"); uq = up(node, 0x3010, 5, "<h")
                ifb = up(node, 0x6078, 0, "<h"); fa = up(node, 0x3008, 2, "<h"); fb = up(node, 0x3009, 2, "<h")
                if k % 30 == 0:
                    r = snapshot("run")
                else:
                    log.write("%.4f,rest,,,,,,%d,%d,%d,%d,%d,%d\n" % (t, th, ud, uq, ifb, fa, fb)); continue
            elif args.fastlog:
                k += 1
                t = time.time(); pos = up(node, 0x6064, 0, "<i"); th = up(node, 0x60EA, 0, "<h")
                if k % 50 == 0:
                    r = snapshot("run")
                else:
                    log.write("%.4f,fast,,,,,%d,%d,\n" % (t, pos, th)); continue
            else:
                r = snapshot("run")
            if r["sw"] != last_sw or r["mode"] != last_mode:
                print("  t=%5.1fs statusword 0x%04x mode %d%s" % (time.time() - started, r["sw"], r["mode"],
                      "  (DROPPED out of Operation Enabled)" if (r["sw"] & SW_MASK) != 0x27 else ""))
                last_sw, last_mode = r["sw"], r["mode"]
            if (r["sw"] & SW_MASK) != 0x27 or r["mode"] != MODE_CST:
                dropped = True
                print("\n*** The drive is NOT enabled in CST any more (sw=0x%04x mode=%d). This run is INVALID." % (r["sw"], r["mode"]))
                print("*** fw 4.3.2 drops an enabled drive after ~55 SYNC cycles without a controlword refresh:")
                print("*** with --sync you must also pass --keepalive cwmode|cw.  Stopping.\n")
                break
            if r["target"] != 0 or r["offset"] != 0:
                print("  t=%5.1fs target %d offset %d are not zero!" % (time.time() - started, r["target"], r["offset"]))
        if not dropped:
            print("run ended with the drive still enabled in CST (%.0f s)" % (time.time() - started))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for th in threads:
            th.join(timeout=0.5)
        for what, fn in restore:
            try:
                fn(); print("restored: %s" % what)
            except Exception as e:
                print("RESTORE FAILED (%s): %s" % (what, e))
        try:
            if log is not None:
                snapshot("end"); log.close()
            print("node %d end: sw=0x%04x mode=%d" % (args.node, sw(), up(node, 0x6061, 0, "<b")))
            others_state("after")
        except Exception as e:
            print("final read failed:", e)
        net.disconnect()


if __name__ == "__main__":
    main()
