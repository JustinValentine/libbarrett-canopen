# Cogging in slow free motion on the Puck 4 drives (odin WAM) — open investigation

Status: **unresolved**. This is a record of what we have observed, tried and
measured so far, with the data locations, so that someone else can check the
reasoning and look for what we missed. The hypotheses at the end are ordered by
our current suspicion, not by evidence strength; several are untested and it is
entirely possible the real cause is not on the list.

Bench: 2026-09-16. Desk review: 2026-09-17.

Follow-up: [source and test-method review](p4-cogging-review.md) identifies
limitations in the isolation probe and current-feedback interpretation, and
adds an offline CAN-capture check. The cause is still unconfirmed.

## Setup

- Barrett 4-DOF WAM on the bench PC `odin` (Ubuntu 20.04, SocketCAN `can20`).
- Pucks: P4-37, firmware 4.3.2 (`Puck4—mk1-v4.3.2.ebin`, the only image on
  Barrett's firmware page), nodes 1–4. Configured from Barrett's
  `P437-4dof-4.3.2.csv`, with these changes made by us during bring-up:
  nominal bus voltage 24.0 → 48.0 V (`0x3001:9`), over-voltage trip 52 → 58 V
  (`0x2384:6`), commutation zero / current-sense bias / gain factor
  recalibrated (`0x3011:1/2`, `0x3008/0x3009:3` and `:6`).
- Safety board: our CANopen port of the puck2 firmware, rev 208, node 10.
  Shunt thresholds corrected to 55/60 V, voltage scale `X1` 848 → 869.
- Host: `libbarrett-canopen` (this repo), 500 Hz, CiA 402 Cyclic Synchronous
  Torque, torque RPDO type 0 (apply at SYNC), one SYNC per cycle from the host.
- Host config on odin: `~/lbc-config-wam4/` (torque and position signs −1 on
  all four motors, measured after commutation calibration; position gains at
  half stock, J4 kd stock).

The same arm, with its P3 pucks and the stock safety board, is reported by the
user to be smooth in slow free motion.

## Symptom

Moving a joint slowly by hand while the arm is enabled gives a periodic
"cogging" feel. Reported ordering: J4 strongest, J2/J3 clear, J1 faint. It is
felt in gravity compensation (ex04) and equally in ex01, where the host commands
exactly zero torque every cycle. The only metric so far is the user's hand;
we have no direct torque or acceleration measurement of the effect itself.

Two hand tests that we consider the most informative:

| State | Feel |
|---|---|
| Safety board in Shift-Idle, drives disabled, bus powered | smooth, apart from a faint high-frequency cogging (presumably the motor's own) |
| ex01: drives enabled, host commanding zero torque | the coarse cogging is present |

Our reading: whatever produces it is generated while the drive is enabled and
does not depend on the commanded value. That reading assumes ex01 really does
put the drive into the same state as ex04 apart from the torque value; see
"things we have not checked".

## What was tried, in order

All paths are on odin unless noted. `~/sbflash/capture-run2.log` is a
`candump -t a can20` capture that ran through the whole bench day; every frame
referenced below is in it (timestamps are Unix seconds).

1. **Commutation-zero calibration** (PuckUtility's encoder-zero routine, run
   headless with `~/puckutility/cal_headless.py`, 0.5 A then repeated at
   1.5 A). Stored zeros 428/144/672/521 → 314/421/399/44 counts; the old values
   were 56/150/134/115 electrical degrees off, which had reversed and weakened
   torque on J2–J4. Logs: `~/sbflash/enczero-20260916-1639.log`,
   `enczero-20260916-1648.log`. Torque signs re-measured afterwards with
   `~/sbflash/lbc_torque_sign` (source `tests/lbc_torque_sign.cpp`): −1 on all
   four. The cogging was reported *after* this, with gravity compensation
   otherwise working. Note the second run at 1.5 A moved the Beta gain factor
   on every puck by about 10% relative to the 0.5 A run (e.g. puck 4: 4490
   original → 4578 → 3943), so that calibration method is current-dependent
   and its accuracy is unknown.

2. **Torque-command ripple against electrical angle.** From the capture of a
   gravity-calibration run (window 1789603366–1789603474), the per-mille torque
   commands sent to each puck were binned by that motor's electrical angle
   (`~/sbflash/ripple.py LOG T0 T1`). Puck 4 showed a once-per-electrical-cycle
   pattern of 4.7 per-mille peak-to-peak (≈0.54 Nm at the joint), pucks 2/3
   ≈2.1 per-mille (≈0.4 Nm), 2× harmonic small, 6× ≈ 0. Caveats: this is what
   the *position loop* added, so it only shows disturbances slow enough for the
   loop to fight (a few Hz); anything faster is invisible to this method.

3. **Current-sense offsets.** At rest, filtered current minus stored bias was
   +3 counts on puck 4 (Beta), 1–2 counts on pucks 2/3, ≤1 on puck 1
   (≈13.8 mA per count). A 3-count offset predicts roughly the puck 4 ripple in
   step 2. The bias was re-zeroed at operating temperature
   (`cal_headless.py --ibias-only`). User: "changed almost nothing".

4. **Finer torque resolution.** One per-mille of the 6310 mNm rated torque is
   0.11 Nm at J4, 0.18 at J2/J3, 0.27 at J1; free-motion commands on J4 only
   spanned −11..+8 per-mille. `0x6076` was set to 1000 mNm (libbarrett derives
   its scale from it) with `0x6087` and the safety board's TL1/TL2 rescaled
   (`~/sbflash/set_torque_scale.sh fine|revert`). User: "maybe a small
   difference, not super noticeable"; no difference between J4 horizontal and
   vertical. Reverted.

5. **Drive-reported tracking while J4 is moved by hand in ex04.** SDO poll at
   114 Hz (`~/sbflash/iq_probe.py`, analysis `iq_analyze.py`), file
   `~/sbflash/iqprobe-4-20260916-183123.csv` (t, pos, theta_e, 0x6071 target,
   0x6074 trq_cmd, 0x6078 current feedback, 0x3010:6 id). `ifb − trq_cmd` was
   within −2..+1 per-mille on every sample, no pattern against electrical
   angle; commands stepped about twice a second. Caveat: 0x6078 is in
   per-mille of *peak* current (19.5 mA/LSB), and 114 Hz cannot see
   millisecond-scale dips.

6. **Applied voltages at zero command (ex01).** `~/sbflash/v_probe.py`,
   analysis `v_analyze.py`, file `~/sbflash/vprobe-4-20260916-185403.csv`
   (t, pos, theta_e, 0x3010:4 ud, 0x3010:5 uq, ifb, 0x3008:5, 0x3009:5,
   trq_cmd). No six-per-cycle pattern in ud/uq (the signature we expected from
   dead-time distortion). Instead ud/uq jump sample-to-sample by roughly
   ±0.3–1 V-equivalent (rms 224 / 919 raw units; we assumed ≈1.5 mV per unit
   from a 32768 = bus-voltage scale, which is unverified). Note: the "alpha"
   and "beta" columns in that file are sub-index 5 (Shunt, constant 15), not
   the Filtered currents; that was a mistake in the probe.

7. **Noise at rest, drives disabled** (`~/puckutility/cl_read.py`,
   `fault_read.py`, ad-hoc reads). Raw current-sense noise 2.1–3.1 counts rms,
   14-count peak swings (≈30–40 mA rms, 100 mA peaks); the Filtered channel is
   about 7× quieter. Encoder position dithers by 4–5 counts at rest on all four
   pucks; the drive's velocity estimate (`0x606C`, filter `0x2100:1` = 50 Hz)
   is 70–80 counts/s rms with the motor stationary.

8. **Current-loop gain factor** `0x3024:2` 93 → 50 (`~/sbflash/set_cc_gain.sh`).
   User: "very similar, if not a little worse". However `0x2380:1/2` (the
   float Kp/Ki) did not change when the factor was written, so this test
   probably changed nothing. `~/sbflash/set_cc_pi.sh NODE half|double|restore`
   writes the floats directly and has not been run yet.

9. **Bus timing and host/safety-board traffic.** `~/sbflash/pdo_timing.py`:
   over 40,000 cycles every puck's position TPDO arrived 1.26 ms after SYNC
   (p50), max 1.57 ms, zero missing or duplicated. In ex01 libbarrett sends
   exact zeros in every torque frame (verified in code: `Summer(true)`,
   no dither). While ACTIVE the safety board sends nothing to the drives except
   one keepalive frame every 50 ms (`0x28A`, bytes `0F 00 0A` = controlword
   0x000F + mode 10), see below.

## Facts from documents and code (desk review, 2026-09-17)

Sources: the P2/P3 firmware in `puck2/source` (MOTOR.C, SV.C — note this may
not be the exact P3 build that shipped on this arm), Barrett's P4 CANopen
manual RevBA (`dynamometer/P4_CANopen_Manual_RevBA.pdf`, matches 4.3.1), the
WAM puck config readback
(`puck2/build/sb-bench-2026-09-15/pucks/puck1-odin-wam4-2026-09-16.csv`), and
https://barrett.com/puck-firmware.

| Quantity | P3 (from puck2 source / P3-35 CSV) | P4-37 on this WAM |
|---|---|---|
| Current-sense resolution | ≈4.9 mA/count (12-bit, ±10 A) | ≈13.8 mA/count (`0x3001:8` = 28284 mA span) |
| Sense chain gain | ≈150 mV/A | ≈30–53 mV/A (alpha/beta asymmetric) |
| Current feedback filtering | none (IIR commented out) | 4.3.2: "biquad low-pass from winding L/R"; which sample the loop uses is undocumented |
| Torque command LSB | ≈1 mA (0.37 mNm) | 13.8 mA (6.31 mNm) |
| PWM / current-loop rate | 25 kHz / 5 kHz | 32 kHz / 8 kHz (rate = PWM/4) |
| Dead time | 1.2 µs, no compensation | 250 ns |
| Commutation | software SVM, d/q PI | space-vector FOC, d/q PI (`0x2380` Kp 2.086, Ki 3133) |
| Back-EMF / velocity feedforward | none | none documented |
| Low-torque dithering | none | 4.3.2 changelog: "Added hardware dithering to eliminate audible noise and torque ripple caused by quantization at low torque commands" |
| Encoder | 12-bit magnetic, raw | 12-bit MagAlpha; filter object `0x3013:6` left unset in the WAM config |

Things our own stack does that the P3-era stack did not:

- The safety board re-sends controlword + mode-of-operation (`0x6060` = 10) to
  every puck every 50 ms while ACTIVE. We added this because 4.3.2 silently
  drops an Operation-Enabled drive after ~55 SYNC cycles without a controlword
  RPDO refresh (bench finding, not in the manual). The RPDO2 mapping that
  libbarrett installs is `0x6040` + `0x6060`, transmission type 255, so each
  refresh writes the mode byte into the drive asynchronously to SYNC.
- The host's 500 Hz SYNC with torque latched at SYNC and "ramped to target by
  the next SYNC" (manual, CST description).
- libbarrett's real-time loop runs as an ordinary SCHED_OTHER thread with no
  memory locking (the Xenomai path is never built), and non-real-time threads
  poll the safety board over SDO under the same non-priority-inheriting mutex.
  We have not measured the resulting SYNC jitter.
- libbarrett floored the per-mille torque value instead of rounding it (fixed
  2026-09-17). Irrelevant at exactly zero, a −0.5 LSB bias otherwise.

## Hypotheses, in our current order of suspicion

None of these is established. Each has a test that would move it up or down.

1. **Firmware 4.3.2's low-torque dithering.** A deliberately injected signal,
   active at low commands, new in the version on these pucks; nothing in the
   object dictionary exposes it. Test: ask Barrett whether it is switchable;
   A/B one puck against 4.3.1 if an image can be obtained.
2. **The safety board's 20 Hz mode rewrite.** If the drive re-enters CST (or
   resets its torque pipeline) on every `0x6060` write, that is a 20 Hz tick.
   Test: `~/sbflash/enable_probe.py 4` enables the puck by SDO alone with
   nothing else on the bus; `--keepalive` replays the 50 ms frame;
   `--keepalive-cwonly` remaps RPDO2 to the controlword only and replays it
   (this variant is written but not yet copied to odin); `--keepalive --sync`
   adds the host's 2 ms SYNC; `--pva` enables in phase-voltage mode with no
   current loop at all.
3. **Current-sensor noise driven into the winding.** 30–40 mA rms of
   measurement noise would be ≈0.3 Nm rms at J4 if the loop closes on the raw
   sample. Consistent with the voltage jitter in step 6, but we would expect
   it to feel rough rather than like teeth. Test: `set_cc_pi.sh 4 half`;
   ask Barrett which sample the loop uses.
4. **Velocity-estimate noise through some feedforward or filter path.** The
   encoder dithers two counts at rest. The manual documents no EMF
   feedforward, which is why this is low. Test: `set_vel_filter.sh 4 set 5`.
5. **Something in the generic P437 config that is wrong for this hardware.**
   The CSV came with 24 V nominal on a 48 V arm, so other fields may also be
   generic: ADC settling/sampling/conversion times (`0x3001:3,5,6,7`), max
   sensor current, encoder filter, the current-loop gains derived from
   R = 4.50 Ω (the P3-35 file says 3.47 Ω for the same motor). Not tested.

Things the data argue against, with the evidence and its limits:

- *Torque quantization*: 6.3× finer scale made little difference and there was
  no pose dependence. Limit: subjective.
- *Sensor bias*: re-zeroed warm, no change. Limit: subjective; the 1× ripple
  in step 2 has not been re-measured after the re-zero.
- *Commutation zero*: calibrated twice, values agree within 5–18 counts. A
  residual error scales torque rather than adding bumps. Limit: the
  calibration routine's own accuracy under friction is unknown.
- *Inverter dead time*: no 6× pattern in the applied voltages at 85 Hz
  sampling, and the P4 has a fifth of the P3's dead time. Limit: an 85 Hz
  SDO poll averages over any transient shorter than ~10 ms.
- *Communication*: positions on time every cycle, none lost.
- *Host command value*: exact zeros in ex01.

## Things we have not checked, or could be wrong about

- We have no objective measurement of the cogging itself. A cheap one would
  be an encoder-velocity log during a slow constant-velocity move (position
  loop off, e.g. hand-driven with a known speed), or an accelerometer on the
  forearm, so that firmware/config changes can be compared numerically.
- Whether ex01's drive state really equals ex04's apart from the torque
  value (same mode, same PDO configuration, same keepalive).
- Whether 0x6078 and 0x3010:4/5 report the quantities the loop actually
  uses, and at what internal rate; our probes sample at ≤114 Hz.
- The P3 comparison is against the puck2 tree's TATER code, which may differ
  from the P3 firmware that shipped on this arm.
- We have not tried the puck's own velocity/acceleration or i²t settings, the
  `0x2280` commutator flags (use-Kalman etc.), or the MagAlpha filter.
- We have not measured host SYNC jitter, nor tried RPDO transmission type
  255 (apply on receipt) instead of type 0.
- We have not looked at whether the safety board's keepalive could be replaced
  by something the drive accepts without a mode write (e.g. an RPDO or SDO
  watchdog object); the manual lists `0x2406` SYNC-loss timeout but nothing
  about a controlword refresh requirement.
- The P4 gain-factor calibration's 10% current dependence (step 1) has not
  been explained.

## Next bench session (after the 2026-09-17 review)

Tools for this live in `scripts/` of this repo and need copying to odin
(`~/sbflash/`), where `~/puckutility-venv/bin/python` has python-can and
canopen. The earlier `enable_probe.py` on odin is superseded and should not be
used (mode written before enable, broadcast keepalive, no mapping restore).

1. **Angle-locked or time-locked?** No drive changes. Start
   `candump -t a can20 > ~/sbflash/sweep-$(date +%Y%m%d-%H%M).log`, run ex01,
   Shift-Activate, support the arm, and move J4 slowly by hand over the same
   ~30° range: back and forth several times at one slow speed, then several
   times at a clearly faster (still slow) speed. Note the wall-clock times of
   each set. Then:

   ```sh
   python3 scripts/analyze_p4_cogging_capture.py LOG --start T0 --end T1 --node 4   # bus/timing sanity
   python3 scripts/p4_sweep_fold.py LOG --start T0 --end T1 --node 4               # the discrimination
   ```

   `p4_sweep_fold.py` folds the motor's velocity residual by electrical angle,
   mechanical angle and time since the last keepalive, per direction and per
   speed half, and prints the cross-speed agreement of each fold. On a
   synthetic sweep with ±2-count encoder dither it separates the two cases
   cleanly (`tests/test_p4_sweep_fold.py`). Expect at most one fold to hold
   its pattern at both speeds; if none does, the disturbance is not periodic
   in either sense at this resolution.

2. **Isolated-drive A/B** (only if step 1 points at the keepalive, or is
   inconclusive). `scripts/p4_enable_probe.py` enables one puck by SDO, writes
   the mode after the drive confirms Operation Enabled, verifies target torque
   and torque offset are zero, uses a node-private COB for the keepalive,
   restores the RPDO2 mapping and any RAM tuning change on exit, and logs
   statusword/mode/position/electrical angle to a CSV throughout. Board in
   Shift-Idle, no program running, arm supported:

   ```sh
   P=~/puckutility-venv/bin/python
   $P ~/sbflash/p4_enable_probe.py 4                          # CST, zero torque, nothing else
   $P ~/sbflash/p4_enable_probe.py 4 --keepalive cwmode       # + 50 ms controlword + mode rewrite
   $P ~/sbflash/p4_enable_probe.py 4 --keepalive cw           # + 50 ms controlword only
   $P ~/sbflash/p4_enable_probe.py 4 --keepalive cwmode --sync  # + host-style SYNC and zero quads
   $P ~/sbflash/p4_enable_probe.py 4 --pi 0.5                 # current-loop Kp/Ki halved, RAM only
   $P ~/sbflash/p4_enable_probe.py 4 --velfilter 5            # velocity filter 5 Hz, RAM only
   ```

   The script prints the other drives' statuswords before and after so the
   rest of the arm is shown to have stayed disabled. Keep a candump running
   during each so `p4_sweep_fold.py` can be applied to the same motion.

3. **State check in each condition**: the probe log and, for ex01/ex04, an SDO
   read of `0x6041, 0x6060, 0x6061, 0x6071, 0x6074, 0x60B2` (the review's list;
   `~/puckutility/fault_read.py` covers most of it).

4. For the SDO-probe CSVs, `scripts/p4_probe_fold.py FILE.csv` replaces the
   old analyzers: fixed 65536-count electrical-angle scale, harmonics over the
   visited bins only, coverage reported.

## Bench results, 2026-09-17 (odin, J4 by hand)

All runs with the arm supported and J4 moved slowly by hand; the metric is the
motor's velocity residual folded by electrical angle, converted to a
torque-proportional number (1x amplitude x electrical frequency, in
counts/s x counts/s; larger = stronger ripple). Scripts: `p4_sweep_fold.py`,
`bias_solve.py` (odin `~/sbflash/`). Run-to-run scatter for one condition is
about +-30%.

**1. Angle-locked, not time-locked** (`sweep-20260917-1630.log`, ex01, 94k
cycles, every torque frame zero). Fold by time since the safety board's 50 ms
keepalive: modulation 13-47 counts/s, signal/noise 0.6-1.7, agreement between
speeds r = -0.26 (nothing). Fold by electrical angle: 600-1135 counts/s, s/n
22-29, r = 0.87, phase reverses with direction. One cycle per electrical cycle,
a second harmonic about a third as large, nothing at six times.

**2. The isolated drive does it by itself** (`probe2-20260917-1657.log`, puck
4 enabled over SDO by `p4_enable_probe.py`, mode written after enable, target
and torque offset verified zero, keepalive on a node-private COB):

| condition | 1x ripple | 2x ripple |
|---|---|---|
| SYNC + zero quads + controlword-and-mode refresh every 50 ms (host-equivalent) | 419k | 252k |
| same, controlword-only refresh | 271k | 193k |
| no SYNC, no refresh (drive stays enabled indefinitely without SYNC) | 369k | 242k |

The safety board's mode rewrite, the host's SYNC and the torque frames are all
excluded as the cause. (Note: with SYNC running and no controlword refresh the
drive drops out of Operation Enabled within ~0.1 s; five earlier runs made that
mistake and were discarded.)

**3. The ripple follows the current-sense bias** (RAM-only shifts of the
stored bias, 0x3008:3 / 0x3009:3, ~14 mA per count):

| shift | 1x ripple | phase | user's feel |
|---|---|---|---|
| none | 419k | -1 deg | baseline cogging |
| Beta -6 | 891k | 141 deg | much lower-frequency cogging |
| Beta +6 | 2006k | -31 deg | very strong |
| Alpha -6 | 914k | 52 deg | stronger than baseline |
| Alpha +6 | 1386k | -76 deg | strong |
| Beta -1 | 52k | 135 deg | (not felt separately) |
| Beta -2 | 236k | 171 deg | "a lot better, not perfect" |
| Beta -3 | 649k | 146 deg | worse than -2 |

The phase reverses between 0 and -1, so the 1x null on puck 4 lies at a Beta
shift of about -1.2 counts (~17 mA). One count of bias error is worth roughly
200k in this unit, i.e. the baseline cogging corresponds to 1-2 counts.
Two Beta gain-factor shifts of +-150 (3.7%) did not change the 2x component
in a consistent way; the 2x term remains unexplained.

**4. History of puck 4's Beta bias and rest reading** (the value the
calibration routine stores is the filtered channel at zero current, rounded
to an integer count):

| when | stored Beta bias | filtered reading at rest |
|---|---|---|
| as delivered (Barrett CSV) | 2071 | - |
| 2026-09-16 16:39, after encoder-zero cal at 0.5 A | 2071 | 2071 |
| 2026-09-16 16:48, after encoder-zero cal at 1.5 A | 2069 | 2069 |
| 2026-09-16 ~19:00, bias-only re-zero, warm after use | 2072 | 2072 |
| 2026-09-17 17:20, after a power cycle, drives idle, 35-38 C | 2072 | 2070.9 |

Pucks 1-3 today: Alpha within +-0.3 of the stored bias, Beta -1.7 / +0.4 /
0.0. So between measurements hours apart the rest zero of some channels moves
by 1-3 counts. Whether that is temperature (measurements were taken cold, warm,
and right after calibration currents), the power cycle, or something else is
not established; there is no controlled drift measurement yet. What is
established is that the stored integer bias only nulls the ripple to within
+-0.5 count at the moment it was set, and on this arm a count of error is
clearly felt at J4.

**Questions for Barrett, with these numbers**

- The current-sense zero: is there an automatic zeroing at enable, or a
  fractional bias, in firmware? A one-count residual (14 mA) on the 20 A-range
  sensor is a perceptible torque ripple on a 5 A-continuous WAM motor.
- What determines the enabled-state zero versus the filtered reading with
  the bridge off, and how stable should it be over temperature?
- The 4.3.2 low-torque dithering and the new biquad current filters: are they
  switchable, and could either interact with a bias error?
- A lower sensor range or higher-resolution current path for the WAM
  variant, and the intended Beta gain-factor calibration current (the routine
  gives values 10% apart at 0.5 A and 1.5 A).

## Where everything is

| Item | Location |
|---|---|
| Full-day bus capture | odin `~/sbflash/capture-run2.log` |
| Calibration logs | odin `~/sbflash/enczero-20260916-*.log` |
| Probe data | odin `~/sbflash/iqprobe-4-20260916-183123.csv`, `vprobe-4-20260916-185403.csv` |
| Analysis scripts | odin `~/sbflash/ripple.py`, `pdo_timing.py`, `fault_window.py`, `sb_events.py`, `iq_analyze.py`, `v_analyze.py` |
| Probes and experiments | odin `~/sbflash/iq_probe.py`, `v_probe.py`, `enable_probe.py`, `set_cc_pi.sh`, `set_cc_gain.sh`, `set_vel_filter.sh`, `set_torque_scale.sh`, `set_ovp.sh` |
| Puck calibration and readers | odin `~/puckutility/cal_headless.py`, `cal_read.py`, `fault_read.py`, `cl_read.py` (venv `~/puckutility-venv`); repo `puckutility/cal_headless.py` |
| Sign and position tests | odin `~/sbflash/lbc_torque_sign`, `lbc_position_watch`; repo `tests/lbc_*.cpp` |
| Earlier bench records | repo `puck2/build/sb-bench-2026-09-15/` (console dumps, flash logs, puck CSVs, gravitycal) |
| Puck 4 manual | repo `dynamometer/P4_CANopen_Manual_RevBA.pdf` |
| Firmware release notes | https://barrett.com/puck-firmware |
| Safety-board CANopen port | repo `puck2/source/CANOPEN.C` (keepalive: `canopenCwKeepAliveTick`, `CO_CW_KEEPALIVE_MS`) |
| Host torque path | `src/products/motor_puck.cpp` (`sendPackedTorques`), `include/barrett/products/detail/low_level_wam-inl.h`, RPDO/TPDO setup in `src/bus/canopen.cpp` (`configureMotorPuck`) |

Two unrelated faults were chased and fixed on the same bench day and are not
part of this: the drives' over-voltage trip on downward moves (bus regen with
the safety board's shunt effectively disabled), and J4 saturating its output
limit (the commutation error above).
