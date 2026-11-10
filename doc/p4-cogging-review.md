# Follow-up review of the P4 cogging investigation — 2026-09-17

The cause remains unconfirmed. This review checks the investigation against
the local manual, host code, calibration code, and the original bench scripts.
The saved measurements on odin were **not** reanalyzed: SSH to
`192.168.1.244` timed out, and the raw captures were not found locally.
No drive configuration or safety-board firmware was changed during this review.

## What to test first

Distinguish a disturbance that repeats at a **fixed motor angle** from one
that repeats at a **fixed time interval**. Move J4 over the same modest range
in both directions, at two clearly different slow speeds, using the existing
zero-torque ex01 setup with the arm supported against gravity. Record a bus
capture and the intervals where each sweep was performed.

J4's reduction is 18:1 (`config/wam4.conf`) and the configured motor has six
pole pairs. Consequently:

| Pattern | Motor encoder spacing | J4 angle spacing |
|---|---:|---:|
| Once per motor revolution | 4096 counts | 20 degrees |
| Once per electrical revolution | 682.67 counts | 3.33 degrees |
| Twice per electrical revolution | 341.33 counts | 1.67 degrees |
| Six times per electrical revolution | 113.78 counts | 0.556 degrees |

These are predictions from configuration, not measured tooth spacings.
A repeatable 3.33-degree pattern makes current-sense offset or another
electrical-angle-dependent error a better lead. A disturbance consistently
aligned to the 50 ms keepalive, at both speeds, makes the repeated mode write
a better lead. Merely seeing a 20 Hz spectral peak at one speed does not
separate the two mechanisms.

## Findings that change interpretation of the existing tests

### Current feedback cannot independently exclude current-sense error

If the sensor reads `i_measured = i_true + bias`, an effective current loop
commanded to zero produces `i_true = -bias`, while reporting approximately
zero. A stationary alpha/beta bias rotates into a sinusoidal q-axis error
once per electrical revolution. Thus, good drive-reported tracking and real
torque ripple can coexist. An independent current/force measurement or a
controlled change in calibrated bias is needed to distinguish this.

Warm bias calibration making little subjective difference is useful evidence,
but does not exclude an offset that changes with PWM operation, motor angle,
or measurement timing. The CLI calibration samples each Filtered channel once
and rounds Q12.4 to an integer ADC count before storing it
(`../puckutility/cli_ops.py`, `_cli_calibrate_ibias`).

### Resolve RMS versus peak before claiming a scaling error

The RevBA manual, pages 26 and 34, assigns different reference quantities to
`0x6074` (per-mille rated torque) and `0x6078` (per-mille peak current).
At first sight, subtracting their raw values in `iq_analyze.py` looks wrong.
However, the configured numbers satisfy:

```text
19.526 A * 0.457 Nm/A / sqrt(2) = 6.3098 Nm
rated torque                      = 6.3100 Nm
```

PuckUtility's `getMonitor()` explicitly converts reported current to RMS by
dividing by `sqrt(2)`. If the configured Kt is per RMS amp, the raw values are
effectively on the same torque scale for this configuration. If Kt refers to
peak q-axis amps, they differ by approximately sqrt(2). Confirm the motor/drive
convention before selecting either conversion; this review does **not**
establish a 41% tracking error. Changing `0x6076` requires recalculating the
comparison in either case.

### The proposed enable probe does not reliably isolate the hypotheses

The locally retained `enable_probe.py` was found in the previous session's
scratch directory. Its plain/CW-only paths set `0x6060` **before** enabling,
then never rewrite it after Operation Enabled. The host's `armCstMode()` does
the reverse, specifically because of the measured 4.3.2 mode-entry behavior.
Only the CW+mode keepalive path reliably attempts later mode writes. A
difference between these paths could therefore be a difference in active mode.

Furthermore, its keepalive sends `0x28A`, which all four arm drives subscribe
to after normal host setup. Passing node 4 does not isolate that broadcast.
The CW-only variant changes node 4's mapping, leaves it changed on exit, and
does not reinstall the CW+mode mapping for a subsequent comparison. It also
starts its transmitter before completing the SDO enable sequence.

Do not use that script unchanged for the A/B comparison. A valid replacement
needs a node-specific unused COB-ID, verified mapping in both cases, mode
selection after confirmed enable, continuous status/mode checks, and restored
mapping on exit. The selected drive's targets and torque offset must be zero;
the rest of the arm must remain in its intended disabled state. These are
bench-test requirements, not changes to the production safety contract.

Zero phase voltage (`--pva`) is also **not equivalent to a disabled drive**:
with the output stage switching, back-EMF from hand motion can generate
braking current. That comparison needs speed and measured current alongside
the subjective feel.

### The current-loop gain-factor test probably did not tune the current loop

The EDS names `0x3024` **TuningAppProperties**. PuckTuner reads the actual PI
coefficients from `0x2380:1/2`. The observed unchanged PI readback after
changing `0x3024:2` supports treating the earlier trial as inconclusive.
It does not establish whether firmware uses additional internal coefficients.

For a later PI A/B, capture the original IEEE754 values, use RAM-only writes
to the actual gains, verify readback, and restore those captured values.
The existing `set_cc_pi.sh` **saves every change to EEPROM** and its restore
case uses hardcoded nominal gains; it is not a temporary experiment.

### Sampling limits were described too strongly

An 85 Hz SDO poll is not inherently a 12 ms average. It reads successive
objects at different instants, and can miss or alias faster signals. Internal
filtering and object update rates must be established separately. Likewise,
the absence of a sixth harmonic does not rule out inverter distortion near
zero current, where a clean sinusoidal current-sign pattern is not assured.

`iq_analyze.py` infers the electrical-angle scale from the observed range;
this fails on short sweeps. RevBA page 26 specifies a **65536-count full
electrical revolution** for `0x60EA`. Both original harmonic analyzers also
fill unvisited angle bins with zero, which can distort harmonic amplitudes.
Use the documented scale and report coverage; do not interpret incomplete
angle coverage as a measured full-revolution waveform.

### Check the actual zero-torque state

ex01 and ex04 share standard-main startup; ex01 has no gravity input connected
to its torque summer. That supports the zero-command observation. However,
the manual explicitly adds `0x60B2` (torque offset) in CST, and the host does
not currently initialize that object. This is an unverified state variable,
not an established nonzero offset. At the next bench session, record
`0x6041`, `0x6060`, `0x6061`, `0x6071`, `0x6074`, and `0x60B2` in each condition.

## Offline capture check added with this review

From the libbarrett-canopen directory, using the existing full-day capture:

```sh
python3 scripts/analyze_p4_cogging_capture.py ~/sbflash/capture-run2.log \
  --start 1789603366 --end 1789603474 --node 4 \
  --cycles-csv /tmp/p4-gravitycal-cycles.csv
```

Those timestamps select the documented **gravity calibration** interval, not
the ex01 zero-torque trial. Select the actual ex01 timestamps to verify zeros.
The script reads files only, supports `candump -t a` and `candump -L`, and
reports SYNC period distribution, gaps over 3 ms, feedback omissions and
duplicates, command ranges, command age at SYNC, keepalive cadence/payloads,
and node status/mode frames. The CSV contains the first position in each
complete SYNC interval and the latest preceding torque command/keepalive.

Use steady intervals after host PDO setup; it assumes the WAM4 mappings and
does not reconstruct arbitrary remapping from SDO traffic. A missing position
stays blank, and an unknown command stays unknown. The final partial cycle is
excluded. Receive timestamps bound bus arrival timing, not the drive's exact
ADC or control-loop timing. These outputs help compare temporal and angular
patterns, but do not directly measure disturbance torque or prove causality.

Offline tests:

```sh
python3 tests/test_p4_cogging_capture.py
```

## Decision after those measurements

1. If the roughness follows electrical position, prioritize enabled-state
   current sensing, calibration, and PWM sampling. The observed 1x pattern is
   consistent with an offset, but the previous position-loop command trace is
   indirect evidence and should be repeated in both directions/speeds.
2. If it follows each keepalive at different speeds, perform the corrected
   CW-only versus CW+mode comparison on an isolated drive. A positive result
   would justify separating mode selection from periodic controlword refresh
   while preserving re-enable handling and watchdog behavior.
3. If neither is repeatable, test the actual current PI gains and obtain a
   faster synchronized current/voltage measurement before assigning blame to
   sensor noise or firmware PWM dithering.

Barrett's [4.3.2 release notes](https://barrett.com/puck-firmware) describe
changes to PWM dithering, current filtering, ADC/PWM synchronization, and mode
re-entry. This makes a firmware regression plausible, but does not identify
which change is responsible. A firmware A/B would test the whole bundle of
changes, not dithering alone.
