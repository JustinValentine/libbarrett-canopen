# Bounded gravity-fit diagnostic on opus

Run from a supported, repeatable home pose, with the pendant in Shift-Idle:

```sh
cd ~/lbc-tools
./run_gravity_ramp_logged.sh
```

After reviewing the initial capture, the operator-requested extended trial is:

```sh
./run_gravity_ramp_logged.sh --30-percent
```

This ramps to 30% over 10 seconds, holds for 60 seconds, then removes torque
and requests Idle (70 seconds total). At the operator's request, this mode
permits hand-guided motion: it has no added displacement, speed or diagnostic
torque cutoffs. Watchdog, invalid-data, communication and operator stops remain
active. Compensation is recomputed from the measured pose; there is no position
hold. Puck/safety-board protections and the passive-J5 mask remain in force.

Follow the normal home/zero prompt, then Shift-Activate only after the
diagnostic prints its support instructions. The arm and wrist need external
support throughout: these trials supply only 10% or 30% of the fitted
gravity torque. Software cutoffs cannot hold a falling arm or replace the
pendant, drive supervision, or physical support. This is partial compensation;
the operator must still support the remaining weight during hand-guided motion.

The separate executable is `build/ex04_gravity_ramp`. It leaves stock example 4,
the calibration file, home, gains and drive settings unchanged. It observes the
existing fitted gravity output through a dedicated torque gate; it never calls
`gravityCompensate(true)` and never marks the fit as validated.

The two trial presets have fixed settings:

- Zero torque while waiting for activation. The default 10% diagnostic also
  monitors displacement while waiting; the 30% trial permits manual movement.
- Startup MODE reads use libbarrett's three-attempt retry with 50 ms polling;
  the torque gate stays zero throughout. Motion-time failures still stop it.
- Default: linear 0–10% ramp over 3 seconds, followed by 0.5 seconds at 10%.
- `--30-percent`: linear 0–30% over 10 seconds, followed by 60 seconds at 30%.
- Then zero the torque gate and request safety-board Idle. It cannot restart.
- Default 10% diagnostic only: any of the eight joints moving 0.020 rad
  (1.15 degrees), or reaching 0.15 rad/s in a 20 ms estimate, ends the trial.
- Default absolute requested joint-torque ceilings in J1..J8 order are
  `(2.5, 2.0, 1.5, 1.5, 0, 0.20, 0.10, 0.10)` Nm. They apply only to the
  default 10% diagnostic; `--30-percent` has no added diagnostic torque caps.
  These are host diagnostic caps, not puck or safety-board settings. Exceeding one aborts the default trial;
  torques are not silently clipped to change the fitted proportions. J5 is
  always commanded zero.
- Missing/nonfinite feedback or model output, a control update gap over 20 ms,
  a monitor-thread gap of 50 ms, or full sample storage latches zero.
- Ctrl-C, termination, terminal hangup and Ctrl-Z request a latched abort;
  Ctrl-Z does not suspend the diagnostic while it is applying torque.

Cutoffs execute inside the control graph independently of terminal and file
I/O. The main thread monitors safety mode and requests Idle after a cutoff.
An SDO timeout may delay that request; the torque gate stays zero. If the control
thread itself fails or the process is forcibly suspended/killed, the software
gate cannot deliver fresh CAN frames: existing safety-board/drive supervision
and the pendant remain necessary. If Idle cannot be confirmed, press Shift-Idle.

The wrapper records CAN, terminal output, exact configuration snapshots and
binary/configuration hashes under `~/sbflash/gravity-ramp-*/`. `samples.csv`
records the control-loop position, window-estimated speed, full model torque,
commanded torque, fraction, displacement, and stop reason. Torque is commanded,
not measured. Samples are buffered in memory and written after the stop. The
extended trial preallocates 80,000 records, covering its entire duration at the
installed 500 Hz rate and up to 1 kHz. No log allocation occurs in the control loop.

Only these two presets are accepted; there is no unlimited-duration or
arbitrary-fraction option. `build/ex04_gravity_ramp --help` exits before
opening CAN; all normal invocations initialize the WAM.

Offline verification:

```sh
python3 tests/run_gravity_ramp_tests.py
```

This exercises the actual libbarrett control graph with simulated time and
feedback, with address/undefined-behavior sanitizers. It checks both ramps and
holds, full-duration recording at 500/1000 Hz, fault stops late in the 60-second
hold, hand motion and changing model torques in 30% mode, default-mode
motion/speed cutoffs and torque ceilings, passive masking,
invalid/missing input, monitor stalls, control delays, signals, and latched
stops. It does not validate the physical gravity fit or hardware response.
