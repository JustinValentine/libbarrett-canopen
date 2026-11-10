# IROS WAM startup and the combined WAM8g product

The confirmed hardware is a regular 4-DOF WAM plus one passive encoder joint
and three further P4-37 pucks, using **Puck4—mk1-v4.3.2.ebin**. Barrett lists
mk1 as the build for P4-16/P4-37 on its
[firmware page](https://barrett.com/puck-firmware).
Node map, confirmed by the 2026-09-21 scan on opus: WAM motors 1–4, passive
encoder 5, wrist motors 6–8, safety board 10. Limits were left unchanged at the
user's request.

## Host configuration and firmware pairing

SB r206 and later block activation until the host commissions its required
node set through `0x2100:05`; r208 adds the P4 statusword timing, r209 the
host zero record (the zero survives program restarts, see
[wam8g.md](wam8g.md)). Older host builds cannot perform this
handshake. The new host stops startup if either mask write/readback fails.

The IROS **host's** configuration should contain:

```text
bus:
{
    port = 0;
    expected_puck_mask = 0x01FE;
};
```

The mask is optional and belongs in the robot host's own `default.conf`, not
in the shared template: without it the product is identified from the pucks
that answered, as libbarrett always has, and an eight-puck robot that lost
node 8 would be detected as a 7-DOF WAM. With it, any mismatch stops startup.
`config/default.conf` includes `wam8g.conf` (the combined product).
`BARRETT_CONFIG_FILE` can select a specific config; the
library also uses `~/.barrett/` when that directory exists. Merely editing the
checkout does not update the robot host's installed config.

`wam8g` fixes the passive sensor to node 5; its public commands omit J5, and
the sensor-only exemption cannot be moved onto an active motor. The host
checks the expected physical set before selecting a product or waking drives,
so losing node 8 cannot silently turn this system into a 7-DOF WAM.

For each Shift-Idle session:

1. The SB powers the motor bus and listens for boot-up/heartbeat messages.
   Pucks with persistent logic power may send none; CANopen firmware may still
   enter IDLE, with activation blocked until commissioning.
2. The host uses SDO discovery, checks its configured expected set, and writes
   and reads back the sensor mask `0x2100:04` and required mask `0x2100:05`.
   The required mask installs the full SB watch list and invalidates old PDO
   readiness. It is RAM-only and clears on Shift-Idle/reset-communication.
3. The host validates 4096 motor encoder counts/rev on arm nodes 1–4, then
   resets and configures the pucks, including another encoder check after
   reset.  When the board still holds the previous run's zero (r209) and every
   puck checks out, it only reconfigures them: no reset, no drive enabled.
   Rejected CST timing writes stop startup. A custom execution-manager period
   must match the current 2 ms/500 Hz puck configuration.
4. During temporary CST mode setup the target is cleared first. Failure to
   finish setup or return the drive to Switch-on-disabled is fatal and requests
   SB ESTOP, followed by a best-effort drive disable. These are CAN requests;
   physical cutoff after CAN failure still needs hardware testing.
5. Before Shift-Activate, both torque quads must be streaming zeros, every
   required puck must have appeared, and all required position feedback must
   be fresh. The passive puck is exempt only from drive enable/CST checks.

## Combined product and application integration

`ProductManager::getWam8g()` now creates one `systems::Wam8g` owning all eight
nodes. It starts one 500 Hz loop with one SYNC, eight position samples and both
torque quads **before waiting for ACTIVE**. The passive J5 slot stays zero.

Use the seven active-joint interfaces for teleop and the eight-joint measured
outputs for local compensation. The Cartesian force Jacobian omits J5's column
but is evaluated at its current measured angle. See [wam8g.md](wam8g.md) for the
API, standard-main example, firmware contract and commissioning requirements.

Generic applications written against `Wam<DOF>` run on it as DOF = 8 through
the standard main. The haptic-world application opts out with
`BARRETT_SMF_NO_WAM8G` and is refused on this robot.

Puck2's eight-node position watchdog and torque monitor do not imply eight-joint
speed limiting: `getJointAngles()` and `safetyLimitJointVelocity()` still cover
only J1–J4. Wrist speed supervision remains a firmware commissioning task.

## Checks run without hardware

```sh
python3 tests/run_canopen_startup_tests.py
python3 tests/run_wam8g_tests.py
python3 ../puck2/tests/run_host_tests.py
```

The startup suite covers 70 transport/detection/zero-keeping scenarios. The WAM8g suite
compiles the real product, control graph, driver and kinematics against a fake
CAN bus, and syntax-checks ProductManager and both standard-main paths. It
checks seven-command/eight-measurement mapping, passive isolation, wrist torque
conversion, J5-dependent FK and Cartesian Jacobians/wrenches. Both suites use
AddressSanitizer and UndefinedBehaviorSanitizer. The puck2 suite includes J5
sensor-only activation, active J8, sensor-position loss and loss of either quad.

The complete Linux RT backend and physical assembled robot still require a host
build and commissioning. The older local Eigen zero-expression incompatibility
and GNU-only current-directory call were corrected to compile these host tests.

## Bench findings, odin 4-DOF WAM, 2026-09-16 (SB r208, P4 fw 4.3.2)

Built and run on odin (Ubuntu 20.04, private prefix `~/lbc-install`, config
`~/lbc-config-wam4`, SocketCAN can20). Zero-torque activation through the
library is verified; the first torque tests changed three things in the port:

- **Position and torque signs are per arm, per motor.** Measured against the
  joint stickers with `tests/lbc_position_watch.cpp` (hand-moved, drives
  disabled) and the ramped `tests/lbc_torque_sign.cpp`: the P4 firmware's
  reported counts are mirrored on all four pucks relative to the j2mp
  convention, and so is its torque direction. `LowLevelWam` reads two
  optional per-motor arrays from the arm's `low_level` block,
  `canopen_position_sign` and `canopen_torque_sign` (defaults +1), applied
  to the raw counts, the software zero and the sent per-mille. The odin
  arm's values are `(-1,-1,-1,-1)` for both and live only in its config
  copy. `MotorPuck`'s per-mille scale stays positive.
- **Calibrate the pucks' commutation zero first.** The first sign
  measurement gave `(-1,1,1,1)` with J2 needing 15 Nm to break away and a
  gravity fit 1.2-3x the stock model. The pucks' encoder-zero values had
  come from a generic CSV: PuckUtility's calibration (run headless with
  `puckutility/cal_headless.py`, 1.5 A) moved them by 56, 150, 134 and 115
  electrical degrees. Torque scales with the cosine of that error, so pucks
  2-4 had been delivering reversed, weakened torque and read as "+1". After
  calibration all four measure `-1` and J4 breaks away at 0.4 Nm.
- **Zeroing.** The puck's position counter restarts at 0 on the NMT reset that
  `Puck::wake()` issues, and writing the manufacturer user-zero object does not
  change 0x6064. `LowLevelWam` therefore keeps a software offset, set at zeroing
  from the pucks' SDO-read counts, and no longer writes the puck. The safety
  board's ZERO flag is cleared after every wake and never set by the host, so
  the home-position prompt appeared on every start and the board keeps its
  per-joint velocity limits, which do not depend on absolute counts. (Since SB
  r209 the zero survives program restarts; see wam8g.md.) The P4's raw
  single-turn encoder (0x3012:1, the MECH mapping, absolute within a motor
  turn) now gives P3-style rough placement at power-up once `bt-wam-zerocal`
  has recorded `zeroangle` and `zeroangle_dir`; see wam8g.md.
- **Statusword cadence.** The drive reports its statusword every 100 ms whatever
  transmission type is written; SB r208 sizes its timeouts for that.

Still open on that bench: the drives measure 50.5 V against their 52 V over-
voltage fault (bring the supply to 48 V or enable the SB bleed after
recalibrating its voltage sense); torque scale under load; gravity calibration
(the installed data is the stock template).
