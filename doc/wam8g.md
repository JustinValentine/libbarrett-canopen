# WAM8g: eight measured joints, seven active joints

`barrett::systems::Wam8g` owns nodes 1–8 as one product and one 500 Hz CANopen
loop. `ProductManager::foundWam8g()` / `getWam8g()` select it when the complete
eight-node set and the `wam8g` profile are present. `Wam8g` **is a** `Wam<8>`:
everything written against `Wam<DOF>` works on it with eight-wide vectors, and
J5 is simply an inert slot (no motor, zero gains, never sent torque). On top of
that it adds a seven-wide "active" view; every seven-wide member is named
`active*` or takes an `active_*` type, so the two views cannot be confused.

| Interface | Size and order |
| --- | --- |
| Everything inherited from `Wam<8>`: `jp_type`, `jv_type`, `jt_type`, `input`, `jpOutput`, `jvOutput`, `getJointPositions()`, `getJointVelocities()`, `getJointTorques()`, `getHomePosition()`, `moveTo(jp_type)`, `trackReferenceSignal(...)`, `getToolJacobian()` (6 × 8) | 8: J1, J2, J3, J4, **J5**, J6, J7, J8. J5's torque slot is ignored |
| `active_jp_type`, `active_jv_type`, `active_jt_type`, `activeInput`, `activeJpOutput`, `activeJvOutput`, `getActiveJointPositions()`, `getActiveJointVelocities()`, `getActiveJointTorques()`, `getActiveHomePosition()`, `moveTo(active_jp_type)`, `trackReferenceSignal(active_*)` | 7: J1, J2, J3, J4, J6, J7, J8. Torques are commanded Nm, not measured torque |
| `getActiveToolJacobian()` | 6 × 7; linear then angular rows, active joint columns |

J5 is always measured and always passive. Its measured angle participates in
forward kinematics and gravity evaluation every cycle. Cartesian forces use the
partial Jacobian with J5 fixed at that current angle: `tau_active = J_active^T w`.
This does not model a five-bar constraint or infer how J5 follows active joints.
The full measured tool velocity includes actual J5 motion. A Cartesian command
cannot servo J5. The low-level torque path removes its generalized torque before
the transmission and zeros its CAN torque slot afterward.

Gravity uses the eight-coordinate serial pose, then sends only the seven active
components. This supports local gravity compensation at the measured J5 pose;
it is not a model of forces redistributed through the human/closed linkage.

## Startup and puck2 contract

WAM8g is detected like every other WAM: eight motor-typed pucks on nodes 1-8
plus a `wam8g` profile in the config. Before puck wake, ProductManager writes
and reads back safety-board OD `0x2100:05` (the puck set the board must watch)
and `0x2100:04 = 0x0020` (sensor-only node 5). The existing puck2 CANopen
firmware supports this layout; no firmware behavior or limits were changed for
WAM8g.

`bus.expected_puck_mask` is optional. Without it the product is whatever
answered, as always. Declaring `0x01FE` in the robot host's own config makes a
missing puck stop startup instead of demoting the robot to a 7-DOF WAM; it is
recommended for the IROS host and must not be placed in a shared default.

The host sensor-only mask is stored per CAN bus. A WAM8g master and regular
WAM7 slave in the same process retain their respective passive/active J5 roles.

- One SYNC per 2 ms cycle; collect positions from **all eight** nodes.
- Torque quad A (`0x201`): J1–J4 motor commands.
- Torque quad B (`0x205`): **zero**, motor 6, motor 7, motor 8. Its last slot
  commands motor 8, which contributes to both J7 and J8. Wrist differential torques follow the configured
  transmission, rather than treating motors 6 and 7 as independent joint axes.
- While idle, continue both zero torque groups plus the host heartbeat. Puck2
  requires fresh positions and zero commands before pendant activation.
- Sensor-only exempts J5 from drive enable/CST checks, **not** position liveness.
  Missing J5 feedback or either torque quad still triggers the firmware watchdog.
- The zero is a software offset per puck, for all eight coordinates (a P4
  restarts its position counter at 0 on every reset). See "Keeping the zero
  across program runs" below. The safety board's ZERO flag stays clear,
  preserving its per-joint velocity checks for **J1–J4** rather than applying
  its stock WAM Cartesian model to this robot. Puck2's `getJointAngles()` and
  `safetyLimitJointVelocity()` calculate only those four joints. **J5–J8 do
  not yet have safety-board speed limiting**, even though all eight have
  torque/liveness monitoring. Wrist speed supervision needs a separate firmware
  extension with the wrist transmission/encoder calibration.

With `wam8g` configured, ProductManager does not report a separate WAM4, WAM7,
gimbals controller, or force/torque sensor at node 8.

## Keeping the zero across program runs

With safety-board firmware **r209** or later, a program run that zeroes the WAM
parks the zero in the board's RAM (object `0x2103`: per puck the joint angle
defined, the raw `0x6064` count and the single-turn `0x3012:01` reading at
that moment). The next program start then skips the NMT resets, which would
restart the counters, and keeps the zero instead of asking for the home pose:

```text
>>> Keeping the WAM zero from the previous run (no puck has been reset since).
```

The board forgets the zero exactly when it may be void: at its own boot, on
**E-Stop → Idle** (where the classic board also cleared ZERO), and whenever a
puck sends a boot-up or is sent an NMT reset. So Shift-Idle and restarting a
program keep the zero; E-Stop, a power cycle or any puck reset (PuckUtility,
bench scripts) mean homing again - and **E-Stop, Shift-Idle, restart** is the
way to re-home on purpose. Before keeping it, the host also checks every puck:
heartbeat period still the one it configured, position counter still in step
with the absolute single-turn encoder (mod one turn), drive still CST-armed and
disabled (a warm start never enables a drive). Any doubt means the normal
reset-and-prompt start, with the reason printed. Older boards (r208 and
before) behave as before: every start resets the pucks and asks for the home
pose. `zerocal` forgets the kept zero, so a new calibration takes effect on
the next run.

## Configuration (`config/wam8g.conf`)

State of the checked-in profile as commissioned on opus (2026-09-22):

1. Transmission: puck 5 is the passive J5 (1:1), puck 6 drives the J6 roll
   (N2 = 8.7456), pucks 7 and 8 form the J7/J8 differential (N1 = 7.5077):
   m6 = -N2 q6, m7 = N1 (q7 + q8), m8 = N1 (-q7 + q8), identified by hand
   motion ([record](wam8g-j78-identity-2026-09-22.md)). Per-motor signs
   `canopen_position_sign` / `canopen_torque_sign` are -1 on the arm pucks and
   +1 on the wrist, measured after the pucks' commutation zero was calibrated.
   J7/J8 positive directions still need a physical check.
2. Home: `calibration_data/wam8g/zerocal.conf`, the operator's repeatable rest
   pose. Its J1-J3 values were derived from count differences captured before
   the arm position signs changed to -1; recalibrate with `bt-wam-zerocal`
   (below).
3. Kinematics: J5-J8 DH rows confirmed by the operator,
   `kinematics_calibrated = true`. Without it the Cartesian entry points of the
   `Wam8g&` view throw; the generic `Wam<8>&` view is not guarded.
4. Gravity: fit with `bt-wam-gravitycal` on the robot (its `~/.barrett`). The
   checked-in `calibration_data/wam8g/gravitycal.conf` is still the zero
   placeholder with `gravity_calibrated = false`; copy the fit back if the
   repository should carry it.
5. Gains: arm position kp reduced 30%, wrist 20/10/10 with 5 Nm limits. A
   `control_signal_limit` of 0 clamps that joint's controller output to zero
   whatever its gains are. J5 position/velocity gains must stay zero and are
   checked at construction.

Software tests do not establish hardware torque direction, stability, stiffness
or gravity accuracy. Existing firmware torque/velocity limits remain in force.

## Zero calibration (`bt-wam-zerocal`)

Run once, and again only after something moves a joint relative to its motor
(a cable, a puck, the wrist). It measures the true joint angles of your home
pose against each joint's zero mark and each motor's single-turn encoder, and
writes `home`, `zeroangle` and `zeroangle_dir` to
`calibration_data/wam8g/zerocal.conf` (the old file is kept as a backup).

1. The installed `calibration.conf` (e.g. `~/.barrett/calibration.conf`) needs
   the eight `zerocal.wam8g` rows from `config/calibration.conf`; the config
   installer does not update an existing file.
2. Place the arm carefully at the home pose (this one time it must be exact),
   start `bt-wam-zerocal`, press Enter, Shift-Activate.
3. Step through the rows. Each moves the arm (gravity compensation off) and
   holds it; bring that joint onto its mark with the arrow keys (Left/Right pick
   the digit) and press Enter. zerocal records where the joint actually is, so
   a sticky joint (J6) can be helped onto its mark by hand. J5 only needs Enter.
4. After the last row it writes the file, prints which joints may now be
   placed roughly at power-up, moves home and waits for Shift-Idle.

### Rough placement at power-up

As on classic WAMs, the WAM then only has to be near home when a start asks
for the home pose. Each P4's built-in encoder (`0x3012:01`) is absolute within
one motor turn, so startup compares it with the reading recorded at zero and
corrects the zero, printing how far each joint was from home:

```text
  WAM zeroed from the single-turn encoders; it sat this far from home (deg): J1 +1.20, ...
```

The allowed error is half a motor turn: about ±4° at J1, ±10° at J4 and ±20°
at J6; J2/J3 share motors (±6° at J2 with J3 exact, ±10° at J3 with J2 exact)
and so do J7/J8 (|ΔJ7| + |ΔJ8| < 24°). Aim well inside that. Beyond it the zero
is silently a whole motor turn off (20° at J4); a motor read more than 60% of
the way to that limit prints a WARNING.

zerocal measures which way each encoder counts relative to its puck's
position counter from the motion during the calibration (it does not rely on
the puck's settings). Every motor therefore has to turn: the J2 and J4 rows
swing J1 and the J6 roll out and back with the arm straight up, and zerocal
prints any joint it still could not measure. A motor that never turned (the
pinned J5) gets `zeroangle_dir` 0, and its joint must be exactly at home at
power-up, which the pin guarantees. To turn rough placement off, delete `zeroangle_dir` from
`zerocal.conf`. A zerocal file written before this feature has no
`zeroangle_dir`, so the zero comes from the home pose alone.

## Using the product

`systems::Wam8g` is a `systems::Wam<8>`. The standard main dispatches it to
the ordinary template `wam_main<DOF>` with `DOF = 8`, so every generic
application, example and tool built on it (gravitycal, ex01-ex11) runs on it
unchanged: eight measured joints, eight commanded torques, and J5 is simply an
inert slot (no motor, zero gains, never sent torque). An application that
cannot be built for `DOF = 8` defines `BARRETT_SMF_NO_WAM8G` and is refused on
this robot with a message.

An application that wants the seven-active-joint view defines
`BARRETT_SMF_WAM8G` and implements `wam_main(..., barrett::systems::Wam8g&)`;
see `examples/wam8g_state.cpp`. That view adds `activeInput` (seven torques),
`activeJpOutput` / `activeJvOutput`, `getActive*()` getters,
`moveTo(active_jp_type)`, `trackReferenceSignal(active_*)` and
`getActiveToolJacobian()`. Everything else is the plain `Wam<8>` interface.
A custom main can call `pm.getWam8g()` after the usual idle, wake and
reference-pose sequence.

On an existing host installation, the config installer adds missing profile
files without replacing existing calibration or `default.conf`. Add
`@include "wam8g.conf"` to that host's active configuration (and, recommended,
the expected mask). Copying/building this checkout alone does not change the host.

For seven-active-joint teleop, use `wam.activeJpOutput` and `activeJvOutput` to map
to the slave's seven joint references. Feed returned seven active torques to
`wam.activeInput`. Joint offsets, signs, scaling, limits and engagement remain the
teleop application's responsibility. J5 stays in `wam.jpOutput` for local
compensation and logging, without a different teleop mode.

For haptics, connect world-frame force and moment signals at the configured tool
origin to `wam.getToolForceInput()` and `wam.getToolTorqueInput()`. They are summed
with the external active torque input, supervisory controller and optional
gravity compensation, then sent through the single combined hardware loop.
Use the pose/velocity/orientation output accessors for real-time feedback.

`idle()` follows the existing WAM convention: it disconnects the supervisory
reference controller. External torque/wrench inputs and gravity compensation
remain connected until the application disconnects them.

## Software verification

```sh
python3 tests/run_canopen_startup_tests.py
python3 tests/run_wam8g_tests.py
python3 ../puck2/tests/run_host_tests.py
```

The WAM8g test compiles the production product, control graph, low-level driver,
Puck/CANopen transport logic and C kinematics against a fake bus. It checks all
eight position reads, one SYNC and two quads per cycle, active-joint ordering,
passive isolation, differential torques, finite-difference Cartesian Jacobians,
J5-dependent pose/gravity, wrench projection, per-bus sensor masks, eight-joint
software zeroing and keeping the zero across runs (warm wake without resets or
drive enables), consistent velocity getters/ports and invalid configurations.
The puck2 tests exercise the production safety code with simulated peripherals,
including sensor-only J5 and active J8. These tests never open or activate hardware.
