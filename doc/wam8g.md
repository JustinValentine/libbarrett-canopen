# wam8g: eight measured joints, seven active joints

The wam8g is a 4-DOF WAM carrying a wrist with a passive encoder joint (J5)
and three motors (J6 roll, and J7/J8 through a differential). All eight
pucks are P4s on one CANopen bus; see [canopen.md](canopen.md).

`barrett::systems::Wam8g` owns nodes 1-8 as one product with one 500 Hz
control loop. It is a `Wam<8>`: everything written against `Wam<DOF>` works
on it with eight-wide vectors. J5 has no motor, so its torque is never sent
and its controller gains must be zero. On top of that the class has a
seven-wide "active" view that leaves J5 out.

| Interface | Size and order |
| --- | --- |
| Inherited from `Wam<8>`: `jp_type`, `jv_type`, `jt_type`, `input`, `jpOutput`, `jvOutput`, `getJointPositions()`, `moveTo(jp_type)`, `trackReferenceSignal(...)`, `getToolJacobian()`, ... | 8: J1, J2, J3, J4, J5, J6, J7, J8. J5's torque is ignored |
| `active_jp_type`, `active_jv_type`, `active_jt_type`, `activeInput`, `activeJpOutput`, `activeJvOutput`, `getActiveJointPositions()`, `getActiveJointVelocities()`, `getActiveJointTorques()`, `getActiveHomePosition()`, `moveTo(active_jp_type)`, `trackReferenceSignal(active_*)` | 7: J1, J2, J3, J4, J6, J7, J8 |
| `getActiveToolJacobian()` | 6 x 7: the active joints' columns |

J5 is always measured. Its angle enters the forward kinematics, the Jacobian
and gravity compensation every cycle, but nothing can command it: Cartesian
forces are applied through the seven active columns of the Jacobian, with J5
at its measured angle.

## Detection and startup

A wam8g is detected when pucks 1-8 answer and the configuration has a `wam8g`
profile. `ProductManager` then reports no separate 4-DOF or 7-DOF WAM, gimbals
or force/torque sensor. Declaring `bus.expected_puck_mask = 0x01FE` in the
robot's `default.conf` is recommended: without it, a wam8g whose puck 8 does
not answer would be detected as a 7-DOF WAM.

During wake the library marks node 5 as sensor-only, both for itself and on
the safety board (object `0x2100:04`). A sensor-only puck sends its position
like the others, but has no torque or controlword PDO and is never enabled.
Losing its position feedback still stops the WAM.

The safety board limits joint velocity on J1-J4 only. The wrist joints have
torque and communication monitoring but no velocity limit.

## Configuration (`config/wam8g.conf`)

- `low_level.j2mp`: puck 5 is J5 (1:1), puck 6 drives J6 (N2 = 8.7456) and
  pucks 7 and 8 drive J7 and J8 through a differential (N1 = 7.5077):
  `m6 = -N2 q6`, `m7 = N1 (q7 + q8)`, `m8 = N1 (-q7 + q8)`. `Wam8g` refuses a
  matrix that couples J5 to a motor.
- `canopen_position_sign` / `canopen_torque_sign`: -1 on the arm's four
  motors and +1 on the wrist's in the shipped file. Check them on each arm.
- `kinematics`: the DH rows of J4-J7 use fixed offsets
  ([dh-fixed-offsets.md](dh-fixed-offsets.md)).
- `kinematics_calibrated` and `gravity_calibrated`: the Cartesian interface
  and gravity compensation of the `Wam8g` view throw until these are true.
  Set `gravity_calibrated` after running `bt-wam-gravitycal`; the shipped
  `calibration_data/wam8g/gravitycal.conf` is a placeholder. The plain
  `Wam<8>` view is not guarded.
- J5's position and velocity gains must be zero; `Wam8g` checks this.

`bt-wam-zerocal` and `bt-wam-gravitycal` use the `wam8g` entries of
`calibration.conf`. The gravity calibration poses keep J4 and J8 clear of
their stops.

## Using it in an application

With the standard main function, a wam8g is passed to the ordinary
`wam_main<DOF>()` template with `DOF = 8`, so generic programs run on it
unchanged. An application that cannot be built for eight joints defines
`BARRETT_SMF_NO_WAM8G` and exits with a message on this robot.

An application that wants the active view defines `BARRETT_SMF_WAM8G` and
implements `wam_main(int, char**, ProductManager&, systems::Wam8g&)`; see
`examples/wam8g_state.cpp`. A custom main calls `pm.getWam8g()` after waking
the pucks.

For teleoperation, `activeJpOutput` and `activeJvOutput` give the seven joint
references and `activeInput` takes seven joint torques. For haptics, connect
world-frame force and moment signals to `getToolForceInput()` and
`getToolTorqueInput()`. These inputs are summed with `Wam<8>::input`, the
supervisory controllers and gravity compensation.

## Tests

```sh
python3 tests/run_wam8g_tests.py
```

builds `Wam8g`, the control graph, the low-level driver and the kinematics
against a simulated CAN bus. No hardware is used.
