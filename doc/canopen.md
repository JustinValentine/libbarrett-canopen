# CANopen WAM support

This version of libbarrett drives a WAM whose motor controllers are P4 pucks
(CiA 402 drives in cyclic synchronous torque mode) and whose safety board runs
the CANopen firmware. It replaces the BarrettCAN transport: P2/P3 pucks and
BarrettCAN safety boards are not supported.

The bus itself (node IDs, COB-IDs, objects, PDO layouts, state machines and
timing) is specified in `docs/canopen-contract.md` of the safety-board
firmware repository. Comments in this library cite it as "the contract".

## Requirements

| Component | Version |
| --- | --- |
| P4 pucks | firmware 4.3.2, 4096 counts per revolution on nodes 1-4 |
| Safety board | CANopen firmware r206 or later. r209 keeps the WAM zero across program runs; r210 is needed for reliable activation with eight pucks |
| Control loop | 2 ms (500 Hz); `ProductManager::getExecutionManager()` refuses any other period |
| Host | Linux with SocketCAN. The control thread runs `SCHED_FIFO` with memory locked, so run as root or grant `CAP_SYS_NICE` and `CAP_IPC_LOCK` |

Nodes 1-8 are pucks, node 10 is the safety board and the host is node 127.

## What the library does on the bus

- **Properties** (`Puck::getProperty()` / `setProperty()`) are expedited SDO
  transfers. The safety board serves every BarrettCAN property `p` at object
  `0x2000 + p`. A P4 has objects for `VERS`, `SN`, `MODE`, `T`, `P`, `MECH`
  and `CTS` only (`Puck::propToOd()`).
- **Enumeration** probes each node with an SDO read of `0x1000`.
- **Wake** (`ProductManager::wakeAllPucks()`, called by the standard main
  function) tells the safety board which pucks to watch, resets each puck
  with NMT, writes its PDO configuration in pre-operational, starts the bus
  and selects CST mode on each drive. The drives are left disabled: only the
  safety board enables them, on Shift-Activate.
- **Each control cycle** (`LowLevelWam::update()` and `setTorques()`) sends one
  SYNC, reads one position PDO per puck (32-bit counts) and sends the torques
  as two frames, `0x201` for pucks 1-4 and `0x205` for pucks 5-8, in per-mille
  of each motor's rated torque (object `0x6076`). The host heartbeat (`0x77F`)
  goes out every 250 ms.
- SDO transactions are serialized by `CommunicationsBus::getSdoMutex()`,
  separately from the bus I/O mutex, so a slow or lost reply does not hold up
  the control cycle.

Not available over CANopen: joint encoders, the BarrettHand, the force/torque
sensor and `SafetyModule::setTorqueLimit()`.

## Zeroing

A P4's position counter restarts at zero whenever the puck is reset and cannot
be set, so the library keeps the WAM zero as an offset per puck
(`LowLevelWam::definePosition()`). The safety board's `ZERO` property stays 0,
which keeps the board on its per-joint velocity limits.

- **Keeping the zero.** With safety-board firmware r209 or later the zero is
  also stored in the board's RAM (object `0x2103`). The next program start
  finds it there, checks that every puck has counted without interruption
  since, and then neither resets the pucks nor asks for the home position.
  The board discards the record at power-up, on leaving E-stop and whenever
  a puck boots or is reset. To zero the WAM again on purpose: E-stop,
  Shift-Idle, restart the program.
- **Zero compensation.** A P4's single-turn encoder (`0x3012:01`) is absolute
  within one motor turn. After `bt-wam-zerocal` has recorded `zeroangle` and
  `zeroangle_dir`, the WAM only needs to be within half a motor turn of home
  at each joint when it is zeroed. `zeroangle_dir` is the direction of that
  encoder relative to the position counter; zerocal measures it from the
  motion during the calibration, so every motor has to turn. A motor without
  a direction (0) is zeroed at the home position as before.

## Configuration

In addition to the usual files (`default.conf`, `wam4.conf`, calibration
data):

- `bus.expected_puck_mask` (optional): the robot's puck set, bit n = node n.
  With it, a missing puck stops startup instead of the library detecting a
  smaller WAM.
- `low_level.canopen_position_sign` and `low_level.canopen_torque_sign`
  (optional, default +1): the sign of each motor's reported counts and of its
  commanded torque relative to the convention `j2mp` encodes. The two must
  agree for each motor. Measure them on each arm, and only after the pucks'
  commutation zero has been calibrated.
- `low_level.zeroangle` and `low_level.zeroangle_dir`: written by
  `bt-wam-zerocal`.

The safety board's velocity and torque limits (`VL1`, `VL2`, `TL1`, `TL2`)
are held in its RAM and return to their defaults when the board powers up.
`SafetyModule::setDefaultSafetyLimits()` does not change them on a CANopen
board; set them with `SafetyModule::setVelocityLimit()` or over SDO.

## Tests

These run without a robot or a CAN interface. They need clang, Boost, Eigen,
GSL and libconfig.

```sh
python3 tests/run_canopen_startup_tests.py        # enumeration, wake, zero keeping
python3 tests/run_can_sdo_concurrency_tests.py    # SDO and I/O locking
python3 tests/run_linux_realtime_tests.py         # scheduling, timer, mutex (Linux only)
python3 tests/run_dh_offset_tests.py              # fixed DH offsets
python3 tests/run_wam8g_tests.py                  # wam8g against a simulated bus
python3 tests/run_torque_hold_tests.py            # the torque ramp of lbc_torque_sign
```

`tests/` also holds three programs that run on the robot (`lbc_probe`,
`lbc_position_watch`, `lbc_torque_sign`). The P4-WAM-Setup repository builds
them and describes how they are used when setting up an arm.
