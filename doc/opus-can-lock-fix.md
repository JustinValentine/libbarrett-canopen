# Opus CAN reply-wait fix — 2026-09-22

A missing safety-board MODE SDO reply held the CAN I/O mutex for the entire
one-second receive timeout. The control worker needs that same mutex for SYNC,
position feedback, torque quads and host heartbeat. Retrying MODE after the
timeout did not prevent the watchdog fault caused by the blocked worker.

`CommunicationsBus::getSdoMutex()` now serializes SDO requests and replies
separately from CAN I/O. `BusManager` delegates transaction ownership to the
underlying bus and holds the I/O mutex only while accessing buffers and polling
the transport. Puck property reads and CANopen uploads/downloads share this
transaction lock. Split property callers must hold it across send and receive.
Acquire the SDO lock before the I/O lock whenever both are needed.

Startup still excludes cyclic traffic while resetting/remapping drives, with
the corrected lock order. Software zeroing collects its SDO results before
committing all offsets under the I/O lock; a failed read leaves the old offsets
intact. Position-feedback waits retain their existing fail-stop behavior.

The torque-sign tool runs its ramp, displacement stop and elapsed-time limit
in the control graph. If the main/status thread fails to refresh within 50 ms,
the next executing control cycle latches zero torque. Late replies cannot
restart it. This avoids continuing an old nonzero command while an SDO times
out. It is not a hard real-time guarantee if the control worker itself stops;
the existing safety-board watchdog remains responsible for that failure.

Linux sanitizer testing also exposed references to unconstructed Source/Sink
members in `LowLevelWamWrapper`. The owning members now precede the public
reference aliases in declaration and initialization order.

## Validation

- `python3 tests/run_can_sdo_concurrency_tests.py`: real mutexes, BusManager,
  Puck and CANopen paths over an in-memory transport; lost replies during
  blocking reads, timed reads, uploads and downloads; concurrent clients,
  independent buses, timeout cleanup and exception cleanup. On opus, cyclic
  traffic continued during the one-second timeout, with maximum observed
  gaps below 2.2 ms in this test. These are simulated transport timings.
- `python3 tests/run_canopen_startup_tests.py`: 69 startup scenarios.
- `python3 tests/run_wam8g_tests.py`: production eight-joint control graph,
  passive motor isolation, calibrated transmission signs, zeroing, and torque
  ramp stops for displacement, time, stalled status polling and invalid input;
  AddressSanitizer and UndefinedBehaviorSanitizer enabled.
- `python3 tests/run_linux_realtime_tests.py`: Linux timers, recursive priority
  inheritance mutexes, startup/error paths and 100 worker restarts; no CAN.
  The real `--probe` also passed as user `opus`: SCHED_FIFO/50, 80,220 KiB locked.

## Deployment layout

The isolated source, build, library prefix and logs are under
`/home/opus/lbc-updates/can-lock-20260922-v1/`. All five programs in
`/home/opus/lbc-tools` must be rebuilt against that prefix together: the bus and
WAM wrapper layouts changed, so mixing old binaries with these headers/library
is unsupported. The original `/home/opus/lbc-install` remains available for
older applications. Do not copy the new library over that old installation.

All five tools were rebuilt in the usual `~/lbc-tools/build` directory and
their library resolution was checked. Original tools/build files are backed
up in `can-lock-20260922-v1/original-tools`. The deployed `lbc_torque_sign`
SHA-256 is `7b3e7014d5564fe926b29e19e01a5f7a590543efeffeff969b6b5b07ed3ace50`.
The active `~/.barrett` configuration hashes were unchanged.

No robot activation or torque test is part of the automated verification.
The fix removes a host software stall; it does not establish why the adapter
lost a particular boot-up or SDO frame, or rule out physical CAN faults.
