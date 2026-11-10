# Linux SYNC timing investigation — 2026-09-17

The host targets one CANopen SYNC every 2 ms (500 Hz). A typical period of
2.000 ms does not describe the worst delay. In the reported three-minute
capture, 737 intervals exceeded 3 ms and the longest was 14 ms: seven nominal
cycle intervals between SYNC frames. This is host/bus timing evidence, not a
measurement of cogging or proof of the cause of a particular velocity fault.

## What was verified

On Odin, the running Haptic World process had two threads, both `TS`
(`SCHED_OTHER`), including its control worker. Running the application under
sudo had not selected a real-time scheduling policy. `/proc/PID/status`
reported `VmLck: 0 kB`. The kernel was Ubuntu `5.15.0-140-lowlatency`.

A separate passive 30-second capture, saved on Odin as
`~/sbflash/sync-review-baseline-20260917.log`, contained 15,000 SYNC frames:

| Metric | Result |
| --- | ---: |
| Complete intervals | 14,999 |
| Median interval | 2.0001 ms |
| 99th percentile | 2.0590 ms |
| Maximum | 2.1210 ms |
| Intervals over 3 ms | 0 |
| Missing position feedback | 0 for each of nodes 1–4 |

That window does not reproduce the earlier pauses. It shows why before/after
captures should cover several minutes and representative application activity.
No control frames were transmitted for this check.

## Patch prepared locally

The native Linux path used `timerfd` but ignored the execution manager's
requested priority. The patch configures **only the control worker** with
`SCHED_FIFO` at the configured priority (normally 50), calls
`mlockall(MCL_CURRENT | MCL_FUTURE)`, and touches 64 KiB of worker stack before
entering the loop. Priority 0 explicitly opts out for offline tests.

It also:

- Uses recursive pthread mutexes with `PTHREAD_PRIO_INHERIT` on Linux.
- Uses `CLOCK_MONOTONIC` for elapsed-time measurements and deadlines.
- Reports scheduling/memory-lock/timer setup errors to `start()` instead of
  silently starting without real-time scheduling, terminating in the worker,
  or waiting forever for it to start.
- Closes timer descriptors, retries interrupted timer reads, reports other
  timer failures, and joins failed workers during shutdown.

Real-time setup belongs to `RealTimeExecutionManager`; a standalone
`PeriodicLoopTimer` remains a timing primitive on native Linux. Xenomai retains
its own task setup. Rebuild applications against the updated headers and
library together. No live library or running controller has been replaced.

## What this patch does not resolve

`CANopen::sdoTransact()` holds the CAN mutex across a request/reply wait, and
`Puck::getPropertyHelper()` similarly holds it while receiving. These are used
by configuration/status operations. A control thread waiting for that mutex
cannot send SYNC, regardless of its scheduler priority. Priority inheritance
helps a runnable lock owner finish, but cannot accelerate a sleeping owner or
a delayed device reply.

The follow-up is to serialize SDO transactions separately from the short CAN
I/O lock, preserving request/reply ownership while allowing SYNC and PDOs to
continue between polls. Audit outer locks in startup/zeroing and the split
property APIs before making that change; simply deleting the lock can mix up
concurrent replies or introduce deadlocks.

The feedback receive path also busy-polls and can wait up to one second on
missing feedback. Its timeout and blocking behavior need a separate review,
especially before moving to a kernel with threaded CAN interrupts. The present
patch is not a hard real-time guarantee. No safety thresholds or watchdogs
were changed.

## Validation and rollout

Local validation passed: C++ syntax checks for the execution manager and the
Mac mutex fallback, Python runner syntax, and 69 CANopen/startup regression
scenarios. Odin became unreachable over SSH before the new Linux runtime tests
or the complete Linux build could run. The native Linux changes are therefore
**not yet validated or deployed**.

On Linux, from this source tree:

```sh
python3 tests/run_linux_realtime_tests.py --output /tmp/lbc-linux-rt-test
sudo /tmp/lbc-linux-rt-test --probe
```

The first command runs without root or CAN. It exercises real timers and
mutexes, intercepts privileged scheduling/memory calls, and tests permission
failures, timer failures, interrupted reads, descriptor cleanup, recursive
locking, startup retries, 100 restarts, and worker-error shutdown.
The second command uses an empty execution manager to verify actual Linux
FIFO/50 and memory-lock permissions; it also opens no CAN interface.

Before applying to hardware, build the complete library and the demo against
the same updated source/headers in a separate prefix. Stop the demo normally
before switching libraries, then relaunch with the usual sudo-based launcher.
Do not overwrite the shared library mapped by a running controller.

After restart, substitute the controller PID below:

```sh
ps -L -p PID -o pid,tid,cls,rtprio,comm
grep VmLck /proc/PID/status
```

The control worker should be `FF` with priority 50 (or the configured value),
the main/UI thread should remain `TS`, and `VmLck` should be nonzero. An error
about real-time permissions or locked memory is a failed setup, not a reason
to set priority 0 on hardware.

Capture another representative three-minute interval on Odin:

```sh
timeout 180 candump -L can20 > /tmp/sync-after.log
python3 scripts/analyze_p4_cogging_capture.py /tmp/sync-after.log
```

`timeout` normally exits with status 124 when the capture period ends; run the
analysis as a separate command. Compare worst-case and tail intervals, not
just the median, and exercise the same status/configuration operations that
were present during the original gaps. If gaps persist, correlate them with
SDO traffic and scheduler/IRQ latency before changing kernels or CPU affinity.

References: [Linux scheduler policies](https://man7.org/linux/man-pages/man7/sched.7.html),
[memory locking](https://man7.org/linux/man-pages/man2/mlock.2.html),
[pthread priority inheritance](https://man7.org/linux/man-pages/man3/pthread_mutexattr_setprotocol.3p.html).
