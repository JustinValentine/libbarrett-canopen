# J8 torque test with J7 position hold

Use the existing torque test on opus, from the saved home in Shift-Idle:

```sh
cd ~/lbc-tools
./run_torque_sign_logged.sh 8 0.3 3 0.04 --hold-j7
```

It uses normal libbarrett activation, waits three seconds, captures the current
J7 position, then ramps J8 from zero to +0.3 Nm over three seconds. It stops at
0.04 rad (2.3°) displacement on ANY joint, or after 3.5 seconds. J7 is held by
its configured proportional/derivative gains (currently 10 and 0.1), with no
integral and at most 0.5 Nm. There is no move to zero and no gravity compensation.
To test the opposite torque direction, replace `0.3` with `-0.3` on a separate run.

Support the other joints and be ready to support J7 too when it stops. J1–J6
receive zero commanded torque. Both J7 hold and J8 torque turn off together at
every stop. This is a compliant encoder-based hold, not a mechanical lock; if
J7 moves 0.04 rad the test aborts. A stalled status poll for 50 ms, missing or
invalid feedback, Ctrl-C/Ctrl-Z/termination also latches zero. Keep Shift-Idle
ready. No software gate can replace drive/pendant supervision if the control
process itself stops running.

The flag requires an eight-joint WAM, J8 as the requested joint, |J8 torque| at
most 0.5 Nm and J8 displacement cutoff at most 0.349066 rad (20°). J7 and all
other joints retain a cutoff of min(0.04 rad, the requested J8 cutoff).
Existing commands without
the flag retain the single-axis torque mode. No saved gains, calibration,
current limits or home values are changed. CAN/terminal capture uses the same
`~/sbflash/torque-diagnostic-*` folders as before; the terminal reports all
encoder-derived displacements and the last J7 holding command.

For a larger J8 trial, the fourth number is the travel cutoff in radians;
20° is 0.349066 rad. This is measured relative to the position at ramp start,
not absolute DH zero. It only removes torque when crossed: it neither brakes
nor holds at the angle, so coasting beyond the cutoff is possible. The same
time limit still applies and may end the trial before the angle is reached.
Confirm physical J8 identity, positive direction and clearance with J7 held
at the actual starting position before a larger trial. The older estimated
coupled limits cannot certify clearance at the currently configured home
(J7 approximately -93°).

Offline verification: `python3 tests/run_torque_hold_tests.py`. Tests exercise
the actual control graph with simulated feedback and sanitizers, including
coupled shutdown of the ramp and hold, both torque directions, missing inputs
and watchdogs. They do not establish physical joint identity or holding ability.
