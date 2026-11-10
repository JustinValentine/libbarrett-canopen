# J7/J8 torque capture analysis, 2026-09-22

Read-only analysis of the operator's two runs on opus:

- J7: `/home/opus/sbflash/torque-diagnostic-20260922-164929-d9XnSN`
- J8: `/home/opus/sbflash/torque-diagnostic-20260922-164731-txSjYq`

Both runs used `run_torque_sign_logged.sh JOINT 1.0 3` and the default
0.020 rad motion cutoff. No robot executable was run, and no robot
configuration or software was changed during this analysis.

## Commands and feedback during the ramps

RPDO `0x205` contains four signed little-endian 16-bit torque commands for
motors 5, 6, 7 and 8, in per-mille of motor rated torque.

| Test | Nonzero command sequence | Raw motor 7 change | Raw motor 8 change | Reported joint changes at cutoff |
| --- | --- | --- | --- | --- |
| J7 | `(0,0,+1,-1)` through `(0,0,+16,-16)` | +93 counts | -104 counts | J7 +0.02013 rad; J8 -0.00112 rad |
| J8 | `(0,0,+1,+1)` through `(0,0,+14,+14)` | approximately +143 counts | approximately +58 counts | J7 +0.00868 rad; J8 +0.02053 rad |

The J8 pre-ramp motor-8 reading alternates between -12 and -13 counts.
Using -13 at 22 ms before the first nonzero command reproduces the terminal
delta; using -12 at 50 ms before it changes the reconstructed joint deltas
by one motor count. This is about 0.000102 rad per joint, and does not
affect the interpretation.

The signs match the current transmission: J7 uses the motor difference;
J8 uses the motor sum. The on-wire commands are distinct. Motor 8's
encoder changes sign between the tests, as expected. These observations
do not independently prove motor torque production or physical output
joint identity; the latter still relies on the operator's earlier
separate hand-motion checks.

All commands in `0x201` were zero. In both captures, all `0x205` commands
returned to zero at the cutoff and remained zero for the rest of the run.

| Test | First nonzero torque timestamp | First zero after ramp timestamp |
| --- | --- | --- |
| J7 | 1790110219.056870 | 1790110219.718985 |
| J8 | 1790110083.096631 | 1790110083.682679 |

Both drives reported statusword `0x0037` and mode display 10 (CST) throughout
the examined ramp windows. Every examined SYNC interval had exactly one
position PDO from each of nodes 1–8: 430 intervals around J7's ramp and
392 around J8's. There were no captured CAN controller error events during
either ramp.

## Unequal response in the J8 test

Motor 7 starts moving substantially before motor 8. Samples relative to the
first nonzero command, with raw encoder counts:

| Time (s) | Motor 7 | Motor 8 |
| --- | --- | --- |
| -0.022 | 14 | -13 |
| +0.200 | 32 | -11 |
| +0.300 | 87 | -12 |
| +0.400 | 97 | -9 |
| +0.500 | 113 | 3 |
| Cutoff (+0.586) | 157 | 45 |

This unequal motor motion explains the simultaneous computed J7 and J8
motion. Unequal friction, loading, or transmission compliance are possible
causes, not established diagnoses. The captures contain commanded torque
and encoder positions, not measured motor torque or independent output
joint angles.

## Motion after torque cutoff

The cutoff removes applied torque; it does not brake or hold position.
There is no gravity compensation in this tool.

In the J7 run, counts at cutoff were `(91, -102)`. At 0.5 seconds after
cutoff they were `(-1422, 1417)`: approximately **17.8 degrees of additional
J7 motion in the negative direction** after the command had become zero.
The capture cannot distinguish operator repositioning from passive motion.
This later movement must not be interpreted as the response to the positive
torque ramp or as failure of the command cutoff.

In the J8 run, counts changed from `(157,45)` at cutoff to approximately
`(135,44)` 0.5 seconds later, a much smaller change.

## Startup remains a separate issue

Each run has two bursts of NMT reset-node commands, one per wake sequence.
Each burst is followed within milliseconds by a CAN controller error-passive
event; warning and recovery-to-error-active events follow later. The reset
commands for nodes 1–8 are clustered within about 1 ms in these actual
captures. A missing boot message causes a single-node retry about 3 seconds
later. Each capture also has two safety-board EMCY frames with payload
`30 81 11 00 00 00 00 00` during startup.

The last controller recovery precedes the first nonzero torque by 36.37 s
for J7 and 10.41 s for J8. Startup communication errors remain unresolved,
but the captured ramps show complete position feedback and no recorded
controller error events. No further sign or transmission-matrix change is
justified by these two captures alone.
