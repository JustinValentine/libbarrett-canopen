# wam8g rest pose record, 2026-09-22 13:55 (opus, can1)

Read over SDO with the safety board in Shift-Idle and every drive disabled
(statusword 0x0038). Five reads each; spans were 0-3 counts.

| node | joint | multi-turn counts 0x6064 (since the last puck reset, NOT absolute) | single-turn encoder 0x3012:1, of 4096 (absolute within one motor turn) | motor angle |
|---|---|---|---|---|
| 1 | J1 | 695 | 2208 | 194.06 deg |
| 2 | J2 (differential with 3) | -347 | 1656 | 145.55 deg |
| 3 | J3 (differential with 2) | -384 | 2941 | 258.49 deg |
| 4 | J4 | -9 | 3194 | 280.72 deg |
| 5 | J5, passive, 1:1 | 1 | 1892 | 166.29 deg = the joint's absolute angle in encoder counts |
| 6 | J6 roll (N2 = 8.7456, reversed) | 505 | 3582 | 314.82 deg |
| 7 | J7/J8 differential | 10821 | 1411 | 124.01 deg |
| 8 | J7/J8 differential | -895 | 284 | 24.96 deg |

How to use it:
- The multi-turn counts restart at 0 whenever a puck is reset (every libbarrett
  start does that), so they only describe this session.
- The single-turn values repeat every motor revolution and do not depend on
  resets. They are the P3-style zero-calibration reference: with the joint put
  back at this pose, the motor's single-turn reading must return to these
  values (modulo one turn of the motor, i.e. modulo 360/N joint degrees).
- For J5 the encoder is on the joint, so 1892 counts is its absolute angle at
  this pose; a zero-pose definition for J5 can be expressed directly as an
  offset from it.

## Later rest-to-DH-zero measurement (wrist only)

The operator started `lbc_position_watch 300` at their chosen repeatable rest
pose, then manually moved the wrist to its physical DH zero. With the existing
home `(0, -2, 0, 3.13, 0, 0, 0, 0)`, the last supplied sample at 34.0 s was
`(-0.023, -1.997, -0.008, 3.135, 0, 0, 1.620, 0.026)` rad.

Preserving J1-J5 and subtracting only the wrist readings gives the home
`(0, -2, 0, 3.13, 0, 0, -1.620, -0.026)`. The operator explicitly requested
applying this vector locally and on opus after the limit discrepancy was
identified. J7 is -92.82 degrees at rest, conflicting with the supplied constraint
`-40 + abs(J8) <= J7 <= 90 - abs(J8)` in degrees. The physical DH direction and
the limit coordinate convention must be reconciled before using this offset
for powered motion. No sign change can be established from these readings alone.

The wrist was still drifting in the final samples (J7 changed 0.018 rad and J8
0.008 rad over 28.5-34.0 s), so the final sample is an approximate reference.
The earlier single-turn encoder readings above were not recaptured for this
measurement and must not be assumed to identify this later rest pose.

The requested home vector is now saved in the local configuration and all three
opus copies (active `~/.barrett`, the main source tree, and the CAN-lock update
source tree). The home files match byte-for-byte and the active configuration
passes the offline libconfig check. The check validates stored calibration
targets against the supplied limits; it does not resolve the limit convention
or validate travel from the new home. Opus backups and the deployment manifest
are in `~/lbc-updates/home-offset-20260922-v1/`. No robot program was started.

## Home after interrupted gravitycal

The operator selected the repeatable pose reached after gravitycal stopped as
the new home. It was captured in Shift-Idle at 18:00:18 EDT. They then moved to
the prior home for a second stationary capture at 18:05:56 EDT. J1-J3 and
J5-J8 came from the motor-count differences and the configured transmission.

J4's multi-turn count difference implied 6.880583 rad, at odds with the
operator's observation that the elbow was about 5 degrees more open than the
default WAM4 home. The absolute J4 encoder changed from 1063 to 2108 counts,
equivalent to 5.10 degrees through the -18 gearing. The saved J4 home is
therefore `3.13 - 1045*2*pi/(4096*18) = 3.040944` rad. The unexplained
multi-turn difference is approximately 11 motor revolutions; it was not used
to set J4. The other seven values are unchanged from their measured deltas.

Current home: `(0.011395, -1.990117, 0.003375, 3.040944, 0.009204,
0.000000, -1.628581, -0.025796)`. Local and all three opus configuration
copies match SHA-256 `cebd1e4f391e317e179e29a8a3fb0104c3aaae8b83c95a2e7da76f33a2f44b01`.
Captures, backups, calculation, and deployment manifest are saved under
`~/lbc-updates/home-rebase-20260922-v1/` on opus. No robot motion was commanded
during the capture or configuration update.
