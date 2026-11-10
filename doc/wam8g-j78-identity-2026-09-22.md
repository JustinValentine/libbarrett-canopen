# J7/J8 identity correction on opus, 2026-09-22

The operator moved each physical joint separately with the drives in IDLE and
recorded `lbc_position_watch`. With the previous configuration:

| Physical joint moved | Reported J7 range (rad) | Reported J8 range (rad) |
| --- | --- | --- |
| J8, 10.0–29.5 s | -0.587 to +0.514, following the reversals | +0.049 to +0.289, mostly drift |
| J7, 5.5–29.5 s | -0.003 to +0.003 | -0.424 to +1.066, following the reversals |

These reciprocal observations identify a swapped J7/J8 assignment. They do
not establish the physical positive directions or explain the smaller drift
during the physical-J8 test. No ratio or direction beyond the axis exchange
is inferred from these logs.

Swap columns 7 and 8 of `j2mp`, keeping motor/node rows fixed. Motor 7's equal
entries stay the same; motor 8's last entries change from `(N1, -N1)` to
`(-N1, N1)`, where `N1 = 7.5077`. The corrected wrist equations are:

```
q6 = -m6 / 8.7456
q7 = (m7 - m8) / (2 * 7.5077)
q8 = (m7 + m8) / (2 * 7.5077)

tau_m7 = (tau7 + tau8) / (2 * 7.5077)
tau_m8 = (-tau7 + tau8) / (2 * 7.5077)
```

The library derives the position inverse and torque inverse-transpose from
`j2mp` at startup, so this configuration change corrects both paths without
rebuilding the executables. Both wrist home values are zero. The physical
joint order in DH rows, home values and gains stays J1 through J8.

`tests/run_wam8g_tests.py` checks opposite motor rotations as J7, equal motor
rotations as J8, and the corresponding single-joint torque quad signs through
the production Wam8g control graph. This is a software check, not confirmation
of physical positive direction or drive behavior.

Restart `lbc_position_watch` after deployment and verify physical J7 changes
reported J7, physical J8 changes reported J8, and each intended positive
rotation increases its matching coordinate before repeating torque tests.
