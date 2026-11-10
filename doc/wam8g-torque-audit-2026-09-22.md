# WAM8 torque audit, 2026-09-22

The operator reported another J4 torque trip during the first gravitycal move,
then checked the path by hand without observing contact. During this audit they
confirmed that arm motors 1–4 were **not individually calibrated after the P4s
were installed**. This is a concrete commissioning gap. Incorrect electrical
alignment is a leading explanation for poor torque production and rising
position-controller demand, but the actual alignment error has not been measured.

No motion, drive enable, calibration, configuration write, or threshold change
was performed during this audit. The read-only SDO snapshot was taken with safety
MODE=IDLE and all drives disabled. Local evidence is in
`/tmp/wam8-audit-20260922/drive-snapshot.json` (2026-09-23 00:12:20 UTC).

## Software and configuration checked

- `~/lbc-tools/build/bt-wam-gravitycal` links the private CAN-lock-fix library in
  `~/lbc-updates/can-lock-20260922-v1/install/lib`. Its stock source and binary
  hashes still match the earlier deployment record.
- The active profile is `~/.barrett/wam8g.conf`. J4 Kp is 300 Nm/rad, Ki is 0.5,
  and Kd is 2.5. Gravity compensation is disconnected for the first move; the
  stored WAM8 gravity moments are zero placeholders.
- The eight-joint position controller uses reference minus encoder position.
  Integral updates multiply by the execution period; the derivative divides by
  it. There is no observed hard-coded seven-joint indexing in the gravitycal
  allocation or first-move path.
- `moveTo()` builds a continuous trajectory from the current position/setpoint;
  it does not intentionally step immediately to the final target. The first
  transit uses the default 0.5 rad/s and 0.5 rad/s^2 path limits.
- Stock gravitycal explicitly disables host position-controller saturation.
  Its displayed/logged torque is requested torque, not measured motor torque.
- Position uses signed 32-bit little-endian motor counts. Torque uses the
  inverse transpose of `j2mp`, followed by each motor's rated-torque scaling.
  J4 is an independent -18:1 entry; wrist changes do not change its conversion.

For J4, using the verified configured rated torque of 6310 mNm:

```
motor_torque_Nm = joint_torque_Nm / -18
command_permille = round(motor_torque_Nm * 1e6 / 6310)
```

Thus a magnitude of 1200 corresponds to 7.572 Nm requested motor torque, or
136.296 Nm ideal requested joint torque. The earlier retained command 1202
corresponds to -136.523 Nm requested J4 torque under the present signs.
Neither number establishes the torque physically produced. At Kp=300, an error
of about 0.455 rad (26.1 degrees) alone produces that demand, before I/D terms.
This is a tracking-error calculation, not a claim that the arm weighs enough
to require that torque.

## Drive readback

All eight drives report firmware 4.3.2, 4096 encoder counts per motor turn,
zero target torque, zero internal torque demand, zero torque offset, and no
current drive fault. Motor bus voltage is approximately 49.9–50.4 V. Motor
nodes 1–4 and 6–8 have the intended torque-quad slots and 2 ms CST timing.
Node 5 remains passive with its command PDO disabled.

Arm nodes 1–4 retain Kt=457 mNm/Arms, rated torque=6310 mNm, 12 poles,
continuous and peak q-axis current settings of 4951 mA, and a peak-duration
setting of 196 ms. Their electrical-zero values are 278, 292, 278, 301 counts;
electrical polarity is +1 for each. Reading nonblank values does **not** verify
that they match the installed motor/encoder/phase wiring.

The applied 4951 mA q-axis ceiling corresponds to approximately 3.501 Arms and
1.600 Nm ideal motor torque with correct commutation, or 28.798 Nm ideal J4
torque. It is not an independent measurement or protection guarantee for the
shared 5 A fuse. Increasing the safety threshold cannot overcome current
saturation or incorrect commutation.

The safety board still reports TL1=1000 and TL2=1200. Firmware source explicitly
labels these provisional, unvalidated per-mille defaults. Objects 0x6072,
0x6073, 0x60E0 and 0x60E1 return "object does not exist" on these drives despite
appearing in the newer manual; do not rely on those objects as installed limits.

## Calibration finding and next step

[Barrett's P4 manual, Revision BB, page 16](https://web.barrett.com/support/Puck_Documentation/P4_CANopen_Manual_RevBB.pdf)
specifies calibration when pairing a new motor and puck, including current-sense
bias, current-sense gain factor, and encoder electrical zero/phase polarity.
Encoder zero aligns the magnetic field with the rotor and is separate from WAM
home, DH zero, and libbarrett zerocal. The manual requires the motor shaft to be
free to turn without load for that alignment; otherwise the result is invalid.
Its page 26 explicitly gives the sqrt(2) RMS-to-q-axis current conversion, and
page 35 defines target torque as thousandths of rated torque.

Incorrect electrical alignment can reduce or reverse useful torque while
substantial winding current flows. This can explain high controller demand
without a heavy arm and can contribute to electrical heating. It does not prove
the cause of the earlier blown fuse.

The installed PuckUtility includes its standard calibration sequence. Its
encoder-zero routine writes/saves electrical zero and polarity **before**
checking its motion-success criteria, so a reported failed attempt must not be
accepted as a valid calibration. A loaded arm must not be calibrated blindly
with a bulk CLI command. Establish an unloaded/free-to-align setup for the
selected motor, retain its prior calibration values, and verify the result.
Start by addressing J4, then commission the other newly paired powered motors;
exclude passive node 5. Verify torque/position direction and home afterward
before returning to automatic gravitycal.

The earlier home rebase also recorded an unexplained approximately 11-motor-turn
J4 count discrepancy. The current home uses the operator's approximately
5-degree elbow change and the matching single-turn encoder delta. This audit
does not resolve that historical discrepancy. No CAN capture exists for the
19:50 powered failure, so actual position/current evolution cannot be recovered
from the available post-stop snapshot.

## Operator calibration completed after the audit

The operator ran `cal_headless.py --can can1 --id N --i-cal 1500` separately
for arm nodes 1–4. All four runs reported success. Subsequent read-only SDO
verification on opus confirmed the following values with safety MODE=IDLE and
every arm drive disabled (statusword 0x0070):

| Motor | Previous electrical zero | New electrical zero | Electrical polarity |
|---|---:|---:|---|
| 1 | 278 | 422 | +1 → -1 |
| 2 | 292 | 109 | +1 → -1 |
| 3 | 278 | 0 | +1 → -1 |
| 4 | 301 | 498 | +1 → -1 |

Calibration current returned to 500 mA on all four; peak current remains
4951 mA. Zero counts is a valid electrical-zero value. Because phase polarity
also changed, the reported raw zero differences alone are not measurements of
the former phase-alignment error. The calibration routines report saving the
values; this verification did not reset or power-cycle the drives.

These results strengthen the incorrect-commutation hypothesis but do not yet
verify physical torque direction or magnitude under load. The next step is a
small, logged J4 torque-response check after reproducing the configured home;
software sign arrays and safety limits have not been changed.

## Physical direction check and host sign correction

The operator subsequently reported that negative torque produced physically
positive motion on all four arm joints. The logged encoder deltas were negative.
The J2 log, omitted from the pasted output, was retrieved read-only from
`~/sbflash/torque-diagnostic-20260922-203947-USUxpw/terminal.log`: it stopped at
-1.92 Nm with J2=-0.04026 rad and J3=+0.02760 rad. The J3 test likewise included
J2 motion; with the other axes unheld, these tests do not establish decoupling.

Enabled matching `canopen_position_sign` and `canopen_torque_sign` arrays:
`(-1,-1,-1,-1,1,1,1,1)`. Both paths must change together to preserve negative
feedback and the transmission's virtual-work relation. Since the arm and wrist
transmission blocks are separate, this reverses only the arm joint increments
and torque directions. The physical home reference values were retained; the
existing software-zero routine anchors the new signed readings at that home
on the next startup. DH zero/home accuracy still requires physical verification.

Applied to the local `config/wam8g.conf` and all three opus copies on
2026-09-23 00:43:58 UTC with safety MODE=IDLE and all powered drives disabled.
No robot program was running. All copies match SHA-256
`49f54fe166b1638ef8ef43666a54bfb16643906b4a6c89d8905d4c6a2b05f373`.
Backups and manifest: `~/lbc-updates/arm-signs-20260923T004358Z/`.
The existing offline libconfig/profile/pose checker passed. No motor parameters,
limits, gains, home values, or DH entries were changed and no motion was run.

With the old coordinate directions, an intended elbow-opening move could
physically close the elbow toward its stop. This is consistent with growing
position error and large requested torque, but the earlier powered failure
has no time-series capture proving contact. Re-home and repeat the small J4
direction test before returning to automatic gravity calibration.
