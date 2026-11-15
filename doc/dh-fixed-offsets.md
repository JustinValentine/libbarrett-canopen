# Fixed angular offsets in DH kinematics

A row of `kinematics.moving` may carry `theta_pi`, a fixed offset in multiples
of pi: the link's DH angle is `theta = joint_position + theta_pi * pi`. An
omitted offset is zero, so existing configurations are unaffected. The
toolplate's `theta_pi` is its fixed rotation, as before.

Offsets define the geometry at joint zero. They do not change encoder
readings, transmission signs, home positions or commanded joint positions.
A gravity calibration made before an offset was changed must be redone.

The offsets are stored outside `struct bt_kinematics`, so its public layout
is unchanged.

```sh
python3 tests/run_dh_offset_tests.py
```
