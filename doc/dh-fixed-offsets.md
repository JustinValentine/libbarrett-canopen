# Fixed angular offsets in DH kinematics

Moving-row `theta_pi` is a fixed offset in multiples of pi. Runtime kinematics
and the DH viewer both use `theta = joint_position + theta_pi * pi`.
An omitted offset is zero. The toolplate's fixed rotation is unchanged.

The WAM8 configuration's offsets are J4=-0.547 pi, J5=+0.547 pi,
J6=+0.5 pi and J7=+0.5 pi. They define geometry at joint zero, including
the passive J5 row. They do not change encoder readings, transmission signs,
saved home coordinates, commanded joint poses or joint travel limits.

Older versions of this fork ignored moving `theta_pi`; the old viewer used it
as the initial preview joint pose. The viewer now starts with zero joint
angles and retains all fixed offsets. Editing an offset changes the exported
geometry; moving a slider changes only the preview joint angle.

Gravity fits made with the previous geometry must be regenerated. On opus,
the old fit is backed up with the deployment, and the active gravity file is
replaced with zero first moments pending a new stock gravitycal run. The zero
placeholder supplies no arm support. Do not use ex04 to support the arm until
the new calibration has been checked with the bounded gravity-ramp test.

The implementation stores offsets privately without changing public C/C++
struct layouts. Regression checks cover 1-8 joints, repeated evaluations,
position Jacobians, gravity-transform equivalence, omitted and invalid offsets,
and viewer/C-library agreement at zero and nonzero joint positions.

```sh
python3 tests/run_dh_offset_tests.py
python3 tests/test_dh_viewer.py --browser /path/to/chrome --native
```
