# DH Explorer

Generate an interactive, editable view of the robot's DH geometry:

```sh
python3 scripts/visualize_dh.py --open
```

The default is `config/wam8g.conf`. The generated file is
`build/dh-viewer/wam8g.html`. It runs offline in a browser, with no Python
packages, web server, network access, or robot connection.

Select another configuration, or a profile in a file with several robots:

```sh
python3 scripts/visualize_dh.py config/wam4.conf --open
python3 scripts/visualize_dh.py config/default.conf --profile wam8g --open
python3 scripts/visualize_dh.py config/wam8g.conf --output /tmp/wam8g.html
```

Profiles are inferred when a file contains one robot. Relative `@include`
paths resolve against the containing file. When loading through `default.conf`,
the viewer identifies the included file owning the selected kinematics block.
DH numeric parameters must be in that same file for a single-file export.

## Using the viewer

- Drag to orbit, scroll to zoom, and Shift-drag or right-drag to pan. Use the
  front, side, and top buttons for orthographic views, or **Fit model** after
  changing geometry.
- Set joint angles in degrees or radians. **Zero pose** and **Config home**
  provide presets. A missing or wrong-length home preset is disabled.
  The viewer starts with all joint angles at zero and applies the configured
  fixed `theta_pi` offsets to the geometry.
- Select a joint using its name, canvas label, or marker, then sweep it to inspect
  the bold orange rotation axis. Individual labels separate coincident joints;
  clicking overlapping markers cycles through them. Arrows have white outlines
  to keep them visible over links. An axis pointing toward the camera is shown
  as a circled dot; one pointing away is a circled cross.
  J5 is marked passive for WAM8g; its angle remains editable for inspection.
- Edit `alpha_pi`, `theta_pi`, `a`, and `d` in the DH table. Moving-joint
  `theta_pi` is a fixed angular offset added to the joint angle. It stays
  independent of the pose sliders, presets, and sweep and is saved in exports.
  The tool row's `theta_pi` edits its fixed rotation. Lengths are displayed in
  millimeters and exported in meters; alpha and theta are multiples of π.
- Enable **Original geometry** to compare edits at the same joint pose.
  **Reset edits** restores the geometry while retaining the current joint pose.
- The inspector shows the selected joint's incoming axis and origin in world
  coordinates, plus the outgoing frame's transform **to world**. The tool
  inspector shows the tool frame itself. Matrix translations are in meters.

The convention is standard DH: `Rz(θ) Tz(d) Tx(a) Rx(α)`, matching
`src/cdlbt/kinematics.c`. Joint Jᵢ rotates about zᵢ₋₁. Moving θ is
`q + theta_pi × π`, with a default offset of zero. This requires this fork's
fixed-offset support; older libbarrett builds ignored moving `theta_pi`.
Offsets do not alter encoder coordinates, home positions, or joint limits.
A zero DH row has
coincident origins but still rotates its outgoing frame by θ. Drawn links
represent DH offsets, not robot meshes or collision geometry.

## Saving edits

**Download edited .conf** saves a separate configuration. Only edited numeric
literals change (an edited optional offset is inserted if absent): comments, unrelated settings, calibration flags, relative
includes, and unedited numeric precision are preserved. Save it beside the
source `.conf` so relative includes keep resolving. The pose sliders do not
change the exported home pose. The source file is never updated by the viewer.

The preview/copy controls provide the edited kinematics block. Invalid fields
disable download, copy, and sweep until corrected; leaving an invalid field
restores its last valid value. Browser reload restores the source configuration,
so download edits before closing. Re-run the launcher to pick up disk changes.

The viewer reports the uncalibrated flag when set and coincident axes; it does
not supply measured wrist geometry or change calibration status. Gravity fits
made before changing the fixed offsets must be regenerated with the corrected
kinematics.

## Verification

The parser/export regression suite uses the Python standard library:

```sh
python3 tests/test_dh_viewer.py
```

Browser checks exercise the actual generated page, including editing, units,
presets, validation, sweep restarts, downloads, and desktop/mobile rendering.
With `--native`, the suite compiles the production C kinematics library and
compares all frame matrices and joint axes for 54 poses, including rotated and
translated bases, nonzero tool transforms, and 1–8 joint chains. This option
requires a C compiler, GSL, and libconfig development libraries.

```sh
python3 tests/test_dh_viewer.py \
  --browser '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome' \
  --native --artifacts build/dh-viewer/verification
```

On Linux, pass the installed Chrome/Chromium executable path to `--browser`.
