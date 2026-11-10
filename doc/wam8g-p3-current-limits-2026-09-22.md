# WAM8 arm current-limit update, 2026-09-22

Applied on opus after the operator reported a blown 5 A safety-board fuse
during gravitycal's first move and requested the existing P3 CSV limits.

Reference: [Barrett P335 Allied 4DOF 48 V configuration](https://web.barrett.com/support/WAM_ControlLibrary/PuckConfiguration/Configs/P335-Allied-4DOF-48V-2.5.4.csv).
SHA-256: `9e4c0f1d8058f2ee335d532f4a2645286931b9213c00d657f283836be46abad3`.
The filename says 2.5.4; its internal version field says 2.5.3.

The CSV specifies motor Kt = 457 mNm/A and application torque = 1600 mNm.
The live J1-J4 motor parameters match its 12 poles, Kt, 4.50 ohm terminal
resistance, 4.06 mH inductance, and 6310 mNm peak-rated torque.

Transferred the application torque ceiling to the P4 phase-peak/dq current
convention: `floor(sqrt(2) * 1000 * 1600 / 457) = 4951 mA`, approximately
3.501 A phase RMS. This matches the existing continuous-current field.
The convention is also used by `puckutility/puckutilityapp.py` when displaying
current and its limit lines in RMS units.

| Pucks | Object | Before | After |
|---|---|---:|---:|
| 1-4 | `0x3011:09`, peak current | 19526 mA | 4951 mA |

Continuous current remains 4951 mA; peak-duration parameter remains 196 ms.
Each changed entry was saved individually with `0x1010:04 = 0x00301109`.
All four saves were acknowledged. Readback matched; motor constants,
commutation offsets/polarities, sensor scale, and rated-torque scale were
checked unchanged. Safety MODE was E-stop, bus voltage was zero, and drives
were disabled throughout. No reset, activation, or motion command was sent.
Persistence across a power cycle and physical current limiting have not been tested.

The original CSVs, update script, and complete before/after report are on opus:
`/home/opus/lbc-updates/p3-current-limits-20260922-v1/`.
Applied report: `applied-20260922T233729096926Z.json`.
Local copies are in `/tmp/opus-p3-current-limits-20260922/`.

J5 remains passive. The operator identified J6-J8 as ILM50 wrist motors and
requested retaining their existing limits. Those settings remain 7314 mA
continuous, 23471 mA peak, and 140 ms peak duration, in P4 current units
(approximately 5.17 A RMS continuous and 16.60 A RMS peak). Their configured
20 poles and Kt = 58 mNm/A differ from the standard P3 wrist profiles.
The lower wrist torque rating does not imply lower current; the retained
ILM50 settings have not been independently validated against a motor datasheet.

The derived ideal J4 torque ceiling is approximately 28.8 Nm at 18:1.
These per-motor limits do not establish protection of the shared 5 A fuse.
Gravitycal still disables the host position-controller torque caps and uses
the default speed for transit moves. Its logged requested torque is not
proof of delivered torque if the drive current limit is reached.
