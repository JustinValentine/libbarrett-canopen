## 1.3 Software and Firmware Setup

Earlier WAM systems used the proprietary BarrettCAN protocol between the safety board and the motor controllers. The P4-37 pucks in this arm communicate using the industry-standard CANopen protocol (CiA 301/402, cyclic synchronous torque at 500 Hz) instead. Three pieces of software must match each other:

| Component | Validated version | Notes |
|---|---|---|
| Motor controllers (P4-37) | Puck firmware 4.3.2 (`Puck4—mk1-v4.3.2.ebin`) | Configured from Barrett's `P437-4dof` CSV, then calibrated per puck (Section 1.3.4) |
| Safety board | Firmware revision 208 | Revision 206 or later refuses to activate until the host has commissioned the puck set over CAN, so it cannot be used with an older host build |
| Control PC | libbarrett, CANopen branch `[BRANCH-NAME]` | Standard libbarrett releases speak BarrettCAN and will not find P4-37 controllers |

Note: not tested with the embedded PC/104; all validation was performed with an external control PC. The original PC/104 software stack is BarrettCAN-based and is not compatible.

### 1.3.1 Installing libbarrett

libbarrett is the real-time C++ library that runs on the control PC and provides all robot control, including communication with the safety board and the motor controllers.

Requirements:

- Control PC running Ubuntu 20.04 LTS or later, 64-bit (validated on 20.04). A low-latency kernel is installed by the dependency script.
- A CAN adapter supported by SocketCAN, connected to the WAM's external CAN port. The PEAK PCAN-USB was used for validation.
- The control loop period is fixed at 2 ms (500 Hz) by the puck configuration; the library refuses any other execution-manager period.

Installation:

```
git clone [REPO-URL] libbarrett
cd libbarrett
git checkout [BRANCH-NAME]
scripts/install_dependencies.sh
cmake .
make -j$(nproc)
sudo make install            # installs the library and the /etc/barrett configuration files
sudo reboot
```

Configuration files: libbarrett reads `/etc/barrett/default.conf`. If a `~/.barrett/` directory exists it takes precedence, and the environment variable `BARRETT_CONFIG_FILE` overrides both. Keep one copy per robot; the arm-specific values below live in that copy, not in the library.

CAN interface configuration: the WAM bus runs at 1 Mbit/s. libbarrett ships a setup script and a udev rule that configure the interface automatically whenever the CAN adapter is present. Install them once:

```
sudo cp scripts/reset_can.sh /bin/
sudo chmod +x /bin/reset_can.sh
sudo cp scripts/60-can.rules /etc/udev/rules.d/
sudo udevadm control --reload
```

Re-plug the CAN adapter (or reboot) and confirm the interface is up:

```
ip -br link show can0        # should show can0 UP
```

If you use a different SocketCAN adapter than the PCAN-USB, run `scripts/reset_can0.sh` manually after each boot instead, or adapt the udev rule to your adapter's USB vendor ID. Then set `bus.port` in the active `default.conf` to the SocketCAN interface number (`can0` → `port = 0`). A mismatched port number is the most common cause of "WAM not found" on a fresh install.

If you also use Barrett's Puck Utility or other python-can tools on the same interface, raise the transmit queue, otherwise a bus scan overflows it: `sudo ip link set can0 txqueuelen 1000`.

Arm-specific settings in the robot's `default.conf` / `wam4.conf`:

- `bus.expected_puck_mask` (optional): the physical set of pucks, one bit per node (`0x001E` for a 4-DOF arm). With it declared, a missing puck stops startup instead of the library identifying a smaller arm. Without it the arm is identified from the pucks that answer.
- `canopen_position_sign` and `canopen_torque_sign` in the arm's `low_level` block: one +1/−1 per motor for the direction of the reported encoder counts and of the commanded torque relative to the WAM conventions. They are measured per arm after the pucks' commutation calibration (Section 1.3.4). Both entries must agree for each motor; the library refuses a configuration where they differ.
- Gravity calibration data (`calibration_data/wam4/gravitycal.conf`) and zero-calibration data are per arm, exactly as for a BarrettCAN WAM. Re-run `bt-wam-gravitycal` after the pucks are calibrated.
- Joint position-control gains may need retuning: the CANopen torque path applies a command one control cycle later than the BarrettCAN path, and the stock gains felt stiff on the validation arm.

Mounting orientation: the default configuration assumes a standard (horizontal) mount. If your WAM is mounted vertically, follow Barrett's vertical-mount configuration instructions before calibrating:
https://web.barrett.com/support/WAM_Documentation/Libbarrett_WAM_Vertical_Mount_Instructions.pdf

### 1.3.2 Updating the Safety Board Firmware

The safety board's firmware is updated through its serial port. The board's processor contains a permanent, factory-programmed boot loader that accepts new firmware over serial, and the update tool only ever writes the application flash sectors (B–E). It refuses to touch sector A, where the boot loader lives, so a failed or interrupted update can always be retried; this procedure cannot make the board unrecoverable.

The BOOTLOAD switch tells the processor, at the moment power is applied, to wait for a serial update instead of starting the robot firmware. The switch must then be turned off again before flashing, because while it is on it also holds the board's serial transmit line low and the board cannot answer.

You will need: the firmware update kit (`sci_flash_app.py`, the flash kernel `F2812SerialFlash.out`, and the firmware image `<firmware>.out`), a Linux PC with Python 3 (no extra packages; the tool drives the port directly), and a USB-to-RS-232 adapter connected to the legacy serial jack on the front of the safety board (pin 1 GND, pin 2 board RX, pin 4 board TX). The adapter appears as `/dev/ttyUSB0`; the user must be in the `dialout` group or use `sudo`.

Before updating, record the board's stored settings; the update does not touch them, but the record is cheap insurance:

```
python3 sb_console_dump.py /dev/ttyUSB0 > sb-settings-before.txt
```

Procedure:

1. Press the pendant emergency stop.
2. Power the WAM completely off.
3. Set the BOOTLOAD switch ON, power the WAM on, then set the switch OFF. The pendant stays dark; this is normal. This state is single-use: if the update fails for any reason, repeat from step 2.
4. On the PC, verify what is on the board and flash the new image in one session:

   ```
   python3 sci_flash_app.py /dev/ttyUSB0 F2812SerialFlash.out --check <current>.out --flash <new>.out
   ```

   `--check` compares the flash against the image you believe is installed and reports the match (this is the backup record; the kernel cannot read flash back, so a matching image file is the backup). The flash step then erases sectors B–E, programs the new image and verifies every section. The whole update takes about two minutes and ends with "APPLICATION FLASHED".
5. Power cycle the WAM with the switch off. The board boots the new firmware and the pendant lights normally.

Velocity and torque limits: on the CANopen safety board the four limits (VL1/VL2 warning and fault velocity, TL1/TL2 warning and fault torque) are held in RAM and return to their built-in defaults at every board power-up: 0.5 and 1.0 rad/s per joint, and 1000 and 1200 per-mille of the pucks' rated torque. The CANopen host software does not rewrite them (unlike the BarrettCAN host, which loaded them at every start). The velocity defaults are low enough to fault the standard calibration moves, so after every safety-board power-up set the values your application needs, either from the serial console:

```
python3 sb_console_cmd.py /dev/ttyUSB0 'SET VL1 6144' 'SET VL2 8192' 'GET VL1' 'GET VL2'
```

(units: rad/s × 4096, so 6144 = 1.5 rad/s and 8192 = 2.0 rad/s) or over CAN through the board's objects 0x202D–0x2030. They take effect immediately.

### 1.3.3 Safety Board Settings to Verify

These are stored in the board's EEPROM and survive updates, but they must be right for the arm. Check them with the serial console (`GET <name>`, `SET <name> <value>`, `SAVE <name>`) after installing a board or replacing the power supply. Voltage limits are recomputed when the board goes from E-stop to Idle, so press E-stop and Shift-Idle after changing them.

| Setting | Value | Why |
|---|---|---|
| `VNOM` | 48 | nominal bus voltage, V |
| `X1` | Safety voltage ADC counts at `VNOM` | Safety protection uses filtered `AUX_ANIN_1` (ADC A7); read raw samples using `GET ANA1` or SDO `0x2013:00`. With an independently measured voltage corresponding to that sensor, calculate `X1 = mean(ANA1) × VNOM / measured volts`. `GET VBUS` reads a different channel (`BUS_SENSE`, ADC A4) with a fixed conversion and cannot be used to calculate X1. Calibration is specific to each board. |
| `VOLTH1` | 55 | regen shunt switch-on voltage, V. Above it the pendant shows H while the board dissipates regenerated energy |
| `VOLTH2` | 60 | over-voltage E-stop, V |

The shunt matters more than on a BarrettCAN WAM: the P4-37 drives fault (code 0x3210) if the bus exceeds their own over-voltage limit, and a downward move regenerates a few watts that nothing else absorbs. With the shunt off, every gravity-assisted move trips all four drives and the arm goes limp; the board then shows the number of the lowest faulted puck with the critical light.

### 1.3.4 Puck Configuration and Calibration

Each P4-37 must be configured and calibrated before the arm is usable; a puck straight from Barrett's CSV will run, but with reversed or weakened torque. In order:

1. Load Barrett's configuration CSV with the Puck Utility, then correct two values for a 48 V arm and save them: nominal bus voltage (`0x3001:9`) = 480 (units of 0.1 V) and bus over-voltage limit (`0x2384:6`) = 580.
2. Calibrate the commutation zero of every puck with the Puck Utility's encoder-zero routine, with the arm resting near its zero pose and at 1.5 A calibration current (`cal_headless.py --can can0 --id 1 2 3 4 --i-cal 1500`). At the CSV's 500 mA the routine warns that the rotor did not follow the sweep. Repeat once; the stored zeros should agree within about 20 counts.
3. Measure the per-motor signs with the two bench tools that ship with the CANopen branch (`lbc_position_watch` with the drives disabled, `lbc_torque_sign` with a small ramped torque) and enter them in the arm's `low_level` block as `canopen_position_sign` and `canopen_torque_sign`. Do this only after step 2: a commutation error above 90 electrical degrees reverses the torque and hides the real sign.
4. Zero the arm (home position prompt at every start; the pucks' position counters restart at zero on each reset, so the library keeps a software zero), then run `bt-wam-gravitycal`.

Puck fault codes seen on the pendant's critical light with a digit: the digit is the lowest puck that dropped out of Operation Enabled. Read the puck's error code over CAN (object 0x603F): 0x3210 is bus over-voltage (see Section 1.3.3), 0x3220 bus under-voltage (normal after an E-stop cuts the bus).
