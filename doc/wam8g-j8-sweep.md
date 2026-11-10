# WAM8 J8 position check

On opus, put the arm in its saved repeatable home with the pendant in Shift-Idle:

```sh
cd ~/lbc-tools
./run_j8_sweep_logged.sh
```

Follow the home prompt, then Shift-Activate when the program prompts. The
sequence moves J1–J4 and J6–J8 to DH zero, then commands J8 to +10°, −10°, and
back to zero. J7 remains commanded at zero during the sweep. Passive J5 stays
at its initial position and always receives zero torque. At the end, the arm
holds zero until you support it and Shift-Idle.

Watch which physical wrist axis moves and compare its direction with the DH
convention. Encoder agreement alone cannot establish the physical identity.
The zero approach takes about a minute from the configured home; the sweep
takes about 23 seconds including settling. Check clearance for the whole
home-to-zero path. The ±10° targets at J7=0 fit inside the supplied wrist limits
and 5° margin, but software does not detect self-collision or physical stops.

This separate program uses the configured position proportional and derivative
gains, with integration disabled for this run. Gravity compensation is off.
It does not change the saved home, gravity calibration, gains or drive settings.
Smooth quintic references limit commanded speed to 0.10 rad/s for the zero
approach and 0.08 rad/s for the wrist sweep. Commanded joint torque caps are
the smaller of the configured limits and `(25,20,15,15,0,0.75,0.50,0.50)` Nm.
These are software command limits, not measured torques or a guarantee of
physical holding ability.

The control graph cuts all commanded torque and latches a stop for:

- Tracking error of 3° during the zero approach, 2° during the sweep/hold,
  or 1° at passive J5. While waiting for activation, the limit is 0.5°.
- Any measured joint speed reaching 0.25 rad/s over a 20 ms window.
- Invalid/missing input, unexpected torque beyond the caps, a control-cycle
  gap over 20 ms, or a main-thread monitoring gap of 50 ms.
- Ctrl-C, Ctrl-Z, termination or terminal hangup. Ctrl-Z aborts the executable
  rather than suspending it while applying torque.
- Failure to settle within 8 seconds after a reference trajectory ends.

Each target must settle with all powered joints within 1° and below 0.03 rad/s
for 1.5 seconds before the next move. Insufficient torque or gains abort the
sequence; limits do not increase automatically. A communication timeout can
also trip the monitor watchdog. The main thread requests safety-board Idle
after a stop. Be ready to support the arm and wrist, and keep Shift-Idle ready:
an abort removes torque. Software checks depend on valid encoder feedback and
a running control process; existing pendant/drive supervision remains needed.

The wrapper records passive CAN traffic, terminal output, exact configuration
snapshots and hashes under `~/sbflash/j8-sweep-*/`. Buffered `samples.csv`
contains position, reference, speed estimate, commanded torque, tracking error,
phase (0=zero approach, 1=+10°, 2=−10°, 3=zero return, 4=final hold), and stop
reason. Normal sample recording pauses during the indefinite final hold;
monitoring continues and the eventual stop sample is retained.

Startup uses libbarrett's `pollMode()` retry (up to three reads) while the torque
gate stays zero, at the standard 50 ms activation polling interval. During
motion, a missed safety reply still triggers the 50 ms watchdog; a late reply
cannot restart a stopped test. In the 21:51 opus capture, one activation MODE
reply was absent, the next read returned ACTIVE, and all 4,792 torque frames
were zero. The earlier single-attempt startup read aborted on that lost reply.

`build/wam8_j8_sweep --help` exits before CAN access. Offline tests run with
`python3 tests/run_joint_sweep_tests.py`; they exercise the actual libbarrett
reference/PD/gate graph with simulated feedback and address/undefined-behavior
sanitizers. They do not validate physical motion, encoder scaling, or identity.

The diagnostic supports both WAM8 torque-input layouts. Older opus builds
connect the internal `externalTorques` summer to `wam.input`; this program
replaces that connection with its initially-zero guard after confirming Idle.
The occupied-input regression also checks that the prior source cannot add
torque during the sequence or after a cutoff. No library ABI change is needed.
