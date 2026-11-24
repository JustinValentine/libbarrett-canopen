# Change Log
All notable changes to this project will be documented in this file.
This project adheres to [Semantic Versioning](http://semver.org/). 

## [Unreleased]

- CANopen support for WAMs with P4 pucks and the CANopen safety board, replacing BarrettCAN (see doc/canopen.md)
- The WAM zero is kept across program runs, and zero compensation uses the P4's single-turn encoder
- wam8g: a 4-DOF WAM with a passive joint and a three-motor wrist (see doc/wam8g.md)
- Optional fixed theta offsets in DH kinematics
- Linux: SCHED_FIFO control thread, priority-inheritance mutexes and a monotonic clock
- Optional torque timing for P4 pucks: `bus.torque_on_arrival`, `bus.torque_interpolation_ms` (see doc/canopen.md)
- Builds against stock libconfig
- Added a DH viewer (scripts/visualize_dh.py)
- Added wamudpd script that makes PCs findable by the wamdiscover script.
- Updated wamudpd script to run using python3

## [dev-3.0.1]

- Increased enumeration timeout from 1 ms to 5 ms

## [dev-3.0.0]

- Python3 support
- Clang support
- Updated dependencies (Boost, Eigen, Freeglut)
- Allowed WAM loop to run while reading BHand sensors
- Converted ex10 haptics visualization from vpython to OpenGL
- Based WAM loop on a (100x) more accurate timer_fd instead of sleep()
- More robust installation script
- Improved mutex handling for non-rt operation
- Offered Top10 tactile values from BHand for higher update rates

[dev-3.0.1]: https://git.barrett.com/software/libbarrett/-/tags/dev-3.0.1
[dev-3.0.0]: https://git.barrett.com/software/libbarrett/-/tags/dev-3.0.0

