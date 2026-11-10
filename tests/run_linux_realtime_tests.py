#!/usr/bin/env python3
"""Compile and test the Linux runtime without CAN hardware or root.

--output keeps the executable for an optional `sudo PATH --probe`, which checks
real scheduler/memlock permissions with an empty 500 Hz execution manager.
The default tests intercept privileged calls, not timers or mutexes.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path)
args = parser.parse_args()
if sys.platform != 'linux':
    parser.error('Run this test on Linux (e.g. Odin); it uses timerfd and pthread PI mutexes.')
root = Path(__file__).resolve().parents[1]
sources = [
    'tests/linux_realtime_test.cpp', 'src/os.cpp',
    'src/thread/real_time_mutex.cpp', 'src/thread/null_mutex.cpp',
    'src/thread/disable_secondary_mode_warning.cpp',
    'src/systems/system.cpp', 'src/systems/execution_manager.cpp',
    'src/systems/real_time_execution_manager.cpp',
]
wrapped = ['pthread_setschedparam', 'mlockall', 'pthread_mutexattr_setprotocol',
           'timerfd_create', 'timerfd_settime', 'read']
with tempfile.TemporaryDirectory(prefix='barrett-linux-rt-') as directory:
    exe = args.output.resolve() if args.output else Path(directory) / 'linux-rt-test'
    subprocess.run(['g++', '-std=c++14', '-O1', '-g', '-Wall', '-Wextra',
                    '-DBOOST_BIND_GLOBAL_PLACEHOLDERS',
                    '-I' + str(root / 'include'),
                    *[str(root / path) for path in sources],
                    *['-Wl,--wrap=' + name for name in wrapped],
                    '-lboost_thread', '-lboost_system', '-lconfig++', '-pthread',
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], timeout=15, check=True)
