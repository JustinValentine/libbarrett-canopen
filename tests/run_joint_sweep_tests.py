#!/usr/bin/env python3
"""Build and test the joint reference, PD and torque gate with simulated time; no CAN access."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
prefix = next(p for p in (Path('/opt/homebrew'), Path('/usr/local'), Path('/usr'))
              if (p/'include/libconfig.h++').exists())
sources = ['tests/joint_sweep_test.cpp', 'src/systems/system.cpp',
           'src/systems/execution_manager.cpp', 'src/thread/real_time_mutex.cpp',
           'src/thread/null_mutex.cpp', 'src/thread/disable_secondary_mode_warning.cpp']
with tempfile.TemporaryDirectory(prefix='joint-sweep-test-') as directory:
    binary = Path(directory)/'test'
    subprocess.run(['clang++', '-std=c++14', '-O1', '-g',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                    '-DBOOST_BIND_GLOBAL_PLACEHOLDERS',
                    '-I'+str(root/'include'), '-I'+str(prefix/'include'),
                    '-I'+str(prefix/'include/eigen3'),
                    *[str(root/s) for s in sources],
                    '-L'+str(prefix/'lib'), '-Wl,-rpath,'+str(prefix/'lib'),
                    '-lconfig++', '-lconfig', '-lgsl', '-lgslcblas',
                    '-lboost_thread', '-pthread', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
