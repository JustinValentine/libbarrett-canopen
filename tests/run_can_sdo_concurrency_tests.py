#!/usr/bin/env python3
"""Test real CAN locks and request/reply paths without CAN hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
prefix = next((p for p in (Path('/opt/homebrew'), Path('/usr/local'), Path('/usr'))
               if (p / 'include/boost/thread.hpp').exists()), Path('/usr'))
sources = ['tests/can_sdo_concurrency_test.cpp', 'src/bus/bus_manager.cpp',
           'src/bus/communications_bus.cpp', 'src/bus/canopen.cpp',
           'src/thread/real_time_mutex.cpp', 'src/products/puck.cpp',
           'src/products/property_list.cpp']
with tempfile.TemporaryDirectory(prefix='barrett-sdo-lock-') as directory:
    exe = Path(directory) / 'can-sdo-concurrency-test'
    subprocess.run([shutil.which('clang++') or 'g++', '-std=c++14', '-O1', '-g',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                    '-DBOOST_BIND_GLOBAL_PLACEHOLDERS',
                    '-I'+str(root/'include'), '-I'+str(prefix/'include'),
                    *[str(root/p) for p in sources], '-L'+str(prefix/'lib'),
                    '-Wl,-rpath,'+str(prefix/'lib'), '-lboost_thread', '-pthread',
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], timeout=20, check=True)
