#!/usr/bin/env python3
"""Offline DH/torque regression; optionally test an already built shared library."""
import argparse
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--library-dir', type=Path)
parser.add_argument('--include-dir', type=Path)
args = parser.parse_args()
prefix = next(p for p in map(Path, ('/opt/homebrew', '/usr/local', '/usr'))
              if (p/'include/libconfig.h').exists())
with tempfile.TemporaryDirectory(prefix='dh-offset-test-') as temp:
    binary = Path(temp)/'test'
    command = ['cc', '-std=c99', '-O1', '-g', '-fsanitize=address,undefined',
               '-fno-sanitize-recover=all', '-I'+str(args.include_dir or root/'include'),
               '-I'+str(prefix/'include'), str(root/'tests/dh_offset_test.c')]
    if args.library_dir:
        command += ['-L'+str(args.library_dir), '-Wl,-rpath,'+str(args.library_dir), '-lbarrett']
    else:
        command += [str(root/'src/cdlbt'/name) for name in ('kinematics.c', 'calgrav.c', 'gsl.c')]
    command += ['-L'+str(prefix/'lib'), '-Wl,-rpath,'+str(prefix/'lib'),
                '-lconfig', '-lgsl', '-lgslcblas', '-lm', '-o', str(binary)]
    subprocess.run(command, check=True)
    subprocess.run([str(binary)], check=True)
