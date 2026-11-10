#!/usr/bin/env python3
"""Run production CANopen and ProductManager startup checks with a fake bus.

Requires Clang++, Boost headers and libconfig++. No CAN interface
is opened. ProductManager's hardware-independent startup functions are copied
verbatim; robot products, clock and transport are test doubles.
"""
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'src/products/product_manager.cpp').read_text()
signatures = [
    'int ProductManager::expectedPuckMask()',
    'int ProductManager::presentPuckMask()',
    'bool ProductManager::expectedPucksPresent()',
    'void ProductManager::wakeAllPucks()',
    'bool ProductManager::foundSafetyModule()',
    'bool ProductManager::foundWam3()',
    'bool ProductManager::foundWam4()',
    'bool ProductManager::foundWam7()',
    'bool ProductManager::foundWam8g()',
    'bool ProductManager::verifyWamPucks(',
    'Puck* ProductManager::getPuck(',
    'systems::RealTimeExecutionManager* ProductManager::getExecutionManager(',
]
functions = []
for signature in signatures:
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 0
    for match in re.finditer(r'"(?:\\.|[^"\\])*"|/\*[\s\S]*?\*/|//[^\n]*|[{}]', source[brace:]):
        if match.group() == '{':
            depth += 1
        elif match.group() == '}':
            depth -= 1
            if depth == 0:
                functions.append(source[start:brace + match.end()])
                break
    else:
        raise RuntimeError('Unclosed function: ' + signature)
if shutil.which('pkg-config'):
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'libconfig++'], text=True))
else:
    flags = ['-lconfig++']
    for prefix in (Path('/opt/homebrew'), Path('/usr/local')):
        if (prefix / 'include/libconfig.h++').exists():
            flags += ['-I' + str(prefix / 'include'), '-L' + str(prefix / 'lib'),
                      '-Wl,-rpath,' + str(prefix / 'lib')]
            break
with tempfile.TemporaryDirectory(prefix='barrett-startup-') as directory:
    tmp = Path(directory)
    (tmp / 'product_manager_under_test.inc').write_text('\n\n'.join(functions))
    exe = tmp / 'startup-test'
    subprocess.run(['clang++', '-std=c++14', '-O1', '-g',
        '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
        '-I' + str(root / 'include'), '-I' + str(tmp),
        str(root / 'tests/canopen_startup_test.cpp'),
        str(root / 'src/bus/canopen.cpp'),
        str(root / 'src/bus/communications_bus.cpp'),
        str(root / 'src/thread/real_time_mutex.cpp'),
        '-lboost_thread', '-pthread',
        *flags, '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
