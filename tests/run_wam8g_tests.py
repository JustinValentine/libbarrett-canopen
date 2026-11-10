#!/usr/bin/env python3
"""Build the real Wam8g control graph against a simulated CAN bus, with sanitizers.

Requires Clang, Eigen, Boost, GSL and libconfig++; no CAN hardware or Linux RT
timer is used. Puck, MotorPuck, LowLevelWam, Wam8g and all control/math code are
compiled from production sources. Also syntax-check ProductManager and examples.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
prefix = next((p for p in (Path('/opt/homebrew'), Path('/usr/local'), Path('/usr'))
               if (p / 'include/libconfig.h++').exists()), Path('/usr'))
sources = [
    'tests/wam8g_test.cpp', 'src/systems/wam8g.cpp',
    'src/systems/system.cpp', 'src/systems/execution_manager.cpp', 'src/systems/ramp.cpp',
    'src/thread/real_time_mutex.cpp', 'src/thread/null_mutex.cpp', 'src/thread/disable_secondary_mode_warning.cpp',
    'src/bus/canopen.cpp', 'src/bus/communications_bus.cpp',
    'src/products/puck.cpp', 'src/products/motor_puck.cpp',
    'src/products/multi_puck_product.cpp', 'src/products/puck_group.cpp',
    'src/products/property_list.cpp', 'src/products/safety_module.cpp',
    'src/math/trapezoidal_velocity_profile.cpp',
    'src/cdlbt/kinematics.c', 'src/cdlbt/calgrav.c', 'src/cdlbt/gsl.c',
    'src/cdlbt/spline.c', 'src/cdlbt/profile.c', 'src/cdlbt/interp.c',
]
with tempfile.TemporaryDirectory(prefix='barrett-wam8g-') as directory:
    tmp = Path(directory)
    (tmp / 'barrett').mkdir()
    (tmp / 'barrett/config.h').write_text(
        '#include <string>\nnamespace barrett { inline std::string EtcPathRelative('
        'const std::string& p) { return "/etc/barrett/"+p; } }\n')
    common = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
              '-DBOOST_BIND_GLOBAL_PLACEHOLDERS', '-Wno-deprecated-declarations',
              '-I'+str(tmp), '-I'+str(root/'include'), '-I'+str(prefix/'include'),
              '-I'+str(prefix/'include/eigen3')]

    def compile_source(source):
        obj = tmp / (source.replace('/', '_')+'.o')
        compiler = ['clang', '-std=c99'] if source.endswith('.c') else ['clang++', '-std=c++14']
        command = [*compiler, *common, '-c', str(root/source), '-o', str(obj)]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(source+'\n'+result.stderr)
        return obj

    with ThreadPoolExecutor(max_workers=4) as pool:
        objects = list(pool.map(compile_source, sources))
    exe = tmp/'wam8g-test'
    subprocess.run(['clang++', *common, *map(str, objects), '-L'+str(prefix/'lib'),
                    '-Wl,-rpath,'+str(prefix/'lib'), '-lconfig++', '-lconfig',
                    '-lgsl', '-lgslcblas', '-lboost_thread', '-lboost_chrono',
                    '-pthread', '-o', str(exe)], check=True)
    subprocess.run([str(exe), str(root/'config')], check=True)
    for source in ['src/products/product_manager.cpp', 'examples/ex01_initialize_wam.cpp',
                   'examples/wam8g_state.cpp', 'tests/lbc_torque_sign.cpp']:
        subprocess.run(['clang++', '-std=c++14', *common, '-fsyntax-only', str(root/source)], check=True)
