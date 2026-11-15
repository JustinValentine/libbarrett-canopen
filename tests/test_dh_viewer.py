#!/usr/bin/env python3
"""Offline viewer regressions. Python-only by default; optional Chrome/C checks.

python3 tests/test_dh_viewer.py
python3 tests/test_dh_viewer.py --browser '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome' --native
"""
import argparse
import json
import math
import os
from pathlib import Path
import random
import re
import signal
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from visualize_dh import ConfigError, ConfigParser, build_html, load_model

FIXTURE = '''# Geometry 🦾; placeholder-looking text /*__APP__*/ stays data.
# </script><script>throw new Error("config was executed")</script>
robot: {
  kinematics_calibrated = false;
  low_level: { home = (0.25); };
  kinematics: {
    moving = ({ alpha_pi = -0.5; a = 0.10000000000000001; d = 0.2; theta_pi = 9; label = "keep"; });
    toolplate = { alpha_pi = 0.5; theta_pi = 0; a = 0; d = 0.03; };
    annotation = "keep this too";
  };
  untouched = 42;
};
'''


class ParserTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='dh-parser-')
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'robot.conf'
        self.path.write_text(FIXTURE)

    def test_repository_profiles(self):
        for profile in ('wam3', 'wam4', 'wam7w', 'wam7g', 'wam8g'):
            with self.subTest(profile=profile):
                model = load_model(ROOT / 'config' / (profile + '.conf'))
                self.assertEqual(model['profile'], profile)
                self.assertEqual(len(model['fields']), 4 * len(model['moving']) + 4)

    def test_parent_include_resolves_source_and_home(self):
        model = load_model(ROOT / 'config/default.conf', 'wam8g')
        self.assertEqual(model['path'], str(ROOT / 'config/wam8g.conf'))
        expected = ConfigParser(ROOT / 'config/calibration_data/wam8g/zerocal.conf').parse()['home']
        self.assertEqual(model['home'], expected)
        self.assertEqual(model['passive'], 4)

    def test_ambiguous_profile_is_actionable(self):
        with self.assertRaisesRegex(ConfigError, 'Choose --profile from:.*wam8g'):
            load_model(ROOT / 'config/default.conf')

    def test_unicode_spans_use_javascript_offsets(self):
        model = load_model(self.path)
        source = model['source'].encode('utf-16-le')
        for field in model['fields']:
            if field.get('missing'):
                continue
            start, end = field['span']
            literal = source[start * 2:end * 2].decode('utf-16-le')
            row = model['moving'][0] if field['row'] == 0 else model['tool']
            self.assertEqual(float(literal), row[field['key']])

    def test_windows_line_endings_are_preserved(self):
        source = FIXTURE.replace('\n', '\r\n')
        self.path.write_bytes(source.encode('utf-8'))
        self.assertEqual(load_model(self.path)['source'], source)

    def test_model_is_embedded_only_as_data(self):
        model = load_model(self.path)
        html = build_html(model)
        data = re.search(r'<script id="model-data" type="application/json">(.*?)</script>', html, re.S)[1]
        self.assertEqual(json.loads(data), model)
        self.assertNotIn('<script>throw', html)
        self.assertIn('/*__APP__*/', json.loads(data)['source'])
        self.assertEqual(model['moving'][0]['theta_pi'], 9)
        self.assertTrue(any(f['key'] == 'theta_pi' and f['row'] == 0 for f in model['fields']))

    def test_missing_theta_has_an_insertion_point_inside_its_row(self):
        self.path.write_text(FIXTURE.replace('theta_pi = 9;', ''))
        model = load_model(self.path)
        field = next(f for f in model['fields'] if f['row'] == 0 and f['key'] == 'theta_pi')
        self.assertTrue(field['missing'])
        self.assertEqual(model['moving'][0]['theta_pi'], 0)
        offset = field['span'][0]
        source = model['source'].encode('utf-16-le')
        self.assertEqual(source[offset*2:(offset+1)*2].decode('utf-16-le'), '}')

    def test_include_cycles_and_duplicates(self):
        self.path.write_text('@include "robot.conf"')
        with self.assertRaisesRegex(ConfigError, 'Include cycle'):
            ConfigParser(self.path).parse()
        self.path.write_text('x = 1; x = 2;')
        with self.assertRaisesRegex(ConfigError, 'Duplicate setting'):
            ConfigParser(self.path).parse()

    def test_truncated_or_invalid_syntax_has_no_traceback(self):
        for source in ('x =', 'x = { y =', 'x = (1,', 'x = "unterminated', 'x = @', r'x = "\q"'):
            with self.subTest(source=source):
                self.path.write_text(source)
                with self.assertRaises(ConfigError):
                    ConfigParser(self.path).parse()

    def test_bad_dh_and_home_values(self):
        for before, after in [('a = 0.10000000000000001', 'a = true'),
                              ('d = 0.2', 'd = 1e999'),
                              ('home = (0.25)', 'home = 1'),
                              ('{ alpha_pi = -0.5; a = 0.10000000000000001; d = 0.2; theta_pi = 9; label = "keep"; }', '7')]:
            with self.subTest(after=after):
                self.path.write_text(FIXTURE.replace(before, after))
                with self.assertRaises(ConfigError):
                    load_model(self.path)

    def test_mismatched_home_disables_preset_but_keeps_geometry(self):
        self.path.write_text(FIXTURE.replace('home = (0.25)', 'home = (1, 2)'))
        model = load_model(self.path)
        self.assertIsNone(model['home'])
        self.assertIn('home preset is disabled', model['warnings'][0])

    def test_invalid_world_transforms(self):
        for world in ('7', '(1,2,3,4)', '((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,0))',
                      '((2,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))',
                      '((-1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))'):
            with self.subTest(world=world):
                self.path.write_text(FIXTURE.replace('kinematics: {', f'kinematics: {{ world_to_base = {world};'))
                with self.assertRaises(ConfigError):
                    load_model(self.path)

    def test_cli_cannot_overwrite_config(self):
        proc = subprocess.run([sys.executable, str(ROOT / 'scripts/visualize_dh.py'), str(self.path),
                               '--output', str(self.path)], capture_output=True, text=True)
        self.assertNotEqual(proc.returncode, 0)
        self.assertNotIn('Traceback', proc.stderr)
        self.assertEqual(self.path.read_text(), FIXTURE)


def config_value(value):
    if isinstance(value, dict):
        return '{' + ''.join(f'{key} = {config_value(v)};' for key, v in value.items()) + '}'
    if isinstance(value, list):
        return '(' + ','.join(map(config_value, value)) + ')'
    return json.dumps(value)


def native_cases(directory):
    prefix = next((p for p in map(Path, ['/opt/homebrew', '/usr/local', '/usr'])
                   if (p / 'include/libconfig.h').exists()), Path('/usr'))
    executable = directory / 'dh-reference'
    subprocess.run(['cc', '-std=c99', '-O2', '-I'+str(ROOT / 'include'), '-I'+str(prefix / 'include'),
                    str(ROOT / 'tests/dh_kinematics_reference.c'), str(ROOT / 'src/cdlbt/kinematics.c'),
                    str(ROOT / 'src/cdlbt/gsl.c'), '-L'+str(prefix / 'lib'), '-Wl,-rpath,'+str(prefix / 'lib'),
                    '-lconfig', '-lgsl', '-lgslcblas', '-lm', '-o', str(executable)], check=True)
    rng = random.Random(734)
    cases = []
    models = [load_model(ROOT / 'config' / (p + '.conf')) for p in ('wam3', 'wam4', 'wam7w', 'wam7g', 'wam8g')]
    for n in (1, 2, 4, 8):
        row = lambda: dict(alpha_pi=rng.uniform(-1, 1), a=rng.uniform(-.2, .2), d=rng.uniform(-.4, .4))
        models.append(dict(moving=[dict(**row(), theta_pi=rng.uniform(-.8, .8)) for _ in range(n)],
                           tool=dict(**row(), theta_pi=.37),
                           world=[[0,-1,0,.3],[1,0,0,-.4],[0,0,1,.7],[0,0,0,1]]))
    config = directory / 'native.conf'
    for model in models:
        kin = dict(moving=model['moving'], toolplate=model['tool'], world_to_base=model['world'])
        config.write_text('robot = ' + config_value(dict(kinematics=kin)) + ';')
        poses = [[0] * len(model['moving'])] + [[rng.uniform(-math.pi, math.pi) for _ in model['moving']] for _ in range(5)]
        for q in poses:
            output = subprocess.check_output([str(executable), str(config), *map(str, q)], text=True)
            cases.append(dict(model={key: model[key] for key in ('moving', 'tool', 'world')}, q=q, expected=json.loads(output)))
    return cases


def browser_tests(browser, native, artifacts):
    with tempfile.TemporaryDirectory(prefix='dh-browser-') as tmp:
        directory = Path(tmp)
        cases = native_cases(directory) if native else []
        fixture = directory / 'robot.conf'
        fixture.write_text(FIXTURE)
        models = [('wam8g', load_model(ROOT / 'config/wam8g.conf'), 1440),
                  ('single', load_model(fixture), 1440),
                  # Chrome desktop windows have a minimum viewport width. Use
                  # 600px to exercise the mobile layout without clipping capture.
                  ('mobile', load_model(ROOT / 'config/wam4.conf'), 600)]
        tests = (ROOT / 'tests/dh_viewer_browser.js').read_text()
        if artifacts:
            artifacts.mkdir(parents=True, exist_ok=True)
        for name, model, width in models:
            # A controlled animation clock avoids compositor/virtual-time races
            # in headless Chrome, and lets sweep lifecycle checks be deterministic.
            prelude = '''<script>
window.testErrors=[];addEventListener("error",e=>testErrors.push(e.message));
window.nativeAnimationFrame=window.requestAnimationFrame.bind(window);
window.testFrames=[];
window.requestAnimationFrame=fn=>{testFrames.push(fn);return testFrames.length;};
window.flushTestFrame=()=>testFrames.splice(0).forEach(fn=>fn(performance.now()));
</script>'''
            html = build_html(model).replace('<head>', '<head>'+prelude, 1)
            html = html.replace('</body>', '<script>window.nativeCases='+json.dumps(cases)+';</script><script>'+tests+'</script></body>')
            page = directory / (name + '.html')
            page.write_text(html)
            command = [browser, '--headless', '--disable-gpu', '--no-first-run', '--no-default-browser-check',
                       '--disable-background-networking', '--disable-component-update',
                       '--user-data-dir='+str(directory / ('chrome-'+name)), '--dump-dom', '--virtual-time-budget=3000', '--timeout=10000',
                       f'--window-size={width},1050']
            if artifacts:
                command.append('--screenshot='+str((artifacts / (name + '.png')).resolve()))
            # Chrome helpers can inherit pipes after the main process exits.
            # Use files and a separate process group so the runner always exits.
            with tempfile.TemporaryFile() as out, tempfile.TemporaryFile() as err:
                proc = subprocess.Popen([*command, page.as_uri()], stdout=out, stderr=err, start_new_session=True)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
                finally:
                    try:
                        os.killpg(proc.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                out.seek(0); err.seek(0)
                stdout, stderr = out.read().decode('utf-8'), err.read().decode('utf-8')
            match = re.search(r'<script id="test-report" type="application/json">(.*?)</script>', stdout, re.S)
            if not match:
                raise RuntimeError(f'{name}: Chrome did not finish tests\n{stderr[-2000:]}\n{stdout[-2500:]}')
            report = json.loads(match[1])
            if report['failures']:
                raise AssertionError(name + ':\n' + '\n'.join(report['failures']))
            # Reload the actual browser export with includes resolved as on disk.
            exported = directory / (name + '.conf')
            text = report['exported']
            text = re.sub(r'@include "([^"]+)"', lambda m: '@include '+json.dumps(str(Path(model['path']).parent / m[1])), text)
            exported.write_text(text)
            reloaded = load_model(exported)
            if abs(reloaded['moving'][0]['a'] - .123456789012345) > 1e-15:
                raise AssertionError('Edited config did not round-trip')
            offset_export = report['offsetExport']
            offset_export = re.sub(r'@include "([^"]+)"', lambda m: '@include '+json.dumps(str(Path(model['path']).parent / m[1])), offset_export)
            exported.write_text(offset_export)
            if load_model(exported)['moving'][0]['theta_pi'] != -.5:
                raise AssertionError('Fixed offset did not round-trip')
            print(f'{name}: {len(report["checks"])} browser checks passed; {len(cases)} C-library poses matched', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--browser', help='Chrome or Chromium executable for UI tests')
    ap.add_argument('--native', action='store_true', help='Compare to compiled libbarrett C kinematics (requires GSL/libconfig)')
    ap.add_argument('--artifacts', type=Path, help='Save browser screenshots here')
    args = ap.parse_args()
    if args.native and not args.browser:
        ap.error('--native requires --browser')
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ParserTests))
    if not result.wasSuccessful():
        sys.exit(1)
    if args.browser:
        browser_tests(args.browser, args.native, args.artifacts)
