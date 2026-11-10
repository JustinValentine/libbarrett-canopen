#!/usr/bin/env python3
"""Build an offline, editable DH viewer from a libbarrett configuration.

    python3 scripts/visualize_dh.py [config/wam8g.conf] --open

Uses only the Python standard library. The generated HTML needs no server,
packages, internet connection, or robot connection. Re-run after editing the
source config; edits in the viewer can be downloaded as a separate config.
"""
import argparse
import json
import math
from pathlib import Path
import re
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
ASSETS = Path(__file__).with_name('dh_viewer')
TOKEN = re.compile(r'''(?P<space>\s+|\#[^\n]*|//[^\n]*|/\*[\s\S]*?\*/)
    |(?P<string>"(?:\\.|[^"\\])*")
    |(?P<number>[+-]?(?:0[xX][0-9a-fA-F]+|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)[lL]?)
    |(?P<include>@include) |(?P<name>[A-Za-z_][\w-]*)
    |(?P<punct>[{}()\[\]:=;,])''', re.VERBOSE)


class ConfigError(ValueError):
    pass


class ConfigParser:
    """Small libconfig reader with includes and original group locations.

    No evaluation of source text. Includes are resolved relative to the file
    containing them. The launch script reads configuration only.
    """
    def __init__(self, path, stack=()):
        self.path = Path(path).resolve()
        if self.path in stack:
            raise ConfigError(f'Include cycle: {self.path}')
        self.stack = (*stack, self.path)
        if len(self.stack) > 16:
            raise ConfigError('More than 16 nested includes')
        with self.path.open(encoding='utf-8', newline='') as source:
            self.source = source.read()
        self.tokens = []
        self.ranges = {}
        self.locations = {}
        self.sources = {self.path: self.source}
        self.i = 0
        offset = 0
        while offset < len(self.source):
            match = TOKEN.match(self.source, offset)
            if not match:
                line = self.source.count('\n', 0, offset) + 1
                raise ConfigError(f'{self.path}:{line}: unsupported configuration syntax')
            if match.lastgroup != 'space':
                self.tokens.append((match.lastgroup, match.group(), match.start(), match.end()))
            offset = match.end()

    def peek(self):
        return self.tokens[self.i][1] if self.i < len(self.tokens) else None

    def pop(self, expected=None):
        if self.i == len(self.tokens):
            raise ConfigError(f'{self.path}: unexpected end of file')
        token = self.tokens[self.i]
        self.i += 1
        if expected is not None and token[1] != expected:
            raise ConfigError(f'{self.path}: expected {expected!r}, got {token[1]!r}')
        return token

    def string(self, token):
        try:
            return json.loads(token)
        except ValueError as exc:
            raise ConfigError(f'{self.path}: invalid quoted string: {exc}') from exc

    def group(self, prefix=(), closing=None):
        result = {}
        while self.peek() != closing:
            if self.peek() is None:
                raise ConfigError(f'{self.path}: missing {closing}')
            if self.peek() == '@include':
                self.pop()
                token = self.pop()
                if token[0] != 'string':
                    raise ConfigError('@include requires a quoted path')
                child = ConfigParser(self.path.parent / self.string(token[1]), self.stack)
                included = child.group(prefix)
                for key, value in included.items():
                    if key in result:
                        raise ConfigError(f'Duplicate included setting: {key}')
                    result[key] = value
                self.locations.update(child.locations)
                self.sources.update(child.sources)
                if self.peek() == ';':
                    self.pop()
                continue
            token = self.pop()
            if token[0] != 'name':
                raise ConfigError(f'Expected setting name, got {token[1]!r}')
            key = token[1]
            if key in result:
                raise ConfigError(f'Duplicate setting: {key}')
            if self.pop()[1] not in (':', '='):
                raise ConfigError(f'Expected : or = after {key}')
            if self.i == len(self.tokens):
                raise ConfigError(f'{self.path}: missing value for {key}')
            start = self.tokens[self.i][2]
            result[key] = self.value((*prefix, key))
            self.ranges[(*prefix, key)] = (start, self.tokens[self.i - 1][3])
            self.locations[(*prefix, key)] = (self.path, *self.ranges[(*prefix, key)])
            if self.peek() in (';', ','):
                self.pop()
        if closing:
            self.pop(closing)
        return result

    def value(self, prefix):
        token = self.pop()
        kind, value = token[:2]
        if value == '{':
            result = self.group(prefix, '}')
            self.locations[prefix] = (self.path, token[2], self.tokens[self.i - 1][3])
            return result
        if value in ('(', '['):
            closing = ')' if value == '(' else ']'
            values = []
            while self.peek() != closing:
                values.append(self.value((*prefix, len(values))))
                if self.peek() == ',':
                    self.pop()
                elif self.peek() != closing:
                    raise ConfigError('Expected a comma in a list')
            self.pop(closing)
            return values
        if kind == 'string':
            text = self.string(value)
            while self.i < len(self.tokens) and self.tokens[self.i][0] == 'string':
                text += self.string(self.pop()[1])
            return text
        if kind == 'number':
            value = value.rstrip('lL')
            number = int(value, 16) if 'x' in value.lower() else float(value)
            return finite(number, f'{self.path}: numeric value')
        if value in ('true', 'false'):
            return value == 'true'
        raise ConfigError(f'Unsupported value: {value}')

    def parse(self):
        return self.group()


def finite(value, label):
    try:
        valid = not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise ConfigError(f'{label} must be a finite number')
    return value


def load_model(path, profile=None):
    parser = ConfigParser(path)
    config = parser.parse()
    if profile is None:
        profiles = [key for key, value in config.items()
                    if isinstance(value, dict) and 'kinematics' in value]
        if len(profiles) != 1:
            raise ConfigError('Choose --profile from: ' + (', '.join(profiles) or '(none found)'))
        profile = profiles[0]
    try:
        robot = config[profile]
        kin = robot['kinematics']
        moving = kin['moving']
        tool = kin['toolplate']
    except (KeyError, TypeError) as exc:
        raise ConfigError(f'Missing {profile}.kinematics / moving / toolplate') from exc
    if not isinstance(moving, list) or not moving:
        raise ConfigError('kinematics.moving must be a nonempty list')
    if profile == 'wam8g' and len(moving) != 8:
        raise ConfigError('wam8g requires eight moving DH rows')
    for i, row in enumerate([*moving, tool]):
        if not isinstance(row, dict):
            raise ConfigError(f'DH row {i + 1} must be a group')
        for key in ('alpha_pi', 'a', 'd'):
            value = finite(row.get(key), f'DH row {i + 1}.{key}')
            finite(value * 1000, f'DH row {i + 1}.{key} in display units')
    finite(tool.get('theta_pi'), 'toolplate.theta_pi')
    finite(tool['theta_pi'] * math.pi, 'toolplate.theta_pi in radians')
    for i, row in enumerate(moving):
        theta = finite(row.get('theta_pi', 0), f'DH row {i + 1}.theta_pi')
        finite(theta * 180, f'DH row {i + 1}.theta_pi in degrees')
        row.setdefault('theta_pi', 0)
    world = kin.get('world_to_base', [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    if not isinstance(world, list) or len(world) != 4 or any(
            not isinstance(row, list) or len(row) != 4 for row in world):
        raise ConfigError('world_to_base must be 4 by 4')
    for row in world:
        for x in row:
            finite(x, 'world_to_base')
    if any(abs(world[3][j] - (1 if j == 3 else 0)) > 1e-8 for j in range(4)):
        raise ConfigError('world_to_base last row must be (0, 0, 0, 1)')
    for i in range(3):
        for j in range(3):
            if abs(sum(world[k][i] * world[k][j] for k in range(3)) - (i == j)) > 1e-6:
                raise ConfigError('world_to_base rotation must be orthonormal')
    determinant = sum(world[0][i] * (world[1][(i + 1) % 3] * world[2][(i + 2) % 3]
                      - world[1][(i + 2) % 3] * world[2][(i + 1) % 3]) for i in range(3))
    if abs(determinant - 1) > 1e-6:
        raise ConfigError('world_to_base rotation must have determinant +1')
    low_level = robot.get('low_level', {})
    home = robot.get('home', low_level.get('home') if isinstance(low_level, dict) else None)
    warnings = []
    if home is not None:
        if not isinstance(home, list):
            raise ConfigError('Home position must be a list')
        for v in home:
            finite(v, 'home angle')
            finite(v * 180 / math.pi, 'home angle in degrees')
        if len(home) != len(moving):
            warnings.append(f'Config home has {len(home)} angles for {len(moving)} joints; the home preset is disabled.')
            home = None
    owner, start, end = parser.locations[(profile, 'kinematics')]
    source = parser.sources[owner]
    # JavaScript string offsets count UTF-16 code units, including surrogate pairs.
    def js_offset(offset):
        return len(source[:offset].encode('utf-16-le')) // 2
    fields = []
    for i, row in enumerate([*moving, tool]):
        prefix = (profile, 'kinematics', 'moving', i) if i < len(moving) else (profile, 'kinematics', 'toolplate')
        for key in ('alpha_pi', 'a', 'd', 'theta_pi'):
            location = parser.locations.get((*prefix, key))
            if location is None and key == 'theta_pi' and i < len(moving):
                row_owner, _, row_end = parser.locations[prefix]
                if row_owner != owner:
                    raise ConfigError('DH parameters must be in the file containing the kinematics group')
                fields.append(dict(row=i, key=key, missing=True,
                                   span=[js_offset(row_end - 1)] * 2))
                continue
            if location is None or location[0] != owner:
                raise ConfigError('DH parameters must be in the file containing the kinematics group')
            fields.append(dict(row=i, key=key, span=[js_offset(v) for v in location[1:]]))
    return dict(profile=profile, path=str(owner), source=source,
                span=[js_offset(start), js_offset(end)], fields=fields,
                moving=moving, tool=tool, world=world, home=home, warnings=warnings,
                input_files=[str(p) for p in parser.sources],
                calibrated=robot.get('kinematics_calibrated'), passive=4 if profile == 'wam8g' else None)


def build_html(model):
    html = (ASSETS / 'index.html').read_text()
    # User config text must remain data, even if a comment contains </script>.
    data = json.dumps(model, allow_nan=False).replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    replacements = {'STYLE': (ASSETS / 'style.css').read_text(), 'MODEL': data,
                    'MATH': (ASSETS / 'math.js').read_text(), 'APP': (ASSETS / 'app.js').read_text()}
    # Substitute once: placeholder-looking text inside a config comment is data.
    return re.sub(r'/\*__(STYLE|MODEL|MATH|APP)__\*/', lambda m: replacements[m[1]], html)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('config', nargs='?', type=Path, default=ROOT / 'config/wam8g.conf')
    ap.add_argument('--profile', help='Robot group (inferred when the file has one robot)')
    ap.add_argument('--output', type=Path)
    ap.add_argument('--open', action='store_true', help='Open the generated viewer in your browser')
    args = ap.parse_args()
    try:
        model = load_model(args.config, args.profile)
        output = args.output or ROOT / 'build/dh-viewer' / (model['profile'] + '.html')
        if output.resolve() in map(Path, model['input_files']) or output.suffix.lower() not in ('.html', '.htm'):
            raise ConfigError('--output must be an HTML file, separate from the input configuration')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(build_html(model))
    except (ValueError, OSError, KeyError, TypeError) as exc:
        ap.exit(1, f'Could not build viewer: {exc}\n')
    print(f'Loaded {len(model["moving"])} joints from {model["path"]}')
    print(f'Viewer: {output.resolve()}')
    if args.open:
        webbrowser.open_new_tab(output.resolve().as_uri())


if __name__ == '__main__':
    main()
