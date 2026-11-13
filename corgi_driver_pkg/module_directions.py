"""Per-leg directions for the sim driver (CONVENTIONS.md section 5).

theta, beta and gamma come from the ONE table the robot bridge also reads,
share/corgi_utils/config/module_directions.yaml (override the path with
CORGI_MODULE_DIRECTIONS). The rules are those of the bridge's loader,
corgi_utils/module_directions.hpp: every leg and every key is required, no
other keys are allowed, each value is +1 or -1, and theta is +1. A bad table
stops the driver; nothing defaults to +1.

The Webots model's own details -- the G joint's beta direction and the raw
L/R motor polarity -- are not frame conventions and stay in webots_model.yaml
beside this file, which may not set theta, beta or gamma.

No Webots import here, so the tests can load the real tables.
"""
import os

import yaml

LEGS = ('A', 'B', 'C', 'D')
FRAME_KEYS = ('theta', 'beta', 'gamma')
MODEL_KEYS = ('g_joint_beta', 'motor_l', 'motor_r')
MODEL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'webots_model.yaml')


def table_path():
    """The shared table: CORGI_MODULE_DIRECTIONS, else the corgi_utils share."""
    env = os.environ.get('CORGI_MODULE_DIRECTIONS')
    if env:
        return env
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('corgi_utils'),
                        'config', 'module_directions.yaml')


def _rows(path, keys, forbidden_hint):
    with open(path, 'r') as f:
        root = yaml.safe_load(f) or {}
    out = {}
    for leg in LEGS:
        row = root.get(leg)
        if not isinstance(row, dict):
            raise ValueError(f"leg {leg} is missing from {path}")
        unknown = sorted(set(row) - set(keys))
        if unknown:
            raise ValueError(f"leg {leg}: unknown key(s) {unknown} in {path} "
                             f"(expected {', '.join(keys)}{forbidden_hint})")
        values = {}
        for k in keys:
            if k not in row:
                raise ValueError(f"leg {leg}: {k} is missing from {path}")
            v = row[k]
            if isinstance(v, bool) or v not in (1, -1):
                raise ValueError(f"leg {leg}: {k} must be +1 or -1, got {v!r} in {path}")
            values[k] = float(v)
        out[leg] = values
    return out


def load_frame_directions(path=None):
    """{leg: {theta, beta, gamma}} from the shared table, and its path."""
    path = path or table_path()
    rows = _rows(path, FRAME_KEYS, '; CONVENTIONS.md section 5')
    for leg, d in rows.items():
        if d['theta'] < 0:
            raise ValueError(f"leg {leg}: theta = -1 is not supported; the 17 deg "
                             f"offset in phi breaks the r/l swap (CONVENTIONS.md section 5)")
    return rows, path


def load_model_directions(path=None):
    """{leg: {g_joint_beta, motor_l, motor_r}} from webots_model.yaml."""
    path = path or MODEL_FILE
    return _rows(path, MODEL_KEYS,
                 '; theta/beta/gamma belong in corgi_utils/config/module_directions.yaml')


def load_leg_config(table=None, model=None):
    """Everything LegManager needs per leg, and the shared table's path."""
    frame, path = load_frame_directions(table)
    webots = load_model_directions(model)
    return {leg: {**frame[leg], **webots[leg]} for leg in LEGS}, path
