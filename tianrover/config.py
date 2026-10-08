"""One environment contract shared by every launch entry point."""
import os
import re
import math
from pathlib import Path


class Config:
    def __init__(self, env=None):
        self.env = os.environ if env is None else env
        self.namespace = self.env.get('TIANBOT_NAME', '').strip().strip('/')
        if self.namespace and not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', p)
                                      for p in self.namespace.split('/')):
            raise ValueError('TIANBOT_NAME must be a valid ROS namespace')

    def get(self, key, default=''):
        return self.env.get('TIANROVER_' + key, default)

    def number(self, key, default):
        value = float(self.get(key, str(default)))
        if not math.isfinite(value):
            raise ValueError('TIANROVER_' + key + ' must be finite')
        return value

    def frame(self, name):
        return '/'.join(filter(None, (self.namespace, name)))

    def topic(self, name):
        return '/' + self.frame(name)

    @property
    def maps_dir(self):
        default = Path.home() / '.local/share/tianrover/maps' / (self.namespace or 'robot')
        return Path(self.get('MAPS_DIR', str(default))).expanduser()

    def component(self, share, kind, default):
        name = self.get(kind.upper(), default)
        if name == 'none':
            return None
        if not re.fullmatch(r'[A-Za-z0-9_]+', name):
            raise ValueError('Invalid component name: ' + name)
        external = self.get('COMPONENT_DIR')
        roots = ([Path(external).expanduser()] if external else []) + [Path(share) / 'launch/components']
        for root in roots:
            path = root / kind / (name + '.launch.py')
            if path.is_file():
                return str(path)
        raise FileNotFoundError(f'No {kind} adapter for {name}; searched {roots}')


def rewrite_parameters(data, cfg, use_sim_time=False):
    """Rewrite exact TF/topic values, including nested costmap parameters."""
    frames = {'map', 'odom', 'base_link', 'base_footprint', 'laser', 'imu_link'}
    frame_keys = {'odom_frame', 'map_frame', 'base_frame', 'base_frame_id',
                  'global_frame_id', 'odom_frame_id', 'global_frame',
                  'robot_base_frame', 'target_frame', 'frame_id'}
    def visit(value, key=''):
        if isinstance(value, dict):
            return {k: visit(v, k) for k, v in value.items()}
        if isinstance(value, list):
            return [visit(v, key) for v in value]
        if key == 'use_sim_time':
            return use_sim_time
        if key in frame_keys and value in frames:
            return cfg.frame(value)
        if key == 'odom_topic':
            return cfg.topic('odom')
        if key in {'scan_topic', 'map_topic', 'map_name'}:
            return cfg.topic('scan' if key == 'scan_topic' else 'map')
        if key == 'topic' and value in {'scan', '/scan'}:
            return cfg.topic('scan')
        return value
    return visit(data)
