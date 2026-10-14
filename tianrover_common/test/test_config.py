import pytest
from tianrover_common.config import Config, rewrite_parameters


@pytest.mark.parametrize('namespace', ['', '/', 'mars', '/fleet/mars/'])
def test_frame_topic_contract(namespace):
    cfg = Config({'TIANBOT_NAME': namespace})
    assert not cfg.frame('laser').startswith('/')
    assert cfg.topic('scan').startswith('/')
    data = {'costmap': {'ros__parameters': {'global_frame': 'odom',
            'robot_base_frame': 'base_footprint', 'scan': {'topic': '/scan'},
            'odom_topic': '/odom', 'map_name': '/map', 'use_sim_time': True}}}
    result = rewrite_parameters(data, cfg)['costmap']['ros__parameters']
    assert result['global_frame'] == cfg.frame('odom')
    assert result['scan']['topic'] == cfg.topic('scan')
    assert result['odom_topic'] == cfg.topic('odom')
    assert result['map_name'] == cfg.topic('map')
    assert result['use_sim_time'] is False
    assert data['costmap']['ros__parameters']['global_frame'] == 'odom'


def test_two_robots_are_disjoint():
    a, b = Config({'TIANBOT_NAME': 'mars_a'}), Config({'TIANBOT_NAME': 'mars_b'})
    assert a.frame('map') != b.frame('map')
    assert a.topic('cmd_vel') != b.topic('cmd_vel')


@pytest.mark.parametrize('bad', ['9mars', 'mars-a', 'a//b', '../mars'])
def test_invalid_namespace(bad):
    with pytest.raises(ValueError):
        Config({'TIANBOT_NAME': bad})


def test_external_adapter_without_core_changes(tmp_path):
    adapter = tmp_path / 'lidar/new_sensor.launch.py'
    adapter.parent.mkdir()
    adapter.write_text('')
    cfg = Config({'TIANROVER_COMPONENT_DIR': str(tmp_path), 'TIANROVER_LIDAR': 'new_sensor'})
    assert cfg.component('/missing', 'lidar', 'rslidar') == str(adapter)
    assert Config({'TIANROVER_LIDAR': 'none'}).component('/missing', 'lidar', 'rslidar') is None


def test_missing_adapter_is_explicit():
    with pytest.raises(FileNotFoundError):
        Config({'TIANROVER_LIDAR': 'missing'}).component('/missing', 'lidar', 'rslidar')
