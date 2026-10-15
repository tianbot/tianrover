"""Check the installed product can resolve launches, executables and model assets."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.launch_description_sources import get_launch_description_from_any_launch_file


@pytest.mark.parametrize('namespace', ['', 'mars'])
@pytest.mark.parametrize('frontend, topic', [('fastlio', 'lidar/cloud_deskewed'), ('kiss', 'kiss/frame')])
@pytest.mark.parametrize('footprint', [None, '[[-0.8,-0.5],[-0.8,0.5],[0.8,0.5],[0.8,-0.5]]'])
def test_navigation_3d_serializes_cloud_costmaps(monkeypatch, tmp_path, namespace, frontend, topic, footprint):
    from launch import LaunchContext
    from tianrover_navigation import launching

    monkeypatch.setenv('TIANBOT_NAME', namespace)
    monkeypatch.delenv('TIANROVER_NAV_POINTS_TOPIC', raising=False)
    params = yaml.safe_load((Path(launching.share()) / 'config/nav2.yaml').read_text())
    if footprint is not None:
        for key in ('local_costmap', 'global_costmap'):
            params[key][key]['ros__parameters']['footprint'] = footprint
    config = tmp_path / 'nav.yaml'
    config.write_text(yaml.safe_dump(params))
    monkeypatch.setenv('TIANROVER_NAV_PARAMS', str(config))
    # Inspect the parameter file handed to Nav2 without starting any nodes.
    monkeypatch.setattr(launching, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(use_sim_time='false', frontend=frontend)
    actions = launching.setup(context, 'navigation_3d')
    path = Path(actions[1]['parameters'][0])
    try:
        actual = yaml.safe_load(path.read_text())
    finally:
        path.unlink()
    if namespace:
        actual = actual[namespace]
    for key, layer_name in [('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')]:
        costmap = actual[key][key]['ros__parameters']
        assert 'robot_radius' not in costmap
        assert costmap['footprint'] == (footprint or '[[-0.75,-0.49],[-0.75,0.49],[0.75,0.49],[0.75,-0.49]]')
        layer = costmap[layer_name]
        assert layer['observation_sources'] == 'cloud'
        assert 'scan' not in layer
        assert layer['cloud']['topic'] == '/' + '/'.join(filter(None, [namespace, topic]))
        assert layer['cloud']['data_type'] == 'PointCloud2'
    names = [node['name'] for node in actions[1:]]
    assert 'amcl' not in names and 'map_server' not in names


def test_installed_launches_load_without_the_old_python_package():
    packages = ['tianrover_bringup', 'tianrover_description', 'tianrover_localization',
                'tianrover_slam', 'tianrover_navigation', 'tianrover_calibration', 'tianrover_rviz']
    for package in packages:
        launches = list((Path(get_package_share_directory(package)) / 'launch').rglob('*.launch.py'))
        assert launches, package
        for launch in launches:
            get_launch_description_from_any_launch_file(str(launch))


def test_installed_model_resolves_all_meshes():
    share = Path(get_package_share_directory('tianrover_description'))
    model = ET.parse(share / 'urdf/mars.urdf')
    for mesh in model.iter('mesh'):
        uri = mesh.get('filename')
        assert uri.startswith('package://tianrover_description/')
        assert (share / uri.split('/', 3)[3]).is_file(), uri


def test_metapackage_has_no_launch_or_executables():
    share = Path(get_package_share_directory('tianrover'))
    prefix = Path(get_package_prefix('tianrover'))
    assert not (share / 'launch').exists()
    assert not (prefix / 'lib/tianrover').exists()
    manifest = ET.parse(share / 'package.xml')
    assert manifest.find("exec_depend[.='tianrover_description']") is not None


def test_rviz_setup_generates_namespaced_configuration(monkeypatch, tmp_path):
    import tempfile
    from launch import LaunchContext
    from tianrover_rviz.launching import setup

    monkeypatch.setenv('TIANBOT_NAME', 'view_check')
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(use_sim_time='false', fixed_frame='view_check/map')
    assert len(setup(context, 'rviz')) == 2
    files = list(tmp_path.glob('*.rviz'))
    assert len(files) == 1
    text = files[0].read_text()
    assert 'view_check/map' in text
    assert '/view_check/scan' in text
    assert '/view_check/robot_description' in text
    assert '@MAP@' not in text and '@SCAN@' not in text
