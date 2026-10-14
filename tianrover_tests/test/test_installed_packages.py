"""Check the installed product can resolve launches, executables and model assets."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.launch_description_sources import get_launch_description_from_any_launch_file


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
