from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


@pytest.mark.parametrize('namespace', ['', 'mars', 'fleet/mars'])
@pytest.mark.parametrize('skid', ['false', 'true'])
def test_installed_model(namespace, skid):
    share = Path(get_package_share_directory('tianrover_gazebo'))
    prefix = namespace + '/' if namespace else ''
    xml = xacro.process_file(str(share / 'urdf/tianrover_mini_continuous.xacro'),
                            mappings={'namespace': namespace, 'frame_prefix': prefix,
                                      'enable_skid_rate': skid}).toxml()
    model = ET.fromstring(xml)
    joints = {j.get('name') for j in model.findall('joint')}
    for mesh in model.iter('mesh'):
        uri = mesh.get('filename')
        assert uri.startswith('package://tianrover_gazebo/')
        assert (share / uri.split('/', 3)[3]).is_file()
    for joint in model.iter('joint_name'):
        assert joint.text in joints
    assert model.find('.//odometryFrame').text == prefix + 'odom'
    assert model.find('.//robotBaseFrame').text == prefix + 'base_footprint'
    assert len(list(model.iter('plugin'))) == (3 if skid == 'true' else 2)
    for uri in ET.parse(share / 'urdf/Obstacle.world').iter('uri'):
        if uri.text.startswith('model://terrain/'):
            assert (share / 'meshes' / uri.text[len('model://'):]).is_file()
