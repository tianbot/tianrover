import gc
import json
import math
import numpy as np
import pytest
import yaml
from test_calibration import fixture
from tianrover_calibration.calibration import CODES, digest, load_dataset
from tianrover_calibration.calibration_bag import extract, source_digest


def test_real_rosbag_reader_extracts_measured_joints_and_laser(tmp_path):
    baglib=pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    _, profile, _, _=fixture()
    profile.update(namespace='mars',laser_odom_frame='mars/odom_lidar',base_frame='mars/base_footprint',
                   wheel_measurement='velocity_rad_s',feedback_node='hardware')
    profile_path=tmp_path/'profile.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    bag=tmp_path/'bag'
    writer=baglib.SequentialWriter()
    writer.open(baglib.StorageOptions(uri=str(bag),storage_id='sqlite3'),baglib.ConverterOptions('cdr','cdr'))
    topics={'/mars/kiss/odometry':'nav_msgs/msg/Odometry',profile['feedback_topic']:'sensor_msgs/msg/JointState',
            '/mars/lidar/timing':'std_msgs/msg/String'}
    for name,type_name in topics.items():
        writer.create_topic(baglib.TopicMetadata(name=name,type=type_name,serialization_format='cdr'))
    # Synthetic messages pass through the actual rosbag2 storage/deserializer.
    for k in range(1101):
        t=1000+k*.02
        if k%5==0:
            odom=Odometry(); odom.header.frame_id='mars/odom_lidar'; odom.child_frame_id='mars/base_footprint'
            odom.header.stamp.sec=int(t); odom.header.stamp.nanosec=round((t-int(t))*1e9)
            odom.pose.pose.position.x=.2*(t-1000); odom.pose.pose.orientation.w=1.
            writer.write('/mars/kiss/odometry',serialize_message(odom),round(t*1e9))
            timing=String(data=json.dumps(dict(valid=True,time_field='timestamp',span=.1,stamp=t)))
            writer.write('/mars/lidar/timing',serialize_message(timing),round(t*1e9))
        joints=JointState(); joints.header.stamp.sec=int(t); joints.header.stamp.nanosec=round((t-int(t))*1e9)
        joints.name=[profile['wheels'][c]['wheel_joint'] for c in CODES]+[profile['wheels'][c]['steering_joint'] for c in CODES]
        joints.position=[0.]*12; joints.velocity=[1.6]*6+[0.]*6
        writer.write(profile['feedback_topic'],serialize_message(joints),round(t*1e9))
    del writer; gc.collect()
    (bag/'calibration_recording.json').write_text(json.dumps(dict(reference='kiss_icp_only',dataset_kind='synthetic',
        recording_id='synthetic-real-storage',profile_sha256=digest(profile_path),
        odom_topic='/mars/kiss/odometry',timing_topic='/mars/lidar/timing')))
    manifest=extract(bag,profile_path,tmp_path/'dataset')
    data,loaded=load_dataset(tmp_path/'dataset')
    assert loaded==manifest
    assert manifest['dataset_kind']=='synthetic'
    assert len(data['time'])==220
    assert np.allclose(data['velocity'],[.2,0,0],atol=1e-7)
    assert np.allclose(data['omega'],1.6)
    original=source_digest(bag)
    metadata=bag/'metadata.yaml'
    metadata.write_text(metadata.read_text()+'\n# modified\n')
    assert source_digest(bag)!=original


def test_bag_metadata_external_path_refused(tmp_path):
    bag=tmp_path/'bag';bag.mkdir()
    (bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'relative_file_paths':['../outside.db3']}}))
    with pytest.raises(ValueError,match='external'):
        source_digest(bag)
