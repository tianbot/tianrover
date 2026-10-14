"""Record and extract canonical measured joints plus independent KISS poses."""
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import uuid
import numpy as np
import yaml
from tianrover_calibration.calibration import CODES, check_profile, digest
from tianrover_calibration.calibration_alignment import align


def source_digest(bag):
    bag=Path(bag)
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    h=hashlib.sha256((bag/'metadata.yaml').read_bytes())
    for name in sorted(metadata['relative_file_paths']):
        path=(bag/name).resolve()
        try:
            path.relative_to(bag.resolve())
        except ValueError:
            raise ValueError('bag metadata references external file')
        h.update(name.encode())
        h.update(digest(path).encode())
    return h.hexdigest()


def extract(bag, profile_path, output):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    profile=yaml.safe_load(Path(profile_path).read_text())
    check_profile(profile)
    info=json.loads((Path(bag)/'calibration_recording.json').read_text())
    if info.get('profile_sha256')!=digest(profile_path) or info.get('reference')!='kiss_icp_only':
        raise ValueError('bag provenance/profile mismatch')
    before=source_digest(bag)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr'))
    types={x.name:x.type for x in reader.get_all_topics_and_types()}
    odom=info['odom_topic']; feedback=profile['feedback_topic']; audit=info['timing_topic']
    if types.get(odom)!='nav_msgs/msg/Odometry' or types.get(feedback)!='sensor_msgs/msg/JointState':
        raise ValueError('bag must contain KISS Odometry and canonical measured JointState')
    classes={name:get_message(types[name]) for name in (odom,feedback,audit)}
    poses=[]; joints=[]; timing=[]; frames=set()
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    while reader.has_next():
        topic,raw,_=reader.read_next()
        if topic not in classes:
            continue
        msg=deserialize_message(raw,classes[topic])
        if topic==audit:
            value=json.loads(msg.data)
            if value.get('valid') is True and value.get('time_field') and 1e-6<value.get('span',0)<=.2:
                timing.append(float(value['stamp']))
        elif topic==odom:
            frames.add((msg.header.frame_id,msg.child_frame_id))
            p=msg.pose.pose; q=p.orientation
            norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if not .99<norm<1.01:
                raise ValueError('invalid laser quaternion')
            roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
            pitch=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
            if abs(roll)>.08 or abs(pitch)>.08:
                raise ValueError('non-planar suspension/terrain; flat calibration refused')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            poses.append((stamp(msg),p.position.x,p.position.y,p.position.z,yaw))
        else:
            if len(set(msg.name))!=len(msg.name):
                raise ValueError('duplicate measured joint names')
            indices={name:i for i,name in enumerate(msg.name)}
            values=[]; angles=[]
            mode=profile['wheel_measurement']
            for code in CODES:
                cfg=profile['wheels'][code]
                wi=indices[cfg['wheel_joint']]; si=indices[cfg['steering_joint']]
                values.append((msg.velocity if mode=='velocity_rad_s' else msg.position)[wi])
                angles.append(msg.position[si])
            joints.append((stamp(msg),values,angles))
    if len(frames)!=1 or not poses or len(joints)<2 or not timing:
        raise ValueError('missing measurements, timing audit or changing odometry frames')
    expected=(profile['laser_odom_frame'],profile['base_frame'])
    if next(iter(frames))!=expected:
        raise ValueError('laser reference frames do not match profile')
    poses=np.asarray(poses); jt=np.array([r[0] for r in joints])
    covered=(poses[:,0]>=jt[0]+profile['feedback_time_offset_s'])&(poses[:,0]<=jt[-1]+profile['feedback_time_offset_s'])
    poses=poses[covered]
    if len(poses)<2 or np.ptp(poses[:,3])>.08:
        raise ValueError('insufficient covered poses or non-flat trajectory')
    timing=np.sort(timing)
    # Every accepted laser interval must originate from a checked timed scan.
    for t in poses[:,0]:
        i=np.searchsorted(timing,t)
        near=timing[max(0,i-1):min(len(timing),i+1)]
        if not len(near) or min(abs(near-t))>.03:
            raise ValueError('laser pose lacks matching valid per-point timing audit')
    data=align(poses[:,0],poses[:,[1,2,4]],jt,np.array([r[1] for r in joints]),
               np.array([r[2] for r in joints]),profile)
    if source_digest(bag)!=before:
        raise ValueError('bag changed during extraction')
    out=Path(output); out.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(out/'samples.npz',**data)
    manifest=dict(reference='kiss_icp_only',dataset_kind=info['dataset_kind'],recording_id=info['recording_id'],
                  source_sha256=before,samples_sha256=digest(out/'samples.npz'),profile_sha256=digest(profile_path),
                  frames=list(expected),alignment='measured_feedback_and_SE2_laser_only')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def record(profile_path, output, duration):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    profile=yaml.safe_load(Path(profile_path).read_text()); check_profile(profile)
    if not 15<=duration<=600:
        raise ValueError('recording duration must be 15..600 seconds')
    output=Path(output).expanduser().resolve()
    if output.exists():
        raise ValueError('recording directory already exists')
    ns=profile['namespace'].strip('/')
    prefix='/'+ns+'/' if ns else '/'
    odom=prefix+'kiss/odometry'; timing=prefix+'lidar/timing'
    feedback=profile['feedback_topic']
    rclpy.init(); node=Node('calibration_record_preflight'); latest={}
    def receive(key):
        return lambda msg:latest.update({key:(time.monotonic(),msg)})
    node.create_subscription(Odometry,odom,receive('odom'),qos_profile_sensor_data)
    node.create_subscription(JointState,feedback,receive('feedback'),qos_profile_sensor_data)
    node.create_subscription(String,timing,receive('timing'),10)
    process=None
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and len(latest)<3:
            rclpy.spin_once(node,timeout_sec=.1)
        if len(latest)!=3 or any(time.monotonic()-v[0]>.5 for v in latest.values()):
            raise ValueError('fresh measured joints, KISS poses and timing audit required')
        publishers=node.get_publishers_info_by_topic(feedback)
        if (len(publishers)!=1 or publishers[0].node_name in ('joint_state_publisher','joint_state_publisher_gui')
                or publishers[0].node_name!=profile['feedback_node']):
            raise ValueError('feedback publisher must match confirmed hardware node')
        laser_publishers=node.get_publishers_info_by_topic(odom)
        if len(laser_publishers)!=1 or laser_publishers[0].node_name!='kiss_icp_node':
            raise ValueError('exactly one independent KISS publisher required')
        audit=json.loads(latest['timing'][1].data)
        if not audit.get('valid') or not audit.get('time_field'):
            raise ValueError('valid per-point timing required')
        output.parent.mkdir(parents=True,exist_ok=True)
        topics=[odom,feedback,timing,prefix+'points',prefix+'points_checked','/tf','/tf_static']
        process=subprocess.Popen(['ros2','bag','record','--storage','sqlite3','--output',str(output),*topics])
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if process.poll() is not None:
                raise ValueError('rosbag recorder exited unexpectedly')
            if any(time.monotonic()-v[0]>.5 for v in latest.values()):
                raise ValueError('measurement lost while recording')
            audit=json.loads(latest['timing'][1].data)
            if not audit.get('valid'):
                raise ValueError('point timing became invalid while recording')
        process.send_signal(signal.SIGINT); process.wait(timeout=20)
        if process.returncode not in (0,-signal.SIGINT):
            raise ValueError('recorder did not close successfully')
        info=dict(reference='kiss_icp_only',dataset_kind='hardware',recording_id=str(uuid.uuid4()),
                  profile_sha256=digest(profile_path),odom_topic=odom,timing_topic=timing)
        (output/'calibration_recording.json').write_text(json.dumps(info,indent=2)+'\n')
        return info
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        node.destroy_node(); rclpy.shutdown()


def main(args=None):
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    for mode in ('record','extract'):
        p=sub.add_parser(mode)
        p.add_argument('--profile',required=True); p.add_argument('--output',required=True)
        if mode=='record':
            p.add_argument('--duration',type=float,default=120)
        else:
            p.add_argument('--bag',required=True)
    o=parser.parse_args(args)
    try:
        result=record(o.profile,o.output,o.duration) if o.command=='record' else extract(o.bag,o.profile,o.output)
        print(json.dumps(result,indent=2))
    except (ValueError,KeyError,IndexError,OSError,TypeError,RuntimeError,subprocess.SubprocessError) as error:
        parser.exit(2,'Calibration recording/extraction refused: '+str(error)+'\n')
