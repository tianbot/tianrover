#!/usr/bin/env python3
"""Run real KISS with synthetic 3D scans in an isolated domain; no hardware launch."""
import os
os.environ['ROS_DOMAIN_ID']='184'
os.environ['TIANBOT_NAME']='mars'
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bringup',action='store_true')
    args=parser.parse_args()
    if args.bringup:
        os.environ['TIANROVER_BASE']='none'
        os.environ['TIANROVER_LIDAR']='none'
    expected_frame='mars/odom' if args.bringup else 'mars/odom_lidar'
    root=Path(tempfile.mkdtemp(prefix='tianrover_kiss_runtime_'))
    process=None;node=None
    log=(root/'launch.log').open('w')
    try:
        entry='bringup_3d.launch.py' if args.bringup else 'laser_odom.launch.py'
        process=subprocess.Popen(['ros2','launch','tianrover',entry],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=Node('kiss_runtime_fixture')
        cloud=node.create_publisher(PointCloud2,'/mars/points',QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE))
        values=[]
        node.create_subscription(Odometry,'/mars/kiss/odometry',values.append,qos_profile_sensor_data)
        if not args.bringup:
            static=StaticTransformBroadcaster(node)
            tf=TransformStamped();tf.header.frame_id='mars/base_footprint';tf.child_frame_id='mars/laser'
            tf.transform.rotation.w=1.;tf.header.stamp=node.get_clock().now().to_msg();static.sendTransform(tf)
        deadline=time.monotonic()+15
        while time.monotonic()<deadline and cloud.get_subscription_count()==0:
            rclpy.spin_once(node,timeout_sec=.1)
        assert cloud.get_subscription_count()>0,'point gate did not start'
        for _ in range(20):
            rclpy.spin_once(node,timeout_sec=.05)
        rng=np.random.default_rng(48)
        world=rng.uniform([-6,-5,-1],[7,6,2],(5000,3)).astype('f4')
        dtype=np.dtype(dict(names=['x','y','z','intensity','timestamp','ring'],
                            formats=['<f4','<f4','<f4','<f4','<f8','<u2'],offsets=[0,4,8,12,16,24],itemsize=32))
        expected={}
        for k in range(60):
            if process.poll() is not None:
                raise AssertionError('frontend launch exited; see '+str(root))
            points=np.zeros(len(world),dtype=dtype)
            points['x']=world[:,0]-.02*k;points['y']=world[:,1];points['z']=world[:,2]
            msg=PointCloud2();msg.header.frame_id='mars/laser';msg.header.stamp=node.get_clock().now().to_msg()
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            points['timestamp']=np.linspace(stamp-.08,stamp,len(world))
            msg.height=1;msg.width=len(points);msg.point_step=32;msg.row_step=len(points)*32;msg.is_dense=True
            msg.fields=[PointField(name=n,offset=dtype.fields[n][1],datatype=8 if n=='timestamp' else (4 if n=='ring' else 7),count=1) for n in dtype.names]
            msg.data=points.tobytes();expected[stamp]=.02*k;cloud.publish(msg)
            deadline=time.monotonic()+.1
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.01)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        assert len(values)>=20,'insufficient real KISS poses'
        errors=[]
        first=values[0];initial=expected[first.header.stamp.sec+first.header.stamp.nanosec*1e-9]
        for msg in values:
            t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            assert msg.header.frame_id==expected_frame and msg.child_frame_id=='mars/base_footprint'
            errors.append(abs(msg.pose.pose.position.x-(expected[t]-initial)))
        displacement=values[-1].pose.pose.position.x-values[0].pose.pose.position.x
        assert .7<displacement<1.4,(displacement,str(root))
        assert max(errors)<.15,(max(errors),str(root))
        if args.bringup:
            assert len(node.get_publishers_info_by_topic('/mars/kiss/odometry'))==1
            publishers=node.get_publishers_info_by_topic('/mars/odom')
            assert len(publishers)==1 and publishers[0].node_name=='lidar_velocity'
            names=[name for name,_ in node.get_node_names_and_namespaces()]
            assert not any('ekf' in name or name=='icp_odometry' for name in names)
        result=dict(test='real_KISS_synthetic_3D',poses=len(values),displacement_m=displacement,
                    max_x_error_m=max(errors),logs=str(root),hardware_motion=False,entry=entry)
        (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            # A failed launch can exit while already-started children remain.
            # Clean its isolated process group even if the parent has exited.
            try:
                os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            try:
                os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:
                pass
        log.close()


if __name__=='__main__':
    main()
