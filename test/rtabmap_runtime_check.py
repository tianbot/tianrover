#!/usr/bin/env python3
"""Real RTAB-Map mapping/save/localization with synthetic laser frontend messages."""
import os
os.environ['ROS_DOMAIN_ID']='185';os.environ['TIANBOT_NAME']='mars'
import json
from pathlib import Path
import signal
import sqlite3
import subprocess
import tempfile
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry,OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster
from tianrover.map_database import inspect_database


def main():
    root=Path(tempfile.mkdtemp(prefix='tianrover_rtabmap_runtime_'))
    database=root/'map.db';saved=root/'saved.db'
    from ament_index_python.packages import get_package_share_directory
    params=yaml.safe_load((Path(get_package_share_directory('tianrover'))/'config/rtabmap.yaml').read_text())
    params['Rtabmap/DetectionRate']='0'
    params_path=root/'rtabmap.yaml';params_path.write_text(yaml.safe_dump(params))
    os.environ['TIANROVER_RTABMAP_PARAMS']=str(params_path)
    processes=[];logs=[];node=None;velocity_node=None
    def start(mode,db):
        stream=(root/(mode+'.log')).open('w');logs.append(stream)
        args=['ros2','launch','tianrover',mode+'.launch.py']
        if mode!='navigation_3d':args.append('database:='+str(db))
        p=subprocess.Popen(args,
                           stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p
    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    try:
        rclpy.init();node=Node('rtabmap_runtime_fixture')
        scan=node.create_publisher(PointCloud2,'/mars/kiss/frame',10)
        odom=node.create_publisher(Odometry,'/mars/kiss/odometry',10)
        velocity_node=Node('lidar_velocity',namespace='mars')
        velocity=velocity_node.create_publisher(Odometry,'/mars/odom',10)
        maps=[];tf_maps=[];processed=[];cloud_maps=[]
        node.create_subscription(PointCloud2,'/mars/cloud_map',cloud_maps.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        from rtabmap_msgs.msg import Info
        node.create_subscription(Info,'/mars/info',processed.append,10)
        from tf2_msgs.msg import TFMessage
        node.create_subscription(OccupancyGrid,'/mars/map',maps.append,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(TFMessage,'/tf',lambda m:tf_maps.extend(t for t in m.transforms if t.header.frame_id=='mars/map'),10)
        static=StaticTransformBroadcaster(node);dynamic=TransformBroadcaster(node)
        fixed=TransformStamped();fixed.header.frame_id='mars/base_footprint';fixed.child_frame_id='mars/laser'
        fixed.transform.translation.z=.6;fixed.transform.rotation.w=1.;static.sendTransform(fixed)
        rng=np.random.default_rng(22)
        # Ground and asymmetric room walls, in global ground coordinates.
        floor=np.column_stack((rng.uniform(-4,6,1800),rng.uniform(-4,5,1800),np.zeros(1800)))
        a=np.column_stack((np.full(800,-4),rng.uniform(-4,5,800),rng.uniform(0,2,800)))
        b=np.column_stack((rng.uniform(-4,6,800),np.full(800,5),rng.uniform(0,2,800)))
        c=np.column_stack((np.full(800,6),rng.uniform(-4,5,800),rng.uniform(0,2,800)))
        box=np.column_stack((np.full(400,4),rng.uniform(-1,1,400),rng.uniform(0,1.5,400)))
        world=np.vstack((floor,a,b,c,box)).astype('f4')
        def publish(x,y):
            stamp=node.get_clock().now().to_msg()
            p=Odometry();p.header.stamp=stamp;p.header.frame_id='mars/odom';p.child_frame_id='mars/base_footprint'
            p.pose.pose.position.x=float(x)+.1;p.pose.pose.position.y=float(y);p.pose.pose.orientation.w=1.

            for i in range(6):p.pose.covariance[i*7]=.01
            odom.publish(p);velocity.publish(p)
            tf=TransformStamped();tf.header=p.header;tf.child_frame_id=p.child_frame_id;tf.transform.translation.x=float(x)+.1;tf.transform.translation.y=float(y);tf.transform.rotation.w=1.;dynamic.sendTransform(tf)
            pts=world-np.array([x,y,.6],dtype='f4');cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id='mars/laser'
            cloud.height=1;cloud.width=len(pts);cloud.point_step=12;cloud.row_step=12*len(pts);cloud.is_dense=True
            cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
            cloud.data=pts.tobytes();scan.publish(cloud)
            return stamp.sec+stamp.nanosec*1e-9
        def spin(seconds):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        mapping=start('mapping_3d',database)
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and scan.get_subscription_count()==0:spin(.1)
        assert scan.get_subscription_count()>0, str(root)
        corners=[(0,0),(3,0),(3,2),(0,2),(0,0)]
        route=[]
        for a,b in zip(corners[:-1],corners[1:]):
            route.extend(tuple(np.array(a)*(1-f)+np.array(b)*f) for f in np.linspace(0,1,16,endpoint=False))
        route.append((0,0))
        for x,y in route:
            assert mapping.poll() is None,str(root)
            before=len(processed)
            publish(x,y)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline and len(processed)==before:spin(.05)
            assert len(processed)>before,('backend did not process scan',str(root))
        for _ in range(10):publish(0,0);spin(.25)
        assert maps and maps[-1].info.width>10 and maps[-1].info.height>10,str(root)
        assert any(v>=0 for v in maps[-1].data),str(root)
        assert tf_maps,str(root)
        assert cloud_maps and cloud_maps[-1].width>100,('3D cloud map missing',str(root))
        # Exercise the actual save CLI, including stationarity, service path,
        # parameter verification, database integrity and pause-state restoration.
        stream=(root/'map_save.log').open('w');logs.append(stream)
        saver=subprocess.Popen(['ros2','run','tianrover','map_save_3d','--database',str(database),
                                '--output',str(saved)],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(saver);deadline=time.monotonic()+35
        while time.monotonic()<deadline and saver.poll() is None:
            publish(0,0);spin(.1)
        assert saver.poll()==0,('map save CLI failed',str(root))
        nodes=inspect_database(saved)
        stop(mapping)
        with sqlite3.connect(saved) as db:
            link_types=dict(db.execute('SELECT type,count(*) FROM Link GROUP BY type').fetchall())
        assert any(int(k) in (1,2,3,4) and v>0 for k,v in link_types.items()),('no loop closure',link_types,str(root))
        maps.clear();tf_maps.clear()
        localization=start('localize_and_navigate_3d',saved);spin(3)
        for _ in range(30):publish(0,0);spin(.25)
        assert localization.poll() is None and maps and tf_maps,str(root)
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose
        action=ActionClient(node,ComputePathToPose,'/mars/compute_path_to_pose')
        deadline=time.monotonic()+45
        while time.monotonic()<deadline and not action.server_is_ready():
            publish(0,0);spin(.2)
        assert action.server_is_ready(),('Nav2 planner not ready',str(root))
        for _ in range(20):publish(0,0);spin(.2)
        goal=ComputePathToPose.Goal();goal.goal.header.frame_id='mars/map';goal.goal.header.stamp=node.get_clock().now().to_msg()
        goal.goal.pose.position.x=1.1;goal.goal.pose.orientation.w=1.;goal.planner_id='GridBased'
        sent=action.send_goal_async(goal);rclpy.spin_until_future_complete(node,sent,timeout_sec=10)
        assert sent.done() and sent.result().accepted,('planner rejected',str(root))
        future=sent.result().get_result_async();rclpy.spin_until_future_complete(node,future,timeout_sec=15)
        assert future.done() and future.result().status==4,('planning failed',str(root))
        path=future.result().result.path
        assert len(path.poses)>5,('empty path',str(root))
        result=dict(test='RTABMap_real_nodes_synthetic_backend',graph_nodes=nodes,link_types=link_types,
                    map_cells=len(maps[-1].data),map_tf=len(tf_maps),cloud_map_points=cloud_maps[-1].width,nav2_path_poses=len(path.poses),logs=str(root),hardware_motion=False)
        (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    finally:
        for p in reversed(processes):stop(p)
        if velocity_node is not None:velocity_node.destroy_node()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for stream in logs:stream.close()


if __name__=='__main__':main()
