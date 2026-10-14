LAUNCH_PACKAGES = {'base': 'tianrover_bringup', 'lidar': 'tianrover_bringup', 'scan': 'tianrover_bringup', 'zed': 'tianrover_bringup', 'zed_mount': 'tianrover_bringup', 'bringup': 'tianrover_bringup', 'bringup_3d': 'tianrover_bringup', 'description': 'tianrover_description', 'ekf': 'tianrover_localization', 'laser_odom': 'tianrover_localization', 'fastlio': 'tianrover_localization', 'localization_3d': 'tianrover_localization', 'slam': 'tianrover_slam', 'map_save': 'tianrover_slam', 'gmapping': 'tianrover_slam', 'cartographer': 'tianrover_slam', 'tianrover_gmapping': 'tianrover_slam', 'tianrover_cartographer': 'tianrover_slam', 'tianrover_slam': 'tianrover_slam', 'cartographer_localization': 'tianrover_slam', 'mapping_3d': 'tianrover_slam', 'map_save_3d': 'tianrover_slam', 'navigation': 'tianrover_navigation', 'navigation_3d': 'tianrover_navigation', 'localize_and_navigate_3d': 'tianrover_navigation', 'rviz': 'tianrover_rviz', 'rviz_3d': 'tianrover_rviz', 'calibration_motion': 'tianrover_calibration'}
#!/usr/bin/env python3
"""Gmapping/Cartographer map, save, reload and Nav2 planning; domain189, no hardware."""
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

# Must be set before rclpy initialization; never join the real robot domain.
os.environ['ROS_DOMAIN_ID'] = '189'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose


def main():
    work = Path(tempfile.mkdtemp(prefix='tianrover_slam2d_'))
    processes, streams = [], []
    def launch(ns, entry, *args):
        env = dict(os.environ, TIANBOT_NAME=ns)
        env.pop('TIANROVER_CONFIG_FILE', None)
        log = work / f'{ns}_{entry}.log'
        stream = log.open('w')
        streams.append(stream)
        process = subprocess.Popen(['ros2', 'launch', LAUNCH_PACKAGES[entry], entry + '.launch.py', *args],
                                   env=env, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    rclpy.init()
    node = rclpy.create_node('integration_fixture')
    names = ['gmapping_check', 'cartographer_check']
    algorithms = dict(zip(names, ['gmapping', 'cartographer']))
    scans, maps = {}, {}
    pubs, subscriptions = [], []
    dynamic, static = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for ns in names:
        from sensor_msgs.msg import PointCloud2
        pubs.append((ns, node.create_publisher(PointCloud2, '/' + ns + '/points',
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)),
                     node.create_publisher(Odometry, '/' + ns + '/odom', 10)))
        subscriptions.append(node.create_subscription(LaserScan, '/' + ns + '/scan',
                             lambda msg, name=ns: scans.__setitem__(name, msg), qos))
        subscriptions.append(node.create_subscription(OccupancyGrid, '/' + ns + '/map',
                             lambda msg, name=ns: maps.__setitem__(name, msg), map_qos))
    transforms = []
    initial_pubs = {ns: node.create_publisher(PoseWithCovarianceStamped, '/' + ns + '/initialpose', 10)
                    for ns in names}
    nav_stage, last_initial = False, 0.0
    for ns in names:
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = ns + '/base_footprint', ns + '/laser'
        tf.transform.translation.z, tf.transform.rotation.w = 0.677, 1.0
        transforms.append(tf)
    static.sendTransform(transforms)
    points = []
    for i in range(720):
        angle = -math.pi + i * 2 * math.pi / 720
        distance = 4.0 / max(abs(math.cos(angle)), abs(math.sin(angle)))
        points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))

    def tick():
        nonlocal last_initial
        stamp = node.get_clock().now().to_msg()
        for ns, cloud_pub, odom_pub in pubs:
            cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id=ns + '/laser'), points))
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp, ns + '/odom'
            odom.child_frame_id, odom.pose.pose.orientation.w = ns + '/base_footprint', 1.0
            odom_pub.publish(odom)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id = stamp, ns + '/odom'
            tf.child_frame_id, tf.transform.rotation.w = ns + '/base_footprint', 1.0
            dynamic.sendTransform(tf)
        if nav_stage and time.monotonic() - last_initial >= 2.0:
            for ns, publisher in initial_pubs.items():
                pose = PoseWithCovarianceStamped()
                # A zero stamp requests the latest TF when initializing AMCL.
                pose.header.frame_id = ns + '/map'
                pose.pose.pose.orientation.w = 1.0
                pose.pose.covariance[0] = pose.pose.covariance[7] = 0.05
                pose.pose.covariance[35] = 0.05
                publisher.publish(pose)
            last_initial = time.monotonic()

    def wait(predicate, seconds=70):
        deadline = time.monotonic() + seconds
        next_scan = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_scan:
                tick()
                next_scan = time.monotonic() + 0.1
            rclpy.spin_once(node, timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError(f'Timed out; scans={list(scans)}, maps={list(maps)}; inspect {work}; '
                             f'topics={node.get_topic_names_and_types()}')

    try:
        for ns in names:
            launch(ns, 'scan')
            launch(ns, algorithms[ns])
        wait(lambda: len(scans) == 2 and len(maps) == 2)
        for ns in names:
            assert scans[ns].header.frame_id == ns + '/laser'
            assert maps[ns].header.frame_id == ns + '/map'
            assert maps[ns].info.width > 0 and any(v >= 0 for v in maps[ns].data)
        print('PASS: two pointcloud -> scan -> SLAM maps with isolated frame IDs', flush=True)
        savers = []
        for ns in names:
            savers.append(launch(ns, 'map_save', 'slam_methods:='+algorithms[ns],
                                 'map_path:='+str(work/ns), 'map_file:=map'))
        wait(lambda: all(p.poll() is not None for p in savers), 30)
        assert all(p.returncode == 0 for p in savers)
        assert all((work / ns / 'map.yaml').is_file() for ns in names)
        assert (work/'cartographer_check'/'map.pbstream').stat().st_size > 0
        print('PASS: Gmapping PGM/YAML and Cartographer PGM/YAML/pbstream saved', flush=True)
        # Stop SLAM before AMCL takes ownership of map -> odom.
        for process in processes[:4]:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes[:4]:
            process.wait(timeout=15)
        # Verify the saved Cartographer state can actually be reloaded frozen.
        from cartographer_ros_msgs.srv import GetTrajectoryStates
        native = launch('cartographer_check', 'cartographer_localization',
                        'state:='+str(work/'cartographer_check'/'map.pbstream'))
        client = node.create_client(GetTrajectoryStates, '/cartographer_check/get_trajectory_states')
        wait(client.service_is_ready, 30)
        future = client.call_async(GetTrajectoryStates.Request())
        wait(future.done, 30)
        result = future.result()
        assert result.status.code == 0
        states = result.trajectory_states
        assert states.FROZEN in states.trajectory_state, 'saved graph was not loaded frozen'
        print('PASS: Cartographer pbstream reloaded for native localization', flush=True)
        os.killpg(native.pid, signal.SIGINT)
        native.wait(timeout=15)
        node.destroy_client(client)
        # Restart scan only; bringup hardware adapters are intentionally excluded.
        for ns in names:
            launch(ns, 'scan')
            launch(ns, 'navigation', 'map:=' + str(work / ns / 'map.yaml'))
        nav_stage = True
        clients = {ns: node.create_client(GetState, '/' + ns + '/bt_navigator/get_state') for ns in names}
        wait(lambda: all(c.service_is_ready() for c in clients.values()), 70)
        states = {}
        pending = {}
        def active():
            for ns, client in clients.items():
                if states.get(ns) == 3:
                    continue
                request = pending.get(ns)
                if request is not None:
                    future, sent_at = request
                    if future.done():
                        states[ns] = future.result().current_state.id
                        del pending[ns]
                    elif time.monotonic() - sent_at > 3.0:
                        client.remove_pending_request(future)
                        del pending[ns]
                if states.get(ns) != 3 and ns not in pending:
                    pending[ns] = (client.call_async(GetState.Request()), time.monotonic())
            return len(states) == 2 and all(state == 3 for state in states.values())
        wait(active, 70)
        nav_stage = False
        actions = dict(node.get_service_names_and_types())
        for ns in names:
            assert '/' + ns + '/navigate_to_pose/_action/send_goal' in actions
            assert '/' + ns + '/compute_path_to_pose/_action/send_goal' in actions
            assert '/' + ns + '/map_server/get_state' in actions
        graph = dict(node.get_topic_names_and_types())
        assert '/tf' in graph and '/tf_static' in graph
        assert all(topic not in graph for topic in ('/scan', '/points', '/map', '/odom', '/cmd_vel'))
        for ns in names:
            assert '/' + ns + '/tf' not in graph
            assert '/' + ns + '/cmd_vel' in graph
            assert maps[ns].header.frame_id == ns + '/map'
        print('PASS: both Nav2 stacks active; isolated services/actions/cmd_vel; shared TF transport', flush=True)
        for ns in names:
            client = ActionClient(node, ComputePathToPose, '/' + ns + '/compute_path_to_pose')
            wait(lambda: client.server_is_ready(), 20)
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = goal.start.header.frame_id = ns + '/map'
            goal.goal.pose.position.x = goal.goal.pose.position.y = 1.0
            goal.goal.pose.orientation.w = goal.start.pose.orientation.w = 1.0
            goal.use_start, goal.planner_id = True, 'GridBased'
            sent = client.send_goal_async(goal)
            wait(sent.done, 20)
            assert sent.result().accepted
            result = sent.result().get_result_async()
            wait(result.done, 20)
            path = result.result().result.path
            assert len(path.poses) > 2 and path.header.frame_id == ns + '/map'
            client.destroy()
        print('PASS: both planners returned paths in their own map frames; no navigation motion goal sent', flush=True)
        print('Artifacts: ' + str(work), flush=True)
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for stream in streams:
            stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
