#!/usr/bin/env python3
"""Exercise production EKF bringup with synthetic sensors in an isolated domain."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ.update(ROS_DOMAIN_ID='182', ROS_LOCALHOST_ONLY='1',
                  TIANBOT_NAME='ekf_check', TIANROVER_BASE='none', TIANROVER_LIDAR='none')
# Avoid deployment overrides changing the fixture's sensor or filter contract.
for key in ('TIANROVER_EKF_PARAMS', 'TIANROVER_EKF_IMU_TOPIC', 'TIANROVER_URDF'):
    os.environ.pop(key, None)

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    folder = Path(tempfile.mkdtemp(prefix='tianrover_ekf_fixture_'))
    stream = (folder / 'launch.log').open('w')
    proc = subprocess.Popen(['ros2', 'launch', 'tianrover', 'bringup.launch.py', 'use_ekf:=true'],
                            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('ekf_fixture')
    wheel_pub = node.create_publisher(Odometry, '/ekf_check/odom_raw', 20)
    imu_pub = node.create_publisher(Imu, '/ekf_check/imu', 20)
    output, transforms = [], []
    subs = [node.create_subscription(Odometry, '/ekf_check/odom', output.append, 100),
            node.create_subscription(TFMessage, '/tf', lambda m: transforms.extend(m.transforms), 100)]

    def feed(duration, speed, turn_rate, heading):
        start = time.monotonic()
        while time.monotonic()-start < duration:
            elapsed = time.monotonic()-start
            stamp = node.get_clock().now().to_msg()
            wheel = Odometry()
            wheel.header.stamp, wheel.header.frame_id = stamp, 'ekf_check/odom'
            wheel.child_frame_id = 'ekf_check/base_footprint'
            wheel.twist.twist.linear.x = float(speed)
            # Invalid encoder-derived pose must not influence a velocity-only input.
            wheel.pose.pose.position.x = 999.0
            wheel.pose.pose.orientation.z = math.sin(-0.5)
            wheel.pose.pose.orientation.w = math.cos(-0.5)
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, 'ekf_check/imu_link'
            angle = 1.0 + heading + turn_rate*elapsed
            imu.orientation.z, imu.orientation.w = math.sin(angle/2), math.cos(angle/2)
            imu.angular_velocity.z = float(turn_rate)
            wheel_pub.publish(wheel)
            imu_pub.publish(imu)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.01)
        return heading + turn_rate*(time.monotonic()-start)

    try:
        deadline = time.monotonic()+20
        while time.monotonic() < deadline and not (wheel_pub.get_subscription_count() and
                                                   imu_pub.get_subscription_count()):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert proc.poll() is None
        feed(3, 0, 0, 0)
        assert output, f'No filtered odometry; inspect {folder}'
        assert abs(yaw(output[-1].pose.pose.orientation)) < 0.03
        before = output[-1].pose.pose.position.x
        feed(5, 0.2, 0, 0)
        distance = output[-1].pose.pose.position.x-before
        assert abs(distance-1.0) < 0.1, distance
        heading = feed(4, 0, 0.2, 0)
        assert abs(yaw(output[-1].pose.pose.orientation)-heading) < 0.06
        feed(1, 0, 0, heading)
        assert all(math.isfinite(v) for m in output for v in
                   (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.covariance[35]))
        assert output[-1].pose.covariance[35] > 0
        assert output[-1].header.frame_id == 'ekf_check/odom'
        assert output[-1].child_frame_id == 'ekf_check/base_footprint'
        assert node.count_publishers('/ekf_check/odom') == 1
        assert any(t.header.frame_id == 'ekf_check/odom' and
                   t.child_frame_id == 'ekf_check/base_footprint' for t in transforms)
        names = [name for name, ns in node.get_node_names_and_namespaces()]
        assert 'base' not in names and 'lidar' not in names
        print(f'PASS stationary, 1m straight ({distance:.3f}m), 0.8rad turn, filtered TF; logs={folder}', flush=True)
    finally:
        os.killpg(proc.pid, signal.SIGINT) if proc.poll() is None else None
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=10)
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
