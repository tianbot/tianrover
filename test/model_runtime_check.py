#!/usr/bin/env python3
"""Verify installed description and moving-joint TF without hardware or motor commands."""
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

os.environ['ROS_DOMAIN_ID'] = '181'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['TIANBOT_NAME'] = 'mars_model_check'
os.environ.pop('TIANROVER_URDF', None)

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def main():
    spec = importlib.util.spec_from_file_location('model_kinematics', Path(__file__).with_name('test_mars_urdf.py'))
    kinematics = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(kinematics)
    share = Path(get_package_share_directory('tianrover'))
    installed = share / 'urdf/mars.urdf'
    assert installed.read_bytes() == kinematics.MODEL.read_bytes()
    model = ET.parse(installed).getroot()
    workspace = Path(tempfile.mkdtemp(prefix='tianrover_model_tf_'))
    processes, streams = [], []

    def start(command, name):
        stream = (workspace / (name + '.log')).open('w')
        streams.append(stream)
        proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=10)

    rclpy.init()
    node = rclpy.create_node('model_tf_probe')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    pub = node.create_publisher(JointState, '/mars_model_check/joint_states', 10)

    def observed(frame):
        message = buffer.lookup_transform('mars_model_check/base_footprint',
                                          'mars_model_check/' + frame, rclpy.time.Time())
        t, q = message.transform.translation, message.transform.rotation
        quat = np.array([q.x, q.y, q.z, q.w])
        quat /= np.linalg.norm(quat)
        x, y, z, w = quat
        matrix = np.eye(4)
        matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                          [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                          [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def wait_for_pose(pose, publish=False):
        expected = kinematics.forward(model, pose)
        deadline = time.monotonic() + 15
        difference = None
        while time.monotonic() < deadline:
            if publish:
                message = JointState()
                message.header.stamp = node.get_clock().now().to_msg()
                message.name = [j.get('name') for j in model.findall('joint') if j.get('type') != 'fixed']
                message.position = [float(pose.get(name, 0)) for name in message.name]
                pub.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                difference = max(np.max(np.abs(observed(frame)-matrix))
                                 for frame, matrix in expected.items() if frame != 'base_footprint')
                if difference < 1e-7:
                    return difference
            except Exception:
                pass
        raise AssertionError(f'TF pose not reached: {pose}; last error={difference}')

    try:
        description = start(['ros2', 'launch', 'tianrover', 'description.launch.py'], 'description')
        error = wait_for_pose({})
        assert description.poll() is None
        print(f'PASS installed description.launch.py: all prefixed neutral TF frames, error={error}', flush=True)
        stop(description)
        # Exercise the real feedback launch path, with no default-zero publisher.
        rsp = start(['ros2', 'launch', 'tianrover', 'description.launch.py',
                     'use_joint_state_publisher:=false'], 'moving_model')
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.count_subscribers('/mars_model_check/joint_states'):
                break
        assert node.count_publishers('/mars_model_check/joint_states') == 1, \
            'Only the isolated probe may publish joint_states in feedback mode'
        for angle in (-0.6, 0.6):
            pose = {'Bogie' + code + '_': angle for code in ('LF', 'LM', 'LB', 'RF', 'RM', 'RB')}
            error = wait_for_pose(pose, publish=True)
            print(f'PASS six steering joints {angle} rad: live TF error={error}', flush=True)
        pose = {'WheelRF': 0.7}
        pose.update({'Wheel' + code + '_': 0.7 for code in ('LF', 'LM', 'LB', 'RM', 'RB')})
        print('PASS six rolling joints: live TF error=', wait_for_pose(pose, publish=True), flush=True)
        for side in ('Left', 'Right'):
            print(f'PASS {side} main rocker: live TF error=',
                  wait_for_pose({side + 'MainRocket_': 0.2}, publish=True), flush=True)
        assert rsp.poll() is None
        print('PASS: no hardware nodes or cmd_vel publishers used; logs:', workspace, flush=True)
    finally:
        for proc in reversed(processes):
            stop(proc)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
