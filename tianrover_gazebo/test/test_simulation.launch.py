"""Headless integration test of the installed simulator and keyboard executable."""
import math
import os
from pathlib import Path
import pty
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    share = Path(get_package_share_directory('tianrover_gazebo'))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/gazebo.launch.py')),
        launch_arguments={'namespace': 'sim_test', 'gui': 'false',
                          'world': str(Path(get_package_share_directory('gazebo_ros')) / 'worlds/empty.world'),
                          'spawn_x': '0', 'spawn_y': '0', 'enable_skid_rate': 'true'}.items())
    return launch.LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestSimulation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('simulation_probe')
        cls.keyboard_node = rclpy.create_node('keyboard_probe')
        cls.odom = []
        cls.joints = []
        cls.skid = []
        cls.paths = []
        cls.tf = []
        cls.commands = []
        cls.subscriptions = [
            cls.node.create_subscription(Odometry, '/sim_test/odom', cls.odom.append, 10),
            cls.node.create_subscription(JointState, '/sim_test/joint_states', cls.joints.append, qos_profile_sensor_data),
            cls.node.create_subscription(Float64MultiArray, '/sim_test/SkidRate', cls.skid.append, 10),
            cls.node.create_subscription(NavPath, '/sim_test/abs_path', cls.paths.append, 10),
            cls.node.create_subscription(TFMessage, '/tf', cls.tf.append, 100),
            cls.keyboard_node.create_subscription(Twist, '/sim_test/cmd_vel', cls.commands.append, 10)]
        cls.cmd = cls.node.create_publisher(Twist, '/sim_test/cmd_vel', 1)
        cls.ack = cls.node.create_publisher(AckermannDrive, '/sim_test/ackermann_cmd', 1)

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            cls.cmd.publish(Twist())
        cls.node.destroy_node()
        cls.keyboard_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def wait_until(self, predicate, timeout=60, node=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node or self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('Timed out waiting for simulation data')

    def sim_seconds(self):
        stamp = self.odom[-1].header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def drive(self, publisher, message, duration):
        start = self.sim_seconds()
        deadline = time.monotonic() + 60
        while self.sim_seconds() - start < duration and time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.02)
        self.assertGreaterEqual(self.sim_seconds() - start, duration)
        print('simulation:', self.sim_seconds(), 'position:', self.odom[-1].pose.pose.position,
              'velocity:', self.odom[-1].twist.twist, flush=True)

    def test_simulation_and_keyboard(self):
        self.wait_until(lambda: self.odom and self.joints and self.skid and self.paths)
        self.assertGreater(self.odom[-1].pose.pose.position.z, -0.3)
        self.assertEqual(self.odom[-1].header.frame_id, 'sim_test/odom')
        self.assertEqual(self.odom[-1].child_frame_id, 'sim_test/base_footprint')
        self.assertGreaterEqual(len(self.joints[-1].name), 16)
        self.assertEqual(len(self.skid[-1].data), 19)
        self.assertTrue(all(math.isfinite(x) for x in self.skid[-1].data))
        self.assertEqual(self.paths[-1].header.frame_id, 'sim_test/odom')
        self.wait_until(lambda: any(t.header.frame_id == 'sim_test/odom' and
                                   t.child_frame_id == 'sim_test/base_footprint'
                                   for msg in self.tf for t in msg.transforms))
        self.assertLess(self.odom[-1].header.stamp.sec, 120)
        self.drive(self.cmd, Twist(), 0.5)
        start = self.odom[-1].pose.pose.position.x
        cmd = Twist()
        cmd.linear.x = 0.25
        self.drive(self.cmd, cmd, 4)
        self.assertGreater(self.odom[-1].pose.pose.position.x - start, 0.08)
        # Command loss must stop the physical model, not merely the ROS publisher.
        self.drive(self.cmd, Twist(), 1)
        stopped_x = self.odom[-1].pose.pose.position.x
        self.drive(self.cmd, cmd, 2)
        self.assertGreater(self.odom[-1].pose.pose.position.x - stopped_x, 0.03)
        self.wait_until(lambda: abs(self.odom[-1].twist.twist.linear.x) < 0.03, timeout=30)
        before_turn = self.odom[-1].pose.pose.orientation.z
        ack = AckermannDrive()
        ack.speed = 0.2
        ack.steering_angle = 0.25
        self.drive(self.ack, ack, 5)
        self.assertGreater(abs(self.odom[-1].pose.pose.orientation.z - before_turn), 0.02)
        self.drive(self.cmd, Twist(), 1)
        # Exercise real stdin and DDS with a pseudo terminal, including timeout and Ctrl+C.
        master, slave = pty.openpty()
        output = tempfile.TemporaryFile(mode='w+b')
        proc = subprocess.Popen(['ros2', 'run', 'tianrover_teleop', 'teleop_key',
                                 '--ros-args', '-r', '__ns:=/sim_test'],
                                stdin=slave, stdout=output, stderr=output)
        try:
            # Keep high-rate simulation/TF callbacks out of the terminal probe's
            # executor, so a busy simulator cannot starve stop-message reception.
            def wait_keyboard(predicate, timeout):
                self.wait_until(predicate, timeout=timeout, node=self.keyboard_node)

            self.wait_until(lambda: self.node.count_publishers('/sim_test/cmd_vel') >= 2, timeout=20)
            time.sleep(0.3)
            self.commands.clear()
            os.write(master, b'j')
            wait_keyboard(lambda: any(c.linear.y > 0 and c.linear.x == 0 for c in self.commands), timeout=5)
            wait_keyboard(lambda: self.commands[-1].linear.y == 0, timeout=5)
            self.commands.clear()
            os.write(master, b'a')
            wait_keyboard(lambda: any(c.angular.z > 0 for c in self.commands), timeout=5)
            self.commands.clear()
            os.write(master, b'\x03')
            wait_keyboard(lambda: any(c.linear.x == c.linear.y == c.angular.z == 0
                                     for c in self.commands), timeout=5)
            wait_keyboard(lambda: proc.poll() is not None, timeout=10)
            self.assertEqual(proc.returncode, 0)
        finally:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=5)
            os.close(master)
            os.close(slave)
            output.seek(0)
            print('keyboard process:', proc.returncode, output.read().decode(), flush=True)
            output.close()
