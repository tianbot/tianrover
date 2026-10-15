"""Planar holonomic Twist control, with terminal input independent of ROS time."""
import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from tianrover_common.config import Config

# ROS 1 teleop_twist_keyboard holonomic layout, without requiring Shift.
MOVES = {'u': (1, 1, 0), 'i': (1, 0, 0), 'o': (1, -1, 0),
         'j': (0, 1, 0), 'l': (0, -1, 0),
         'm': (-1, 1, 0), ',': (-1, 0, 0), '.': (-1, -1, 0),
         'a': (0, 0, 1), 'd': (0, 0, -1),
         'q': (1, 0, 1), 'e': (1, 0, -1),
         'z': (-1, 0, 1), 'c': (-1, 0, -1)}
SPEEDS = {'t': (1.1, 1.1), 'b': (0.9, 0.9), 'w': (1.1, 1),
          'x': (0.9, 1), 'r': (1, 1.1), 'v': (1, 0.9)}
HELP = """全向键盘遥控 (geometry_msgs/Twist)
  u i o      前左 / 前 / 前右
  j k l      左移 / 停止 / 右移
  m , .      后左 / 后 / 后右
  a / d      原地左转 / 右转
  q / e      前进并左转 / 右转 (linear.x > 0)
  z / c      后退并左转 / 右转 (linear.x < 0)
  左转 angular.z > 0，右转 angular.z < 0；弧线运动 linear.y = 0
  t/b 调整全部速度，w/x 调整线速度，r/v 调整角速度
  空格或其他按键停车；Ctrl+C 退出；无按键超时自动停车。
请在此终端按键，持续行驶需按住方向键。
"""


def make_twist(move, speed, turn):
    msg = Twist()
    x, y, yaw = move
    scale = max(1.0, math.hypot(x, y))
    msg.linear.x, msg.linear.y = x * speed / scale, y * speed / scale
    msg.angular.z = yaw * turn
    return msg


class Keyboard(Node):
    def __init__(self):
        super().__init__('teleop_key', namespace=Config().namespace)
        defaults = {'speed': 0.2, 'turn': 0.4, 'speed_limit': 0.5,
                    'turn_limit': 1.0, 'key_timeout': 0.5, 'repeat_rate': 20.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: float(self.get_parameter(name).value) for name in defaults}
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        self.speed = min(values['speed'], values['speed_limit'])
        self.turn = min(values['turn'], values['turn_limit'])
        self.limits = values
        self.move = (0, 0, 0)
        self.last_key = time.monotonic()
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)

    def key(self, key):
        if key in MOVES:
            self.move = MOVES[key]
            self.last_key = time.monotonic()
        elif key in SPEEDS:
            linear, angular = SPEEDS[key]
            self.speed = min(self.limits['speed_limit'], self.speed * linear)
            self.turn = min(self.limits['turn_limit'], self.turn * angular)
            print(f'线速度 {self.speed:.3f} m/s，角速度 {self.turn:.3f} rad/s', flush=True)
        else:
            self.move = (0, 0, 0)

    def publish(self):
        if time.monotonic() - self.last_key >= self.limits['key_timeout']:
            self.move = (0, 0, 0)
        self.publisher.publish(make_twist(self.move, self.speed, self.turn))


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('teleop_key 需要交互终端：请使用 ros2 run tianrover_teleop teleop_key')
    # Handle Ctrl+C as a raw input byte, so the final stop can be published
    # before destroying the ROS context.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    settings = termios.tcgetattr(sys.stdin)
    try:
        node = Keyboard()
        print(HELP + '发布话题：' + node.publisher.topic_name, flush=True)
        tty.setcbreak(sys.stdin.fileno())
        # Disable ISIG to receive Ctrl+C, while keeping newline output readable.
        raw = termios.tcgetattr(sys.stdin)
        raw[3] &= ~termios.ISIG
        termios.tcsetattr(sys.stdin, termios.TCSANOW, raw)
        period = 1.0 / node.limits['repeat_rate']
        while rclpy.ok():
            if select.select([sys.stdin], [], [], period)[0]:
                key = sys.stdin.read(1)
                if key in ('', '\x03', '\x04'):
                    break
                node.key(key)
            rclpy.spin_once(node, timeout_sec=0)
            node.publish()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if node is not None:
            node.move = (0, 0, 0)
            # Continue sending stops briefly so busy readers can take the message
            # before DDS removes this writer during node destruction.
            for _ in range(5):
                node.publish()
                time.sleep(0.05)
            # Keep the reliable writer alive until the stop has reached its readers.
            if not node.publisher.wait_for_all_acked(Duration(seconds=0.5)):
                node.get_logger().warning('停车消息确认超时，请确认底盘已停稳。')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
