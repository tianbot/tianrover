"""Expose velocities derived only from consecutive KISS poses for Nav2."""
import copy
import math
import numpy as np


def rotation(q):
    a = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(a).all() or np.linalg.norm(a) < 0.5:
        raise ValueError('invalid lidar pose quaternion')
    x, y, z, w = a/np.linalg.norm(a)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def derive_velocity(previous, current):
    if (not current.header.frame_id or not current.child_frame_id
            or previous.header.frame_id != current.header.frame_id
            or previous.child_frame_id != current.child_frame_id):
        raise ValueError('lidar odometry frames missing or changed')
    stamp = lambda m: m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    dt = stamp(current)-stamp(previous)
    if not 0.001 < dt <= 0.5:
        raise ValueError('lidar odometry time gap or non-monotonic stamp')
    a, b = previous.pose.pose, current.pose.pose
    ra, rb = rotation(a.orientation), rotation(b.orientation)
    delta = np.array([b.position.x-a.position.x, b.position.y-a.position.y, b.position.z-a.position.z])
    velocity = rb.T @ delta / dt
    relative = ra.T @ rb
    angle = math.acos(float(np.clip((np.trace(relative)-1)/2, -1, 1)))
    if angle >= 0.5 or not np.isfinite(velocity).all() or np.linalg.norm(velocity) > 3:
        raise ValueError('lidar odometry discontinuity')
    skew = np.array([relative[2, 1]-relative[1, 2], relative[0, 2]-relative[2, 0],
                     relative[1, 0]-relative[0, 1]])
    angular = skew*(0.5 if angle < 1e-6 else angle/(2*math.sin(angle)))/dt
    return velocity, angular


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry

    class Velocity(Node):
        def __init__(self):
            super().__init__('lidar_velocity')
            self.previous = None
            self.pub = self.create_publisher(Odometry, 'odom', 10)
            self.sub = self.create_subscription(Odometry, 'kiss/odometry', self.receive,
                                                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

        def receive(self, msg):
            try:
                rotation(msg.pose.pose.orientation)
                if not np.isfinite([msg.pose.pose.position.x, msg.pose.pose.position.y,
                                    msg.pose.pose.position.z]).all():
                    raise ValueError('non-finite lidar pose')
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)
                self.previous = None
                return
            result = copy.deepcopy(msg)
            if self.previous is not None:
                try:
                    linear, angular = derive_velocity(self.previous, msg)
                    result.twist.twist.linear.x, result.twist.twist.linear.y, result.twist.twist.linear.z = map(float, linear)
                    result.twist.twist.angular.x, result.twist.twist.angular.y, result.twist.twist.angular.z = map(float, angular)
                except ValueError as error:
                    self.previous = msg
                    self.get_logger().warning(str(error), throttle_duration_sec=5.0)
                    return
            self.previous = msg
            for i in range(6):
                result.twist.covariance[i*7] = 0.05
            self.pub.publish(result)

    rclpy.init(args=args)
    node = Velocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
