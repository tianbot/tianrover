"""Supply explicit uncertainty for firmware messages with unknown covariance."""
import copy
import math


def covariance_or_default(values, diagonal, selected):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite covariance')
    size = len(diagonal)
    if any(values[i * (size + 1)] < 0 for i in range(size)):
        raise ValueError('negative covariance diagonal')
    if any(values):
        if any(values[i * (size + 1)] <= 0 for i in selected):
            raise ValueError('selected measurement has no positive variance')
        return list(values)
    result = [0.0] * (size * size)
    for i, value in enumerate(diagonal):
        result[i * (size + 1)] = float(value)
    return result


def prepare_odom(message, velocity_variance):
    result = copy.deepcopy(message)
    if not all(math.isfinite(v) for v in (result.twist.twist.linear.x,
                                         result.twist.twist.linear.y)):
        raise ValueError('non-finite planar velocity')
    result.twist.covariance = covariance_or_default(
        result.twist.covariance, [velocity_variance, velocity_variance, 0, 0, 0, 0], [0, 1])
    return result


def prepare_imu(message, yaw_variance):
    result = copy.deepcopy(message)
    if result.orientation_covariance[0] == -1:
        raise ValueError('IMU orientation is unavailable')
    q = result.orientation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 0.5 or norm > 1.5:
        raise ValueError('invalid IMU quaternion')
    q.x, q.y, q.z, q.w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    result.orientation_covariance = covariance_or_default(
        result.orientation_covariance, [yaw_variance, yaw_variance, yaw_variance], [2])
    return result


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    class Inputs(Node):
        def __init__(self):
            super().__init__('ekf_inputs')
            defaults = {'odom_topic': 'odom', 'imu_topic': 'imu',
                        'base_frame': 'base_footprint', 'imu_frame': 'imu_link',
                        'velocity_variance': 0.0025, 'yaw_variance': 0.0025}
            for key, value in defaults.items():
                self.declare_parameter(key, value)
            self.values = {key: self.get_parameter(key).value for key in defaults}
            for key in ('velocity_variance', 'yaw_variance'):
                if not math.isfinite(self.values[key]) or self.values[key] <= 0:
                    raise ValueError(key + ' must be finite and positive')
            self.odom_pub = self.create_publisher(Odometry, 'ekf/wheel', 20)
            self.imu_pub = self.create_publisher(Imu, 'ekf/imu', 20)
            self.subscriptions_ = [
                self.create_subscription(Odometry, self.values['odom_topic'], self.odom,
                                         qos_profile_sensor_data),
                self.create_subscription(Imu, self.values['imu_topic'], self.imu,
                                         qos_profile_sensor_data)]
            self.get_logger().info('Zero covariances use commissioning defaults; these are not calibrated uncertainties')

        def odom(self, msg):
            try:
                if msg.child_frame_id != self.values['base_frame']:
                    raise ValueError('unexpected odometry child frame: ' + msg.child_frame_id)
                self.odom_pub.publish(prepare_odom(msg, self.values['velocity_variance']))
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)

        def imu(self, msg):
            try:
                if msg.header.frame_id != self.values['imu_frame']:
                    raise ValueError('unexpected IMU frame: ' + msg.header.frame_id)
                self.imu_pub.publish(prepare_imu(msg, self.values['yaw_variance']))
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)

    rclpy.init(args=args)
    node = Inputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
