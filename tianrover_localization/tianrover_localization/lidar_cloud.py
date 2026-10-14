"""Validate scan timing before feeding the independent laser odometry frontend."""
import copy
import math
import time
from array import array as byte_array
import numpy as np


TIME_NAMES = ('timestamp', 'time', 't', 'time_stamp')
FORMATS = {6: 'u4', 7: 'f4', 8: 'f8'}


def checked_cloud(msg, require_time=True, max_span=0.2):
    if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
        raise ValueError('point cloud has no measurement timestamp')
    fields = {f.name: f for f in msg.fields}
    time_name = next((name for name in TIME_NAMES if name in fields), None)
    if require_time and time_name is None:
        raise ValueError('per-point timestamps missing; rebuild rslidar_sdk with POINT_TYPE=XYZIRT')
    names = ['x', 'y', 'z'] + ([time_name] if time_name else [])
    if any(name not in fields or fields[name].count != 1 or fields[name].datatype not in FORMATS
           for name in names):
        raise ValueError('unsupported coordinate or timestamp field')
    if any(fields[name].datatype not in (7, 8) for name in ('x', 'y', 'z')):
        raise ValueError('coordinates must be floating point')
    if time_name and fields[time_name].datatype == 6:
        raise ValueError('integer timestamp units unknown; convert to seconds explicitly')
    if not msg.point_step or msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.height*msg.row_step:
        raise ValueError('malformed PointCloud2 layout')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': names,
                      'formats': [endian+FORMATS[fields[n].datatype] for n in names],
                      'offsets': [fields[n].offset for n in names], 'itemsize': msg.point_step})
    array = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                       strides=(msg.row_step, msg.point_step))
    valid = np.ones(array.shape, dtype=bool)
    for name in names:
        valid &= np.isfinite(array[name])
    if valid.sum() < 100:
        raise ValueError('fewer than 100 finite points')
    span = None
    if time_name:
        times = array[time_name][valid].astype(float)
        span = float(times.max()-times.min())
        if not math.isfinite(span) or not 1e-6 < span <= max_span:
            raise ValueError('invalid scan timestamp span: ' + str(span))
        # FLOAT64 epoch seconds must agree with the host header clock. Integer
        # offsets are valid for deskew but need an explicit scale in recording audit.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if fields[time_name].datatype == 8 and times.min() > 1e8 and abs(stamp-times.max()) > max_span+0.1:
            raise ValueError('point/header clock mismatch')
    result = copy.deepcopy(msg)
    result.height, result.width = 1, int(valid.sum())
    result.row_step = result.width*result.point_step
    # Structured fancy indexing copies only named fields; padding or extra
    # fields (ring/intensity) can become uninitialized. Copy whole records.
    records = np.ndarray((msg.height, msg.width), dtype=np.dtype('V'+str(msg.point_step)),
                         buffer=bytes(msg.data), strides=(msg.row_step, msg.point_step))
    # ROS generated setters validate bytes element by element; an array('B')
    # avoids that Python loop for the ~750 KB H16 scan while retaining checks.
    result.data = byte_array('B', records[valid].tobytes())
    result.is_dense = True
    return result, {'time_field': time_name, 'span': span, 'finite_points': result.width}


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String, Bool
    from tf2_ros import Buffer, TransformListener, TransformException
    from rclpy.time import Time
    import json

    class Gate(Node):
        def __init__(self):
            super().__init__('lidar_cloud_gate')
            self.declare_parameter('require_time', True)
            self.declare_parameter('laser_frame', 'laser')
            self.declare_parameter('base_frame', 'base_footprint')
            self.declare_parameter('owned_odom_frame', '')
            self.declare_parameter('reliable_cloud', True)
            qos = QoSProfile(depth=5, reliability=(ReliabilityPolicy.RELIABLE
                             if self.get_parameter('reliable_cloud').value else ReliabilityPolicy.BEST_EFFORT))
            self.authority = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pub = self.create_publisher(PointCloud2, 'points_checked', qos)
            self.audit = self.create_publisher(String, 'lidar/timing', 10)
            self.sub = self.create_subscription(PointCloud2, 'points', self.receive, qos)
            self.authority_sub = self.create_subscription(Bool, 'lidar/tf_authority_ok',
                                                          self.authority_received, 10)

        def authority_received(self, msg):
            self.authority = (time.monotonic(), msg.data)

        def check_owner(self):
            if not self.get_parameter('owned_odom_frame').value:
                return
            if self.authority is None or time.monotonic()-self.authority[0] > .3 or not self.authority[1]:
                raise ValueError('TF authority check missing or conflicting; stop old bringup/EKF')

        def receive(self, msg):
            started = time.perf_counter()
            try:
                if msg.header.frame_id != self.get_parameter('laser_frame').value:
                    raise ValueError('unexpected lidar frame: '+msg.header.frame_id)
                self.check_owner()
                if self.get_publishers_info_by_topic('fastlio/odometry_raw'):
                    raise ValueError('FAST-LIO is active; stop it before starting KISS')
                sources = self.get_publishers_info_by_topic('kiss/odometry')
                if len(sources) != 1 or sources[0].node_name != 'kiss_icp_node':
                    raise ValueError('exactly one KISS odometry frontend required')
                readers = [p for p in self.get_subscriptions_info_by_topic('points_checked')
                           if p.node_name == 'kiss_icp_node']
                if (self.get_parameter('reliable_cloud').value
                        and (len(readers) != 1 or readers[0].qos_profile.reliability != ReliabilityPolicy.RELIABLE)):
                    raise ValueError('KISS reliable input missing; apply kiss-reliable-cloud.patch and rebuild')
                self.tf_buffer.lookup_transform(self.get_parameter('base_frame').value,
                                                msg.header.frame_id, Time.from_msg(msg.header.stamp))
                checked, audit = checked_cloud(msg, self.get_parameter('require_time').value)
                validated = time.perf_counter()
                self.pub.publish(checked)
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                audit.update(valid=True, stamp=stamp, check_seconds=validated-started,
                             publish_seconds=time.perf_counter()-validated,
                             input_age_seconds=self.get_clock().now().nanoseconds*1e-9-stamp)
            except (ValueError, TypeError, BufferError, TransformException) as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)
                audit = {'valid': False, 'error': str(error)}
            self.audit.publish(String(data=json.dumps(audit)))

    rclpy.init(args=args)
    node = Gate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
