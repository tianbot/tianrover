"""Finite calibration excitation; preview is the default and creates no ROS publisher."""
import json
import math
import time
import yaml
from pathlib import Path
from tianrover.calibration import check_profile


def plan():
    # Mix straight/curved motion and both directions; no crab or in-place turn.
    return [dict(duration_s=15.,vx=v,wz=w) for v,w in
            ((.1,0),(-.1,0),(.1,.08),(-.1,.08),(.1,-.08),(-.1,-.08),(.1,0),(-.1,0))]


def execute(profile_path,clearance_verified):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    p=yaml.safe_load(Path(profile_path).read_text());check_profile(p)
    if not clearance_verified:
        raise ValueError('explicit clearance verification required for this motion plan')
    timeout=p.get('firmware_command_timeout_s')
    if not isinstance(timeout,(int,float)) or not 0<timeout<=.3:
        raise ValueError('confirmed firmware command watchdog <=300 ms required')
    if p.get('planar_twist_verified') is not True:
        raise ValueError('actual chassis planar Twist interface not verified')
    namespace=p['namespace'].strip('/');prefix='/'+namespace+'/' if namespace else '/'
    rclpy.init();node=Node('calibration_motion');latest={};pub=None
    def receive(key):
        return lambda msg:latest.update({key:(time.monotonic(),msg)})
    node.create_subscription(Odometry,prefix+'odom',receive('odom'),qos_profile_sensor_data)
    node.create_subscription(JointState,p['feedback_topic'],receive('feedback'),qos_profile_sensor_data)
    node.create_subscription(String,prefix+'lidar/timing',receive('timing'),10)
    def ready():
        now=time.monotonic()
        if len(latest)!=3 or any(now-v[0]>.3 for v in latest.values()):
            raise ValueError('fresh laser velocity, actual feedback and timed points required')
        audit=json.loads(latest['timing'][1].data)
        if not audit.get('valid') or not audit.get('time_field'):
            raise ValueError('invalid timed lidar input')
        msg=latest['feedback'][1]
        measured=set(msg.name)
        if any(p['wheels'][c][k] not in measured for c in p['wheels'] for k in ('wheel_joint','steering_joint')):
            raise ValueError('missing actual active joint measurements')
        if len(measured)!=len(msg.name) or len(msg.position)!=len(msg.name) or not all(math.isfinite(x) for x in msg.position):
            raise ValueError('invalid measured joint positions')
        odom=latest['odom'][1]
        if odom.header.frame_id!=p.get('laser_odom_frame') or odom.child_frame_id!=p.get('base_frame'):
            raise ValueError('laser velocity frames do not match the verified profile')
        stamp=odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9
        feedback_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9+p['feedback_time_offset_s']
        clock=node.get_clock().now().nanoseconds*1e-9
        if abs(clock-stamp)>.3 or abs(clock-feedback_stamp)>.3:
            raise ValueError('stale measurement stamps')
        audit_stamp=audit.get('stamp')
        if not isinstance(audit_stamp,(int,float)) or not math.isfinite(audit_stamp) or abs(clock-audit_stamp)>.3:
            raise ValueError('stale lidar timing audit')
        v=odom.twist.twist
        if not all(math.isfinite(x) for x in (v.linear.x,v.linear.y,v.angular.z)) or math.hypot(v.linear.x,v.linear.y)>.3 or abs(v.angular.z)>.3:
            raise ValueError('measured motion exceeds commissioning bounds')
        publishers=node.get_publishers_info_by_topic(p['feedback_topic'])
        if len(publishers)!=1 or publishers[0].node_name!=p['feedback_node'] or publishers[0].node_name.startswith('joint_state_publisher'):
            raise ValueError('actual hardware feedback publisher required')
        laser=node.get_publishers_info_by_topic(prefix+'odom')
        if len(laser)!=1 or laser[0].node_name!='lidar_velocity':
            raise ValueError('independent laser velocity publisher required')
        command=node.get_publishers_info_by_topic(prefix+'cmd_vel')
        expected=0 if pub is None else 1
        if len(command)!=expected or (pub is not None and command[0].node_name!='calibration_motion'):
            raise ValueError('another command publisher exists; stop navigation/teleop first')
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        ready()
        pub=node.create_publisher(Twist,prefix+'cmd_vel',10)
        # Allow discovery before enforcing the exclusive command owner.
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        for segment in plan():
            deadline=time.monotonic()+segment['duration_s']
            next_command=time.monotonic()
            while time.monotonic()<deadline:
                # spin_once can return immediately on incoming feedback. Use a
                # steady-clock deadline to bound commands independently of it.
                rclpy.spin_once(node,timeout_sec=.01)
                if time.monotonic()<next_command:
                    continue
                ready()
                cmd=Twist();cmd.linear.x=segment['vx'];cmd.angular.z=segment['wz'];pub.publish(cmd)
                next_command=time.monotonic()+.05
            for _ in range(5):
                pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
    finally:
        if pub is not None and rclpy.ok():
            for _ in range(5):
                pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    import argparse
    import signal
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--clearance-verified',action='store_true')
    o=parser.parse_args(args)
    if not o.execute:
        print(json.dumps(dict(status='preview',segments=plan()),indent=2));return
    try:
        if not o.profile:
            raise ValueError('verified actual hardware profile required')
        def interrupt(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM,interrupt)
        execute(o.profile,o.clearance_verified)
    except (ValueError,KeyError,OSError,TypeError,KeyboardInterrupt) as error:
        parser.exit(2,'Calibration motion stopped/refused: '+str(error)+'\n')
