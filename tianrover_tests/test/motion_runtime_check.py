#!/usr/bin/env python3
"""Exercise motion interlocks with ROS messages in domain 188; no chassis node."""
import os
os.environ['ROS_DOMAIN_ID'] = '188'
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import yaml


def child(profile):
    from tianrover_calibration import calibration_motion
    # Shorten only this fixture's plan; the product retains its 120-second plan.
    calibration_motion.plan = lambda: [dict(duration_s=.5, vx=.1, wz=0.)]
    try:
        calibration_motion.execute(profile, True)
    except ValueError as error:
        print('REFUSED: '+str(error), flush=True)
        return 2
    return 0


def main():
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tianrover_calibration/test'))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from test_calibration import fixture
    _, profile, _, _ = fixture()
    profile.update(namespace='motion_check', feedback_topic='/motion_check/measured',
                   feedback_node='hardware_feedback_fixture', firmware_command_timeout_s=.2,
                   planar_twist_verified=True, laser_odom_frame='motion_check/odom',
                   base_frame='motion_check/base_footprint')
    root = Path(tempfile.mkdtemp(prefix='tianrover_motion_runtime_'))
    profile_path = root/'profile.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    rclpy.init()
    executor = SingleThreadedExecutor()
    laser = Node('lidar_velocity', namespace='motion_check')
    feedback = Node('hardware_feedback_fixture', namespace='motion_check')
    audit = Node('motion_audit_fixture', namespace='motion_check')
    observer = Node('motion_command_observer', namespace='motion_check')
    nodes = [laser, feedback, audit, observer]
    for node in nodes:
        executor.add_node(node)
    odom_pub = laser.create_publisher(Odometry, '/motion_check/odom', 10)
    joint_pub = feedback.create_publisher(JointState, profile['feedback_topic'], 10)
    timing_pub = audit.create_publisher(String, '/motion_check/lidar/timing', 10)
    commands = []
    observer.create_subscription(Twist, '/motion_check/cmd_vel',
                                 lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10)
    names = [profile['wheels'][c][key] for c in profile['wheels']
             for key in ('wheel_joint', 'steering_joint')]
    results = []
    process = None
    try:
        for case in ('stale_stamp', 'stale_audit', 'wrong_frame', 'competing_publisher',
                     'timing_fault', 'feedback_loss', 'finite_success'):
            commands.clear()
            competitor = None
            if case == 'competing_publisher':
                competitor = observer.create_publisher(Twist, '/motion_check/cmd_vel', 10)
            log = (root/(case+'.log')).open('w')
            process = subprocess.Popen([sys.executable, __file__, '--child', str(profile_path)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+15
            while process.poll() is None and time.monotonic()<deadline:
                stamp = observer.get_clock().now().to_msg()
                odom = Odometry(); odom.header.stamp = stamp
                odom.header.frame_id = 'motion_check/odom'
                if case == 'wrong_frame':
                    odom.header.frame_id = 'unverified/odom'
                odom.child_frame_id = 'motion_check/base_footprint'
                odom.pose.pose.orientation.w = 1.
                joints = JointState(); joints.header.stamp = stamp
                if case == 'stale_stamp':
                    joints.header.stamp.sec -= 1
                joints.name = names; joints.position = [0.]*len(names)
                timing = String()
                fault = case == 'timing_fault' and any(x != 0 for x, _ in commands)
                audit_stamp = stamp.sec+stamp.nanosec*1e-9-(1. if case == 'stale_audit' else 0.)
                timing.data = json.dumps(dict(valid=not fault, time_field='timestamp', stamp=audit_stamp))
                odom_pub.publish(odom); timing_pub.publish(timing)
                if case != 'feedback_loss' or not any(x != 0 for x, _ in commands):
                    joint_pub.publish(joints)
                executor.spin_once(timeout_sec=.025)
            assert process.poll() is not None, 'motion fixture timed out: '+case
            # Drain the final stop messages.
            end = time.monotonic()+.4
            while time.monotonic()<end:
                executor.spin_once(timeout_sec=.025)
            log.close()
            output = (root/(case+'.log')).read_text()
            if case == 'finite_success':
                assert process.returncode == 0, output
                assert any(x == .1 for x, _ in commands), 'finite plan produced no commands'
                assert sum(x != 0 for x, _ in commands) <= 11, 'commands exceeded 20 Hz'
            elif case in ('timing_fault', 'feedback_loss'):
                expected = ('invalid timed lidar input' if case == 'timing_fault'
                            else 'fresh laser velocity, actual feedback and timed points required')
                assert process.returncode == 2 and expected in output, output
                assert any(x == .1 for x, _ in commands), 'fault was not injected during execution'
            else:
                expected = dict(stale_stamp='stale measurement stamps',
                                stale_audit='stale lidar timing audit',
                                wrong_frame='laser velocity frames do not match',
                                competing_publisher='another command publisher exists')[case]
                assert process.returncode == 2 and expected in output, output
                assert not commands, 'refused preflight created a command publisher'
            if commands:
                assert commands[-1] == (0., 0.), 'exit did not publish zero velocity'
            results.append(dict(case=case, exit_code=process.returncode,
                                commands=len(commands), final_zero=bool(commands and commands[-1] == (0., 0.))))
            if competitor is not None:
                observer.destroy_publisher(competitor)
            process = None
        result = dict(test='motion_interlocks_synthetic_ros', cases=results,
                      hardware_motion=False, domain=188, logs=str(root))
        (root/'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--child':
        sys.exit(child(sys.argv[2]))
    main()
