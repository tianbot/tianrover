#!/usr/bin/env python3
"""Check TF publisher GIDs using real Humble nodes in an isolated domain."""
import os
os.environ['ROS_DOMAIN_ID']='186'
import signal,subprocess,time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def main():
    process=subprocess.Popen(['ros2','run','tianrover','tf_authority_guard','--ros-args','-r','__ns:=/mars',
                              '-p','owned_odom_frame:=mars/odom','-p','base_frame:=mars/base_footprint'],start_new_session=True)
    node=None;expected=None
    try:
        rclpy.init();node=Node('old_wheel_frontend',namespace='mars');values=[]
        node.create_subscription(Bool,'/mars/lidar/tf_authority_ok',lambda m:values.append(m.data),10)
        wrong=TransformBroadcaster(node)
        def spin(duration):
            end=time.monotonic()+duration
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        def broadcast(sender):
            msg=TransformStamped();msg.header.frame_id='mars/odom';msg.child_frame_id='mars/base_footprint'
            msg.header.stamp=node.get_clock().now().to_msg();msg.transform.rotation.w=1.;sender.sendTransform(msg)
        spin(3);assert values and values[-1]
        values.clear()
        for _ in range(15):broadcast(wrong);spin(.1)
        assert values and values[-1] is False,'conflicting wheel authority accepted'
        spin(2.5);assert values[-1] is True,'stopped publisher did not expire'
        expected=Node('kiss_icp_node',namespace='mars');right=TransformBroadcaster(expected)
        values.clear()
        for _ in range(15):broadcast(right);spin(.1)
        assert values and all(values),'expected KISS publisher rejected'
        print('TF authority guard: real GID conflict and recovery checks passed; no hardware')
    finally:
        if expected is not None:expected.destroy_node()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT);process.wait(timeout=10)


if __name__=='__main__':main()
