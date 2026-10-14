from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    args=[]
    if LaunchConfiguration('execute').perform(context)=='true':
        args=['--execute','--profile',LaunchConfiguration('profile').perform(context)]
        if LaunchConfiguration('clearance_verified').perform(context)=='true':
            args.append('--clearance-verified')
    return [Node(package='tianrover_calibration',executable='calibration_motion',output='screen',arguments=args)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('execute',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('profile',default_value=''),
        DeclareLaunchArgument('clearance_verified',default_value='false',choices=['true','false']),
        OpaqueFunction(function=setup)])
