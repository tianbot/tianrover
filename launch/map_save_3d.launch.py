from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover.config import Config


def setup(context):
    arguments = ['--database', LaunchConfiguration('database').perform(context),
                 '--output', LaunchConfiguration('output').perform(context)]
    if LaunchConfiguration('offline').perform(context).lower() == 'true':
        arguments.append('--offline')
    return [Node(package='tianrover', executable='map_save_3d', output='screen',
                 arguments=arguments)]


def generate_launch_description():
    cfg = Config()
    return LaunchDescription([
        DeclareLaunchArgument('database', default_value=str(cfg.maps_dir / 'rtabmap.db')),
        DeclareLaunchArgument('output', description='New saved RTAB-Map database path'),
        DeclareLaunchArgument('offline', default_value='true', choices=['true', 'false'],
                              description='Save after RTAB-Map exits; live backup is disabled.'),
        OpaqueFunction(function=setup)])
