from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from tianrover_common.config import Config
from tianrover_common.launching import include, package_share


def setup(context):
    cfg = Config()
    path = cfg.component(package_share('tianrover_description'), 'description', 'mars')
    return [include(path, use_sim_time=LaunchConfiguration('use_sim_time').perform(context),
                    use_joint_state_publisher=LaunchConfiguration('use_joint_state_publisher').perform(context))] if path else []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup)])
