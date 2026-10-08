from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from tianrover.config import Config


def generate_launch_description():
    cfg = Config()
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'), Node(
        package='tianbot_core', executable='tianbot_core', name='base',
        namespace=cfg.namespace, output='screen', parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'serial_port': cfg.get('BASE_PORT', '/dev/tianbot_base'),
            'serial_baudrate': int(cfg.get('BASE_BAUDRATE', '460800')),
            'type': 'tianrover', 'type_verify': True, 'publish_tf': True,
            'base_frame': cfg.frame('base_footprint'),
            'odom_frame': cfg.frame('odom'), 'imu_frame': cfg.frame('imu_link'),
        }])])
