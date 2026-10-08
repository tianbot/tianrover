from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from tianrover.config import Config
from tianrover.launching import share


def generate_launch_description():
    cfg = Config()
    path = Path(cfg.get('URDF', str(Path(share()) / 'urdf/mars.urdf'))).expanduser()
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', namespace=cfg.namespace,
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool), 'robot_description': path.read_text(),
                          'frame_prefix': cfg.namespace + '/' if cfg.namespace else ''}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', namespace=cfg.namespace,
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool), 'rate': 10}]),
    ])
