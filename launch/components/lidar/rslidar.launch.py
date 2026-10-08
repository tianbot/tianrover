import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from tianrover.config import Config
from tianrover.launching import generated_yaml, share
from pathlib import Path


def generate_launch_description():
    cfg = Config()
    with open(cfg.get('LIDAR_CONFIG', str(Path(share()) / 'config/rslidar.yaml'))) as stream:
        data = yaml.safe_load(stream)
    for lidar in data['lidar']:
        lidar['driver'].update(lidar_type=cfg.get('LIDAR_MODEL', 'RSHELIOS_16P'),
                               host_address=cfg.get('LIDAR_HOST', '0.0.0.0'),
                               msop_port=int(cfg.get('LIDAR_MSOP_PORT', '6699')),
                               difop_port=int(cfg.get('LIDAR_DIFOP_PORT', '7788')))
        lidar['ros'].update(ros_frame_id=cfg.frame('laser'),
                            ros_send_point_cloud_topic=cfg.topic('points'),
                            ros_recv_packet_topic=cfg.topic('rslidar_packets'),
                            ros_send_packet_topic=cfg.topic('rslidar_packets'))
    path, cleanup = generated_yaml(data)
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'), cleanup, Node(
        package='rslidar_sdk', executable='rslidar_sdk_node', name='lidar',
        namespace=cfg.namespace, output='screen', parameters=[{'config_path': path}])])
