from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover_common.config import Config
from tianrover_common.launching import include, package_share


def share():
    return package_share('tianrover_localization')


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    frontend = LaunchConfiguration('frontend').perform(context)
    fast = frontend == 'fastlio'
    if mode == 'laser_odom':
        if fast:
            return [include(Path(share())/'launch/fastlio.launch.py', use_sim_time=sim,
                imu_source=LaunchConfiguration('imu_source').perform(context),
                calibration=LaunchConfiguration('calibration').perform(context),
                allow_unverified=LaunchConfiguration('allow_unverified').perform(context))]
        require_time = LaunchConfiguration('require_point_time').perform(context) == 'true'
        publish_tf = LaunchConfiguration('publish_tf').perform(context) == 'true'
        frame = LaunchConfiguration('odom_frame').perform(context)
        with open(cfg.get('KISS_PARAMS', str(Path(share())/'config/kiss.yaml'))) as stream:
            params = yaml.safe_load(stream)
        params.update(use_sim_time=sim, base_frame=cfg.frame('base_footprint'),
                      lidar_odom_frame=frame, publish_odom_tf=publish_tf, invert_odom_tf=False)
        if not require_time:
            params['data.deskew'] = False
        return [Node(package='tianrover_localization', executable='tf_authority_guard', name='tf_authority_guard',
                     namespace=cfg.namespace, output='screen', parameters=[{
                         'owned_odom_frame': frame if publish_tf else '', 'base_frame': cfg.frame('base_footprint')}]),
                Node(package='tianrover_localization', executable='lidar_cloud_gate', name='lidar_cloud_gate',
                     namespace=cfg.namespace, output='screen', parameters=[{
                         'use_sim_time': sim, 'require_time': require_time,
                         'reliable_cloud': params.get('reliable_cloud', True),
                         'laser_frame': cfg.frame('laser'), 'base_frame': cfg.frame('base_footprint'),
                         'owned_odom_frame': frame if publish_tf else ''}],
                     remappings=[('points', cfg.get('POINTS_TOPIC', cfg.topic('points')))]),
                Node(package='kiss_icp', executable='kiss_icp_node', name='kiss_icp_node',
                     namespace=cfg.namespace, output='screen', parameters=[params],
                     remappings=[('pointcloud_topic', 'points_checked')]),
                Node(package='tianrover_localization', executable='lidar_velocity', name='lidar_velocity',
                     namespace=cfg.namespace, output='screen', parameters=[{'use_sim_time': sim}],
                     remappings=[('odom', LaunchConfiguration('output_topic'))])]
    raise ValueError('Unsupported mode: ' + mode)


def entry(mode):
    cfg = Config()
    args = [DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
            DeclareLaunchArgument('frontend', default_value=cfg.get('3D_FRONTEND', 'kiss'), choices=['kiss','fastlio'])]
    if mode in {'bringup_3d','laser_odom'}:
        args += [DeclareLaunchArgument('allow_unverified',default_value='false',choices=['true','false']),
                 DeclareLaunchArgument('imu_source',default_value=cfg.get('FASTLIO_IMU_SOURCE','zed'),choices=['zed','chassis']),
                 DeclareLaunchArgument('calibration',default_value='')]
    if mode == 'laser_odom':
        args += [DeclareLaunchArgument('publish_tf', default_value='false', choices=['true', 'false']),
                 DeclareLaunchArgument('require_point_time', default_value='true', choices=['true', 'false']),
                 DeclareLaunchArgument('odom_frame', default_value=cfg.frame('odom_lidar')),
                 DeclareLaunchArgument('output_topic', default_value=cfg.topic('odom_lidar'))]
    return LaunchDescription(args+[OpaqueFunction(function=lambda context: setup(context, mode))])
