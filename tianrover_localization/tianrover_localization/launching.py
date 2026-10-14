from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover_common.config import Config
from tianrover_common.launching import generated_yaml, package_share


def share():
    return package_share('tianrover_localization')


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    ns = cfg.namespace
    if mode == 'ekf':
        from rclpy.expand_topic_name import expand_topic_name
        odom_topic = expand_topic_name(LaunchConfiguration('odom_topic').perform(context),
                                      'ekf_inputs', '/' + ns)
        output_topic = expand_topic_name(LaunchConfiguration('filtered_topic').perform(context),
                                        'ekf_filter_node', '/' + ns)
        if odom_topic == output_topic:
            raise ValueError('EKF input and output topics must differ')
        with open(cfg.get('EKF_PARAMS', str(Path(share()) / 'config/ekf.yaml'))) as stream:
            params = yaml.safe_load(stream)['ekf_filter_node']['ros__parameters']
        odom_frame = LaunchConfiguration('odom_frame').perform(context)
        params.update(use_sim_time=sim,
                      publish_tf=LaunchConfiguration('publish_tf').perform(context) == 'true',
                      map_frame=cfg.frame('map'), odom_frame=odom_frame,
                      base_link_frame=cfg.frame('base_footprint'), world_frame=odom_frame,
                      odom0=cfg.topic('ekf/wheel'), imu0=cfg.topic('ekf/imu'))
        path, cleanup = generated_yaml({'/**': {'ros__parameters': params}})
        return [cleanup, Node(package='tianrover_localization', executable='ekf_inputs', name='ekf_inputs',
                              namespace=ns, output='screen', parameters=[{
                                  'use_sim_time': sim, 'odom_topic': odom_topic,
                                  'imu_topic': cfg.get('EKF_IMU_TOPIC', cfg.topic('imu')),
                                  'base_frame': cfg.frame('base_footprint'),
                                  'imu_frame': cfg.frame('imu_link'),
                                  'velocity_variance': cfg.number('EKF_VELOCITY_VARIANCE', 0.0025),
                                  'yaw_variance': cfg.number('EKF_YAW_VARIANCE', 0.0025)}]),
                Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                     namespace=ns, output='screen', parameters=[path],
                     remappings=[('odometry/filtered', output_topic)])]
    raise ValueError('Unsupported mode: ' + mode)


def entry(mode):
    cfg = Config()
    args = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    if mode == 'ekf':
        args.extend([DeclareLaunchArgument('publish_tf', default_value='false', choices=['true', 'false']),
                     DeclareLaunchArgument('odom_frame', default_value=cfg.frame('odom_ekf')),
                     DeclareLaunchArgument('odom_topic', default_value=cfg.topic('odom')),
                     DeclareLaunchArgument('filtered_topic', default_value=cfg.topic('odometry/filtered'))])
    return LaunchDescription(args + [OpaqueFunction(function=lambda context: setup(context, mode))])
