from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from tianrover_common.config import Config
from tianrover_common.launching import include, package_share


def share():
    return package_share('tianrover_bringup')


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    frontend = LaunchConfiguration('frontend').perform(context)
    fast = frontend == 'fastlio'
    if mode == 'bringup_3d':
        actions = [include(Path(package_share('tianrover_description') if part == 'description' else share())/'launch'/f'{part}.launch.py', use_sim_time=sim,
                        use_ekf='false', odometry_source='wheel' if fast else 'kiss')
                for part in ('description', 'base', 'lidar', 'scan')]
        if LaunchConfiguration('use_zed').perform(context) == 'true':
            actions.append(include(Path(share())/'launch/zed.launch.py', use_sim_time=sim))
        actions.append(include(Path(package_share('tianrover_localization'))/'launch/laser_odom.launch.py', use_sim_time=sim,
                    frontend=frontend, calibration=LaunchConfiguration('calibration').perform(context),
                    imu_source=LaunchConfiguration('imu_source').perform(context),
                    allow_unverified=LaunchConfiguration('allow_unverified').perform(context),
                    publish_tf='false' if fast else 'true',
                    odom_frame=cfg.frame('odom_lidar') if fast else cfg.frame('odom'),
                    output_topic=cfg.topic('odom_lidar') if fast else cfg.topic('odom')))
        return actions
    raise ValueError('Unsupported mode: ' + mode)


def entry(mode):
    cfg = Config()
    args = [DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
            DeclareLaunchArgument('frontend', default_value=cfg.get('3D_FRONTEND', 'kiss'), choices=['kiss','fastlio'])]
    if mode in {'bringup_3d','laser_odom'}:
        args += [DeclareLaunchArgument('allow_unverified',default_value='false',choices=['true','false']),
                 DeclareLaunchArgument('imu_source',default_value=cfg.get('FASTLIO_IMU_SOURCE','zed'),choices=['zed','chassis']),
                 DeclareLaunchArgument('calibration',default_value='')]
    if mode == 'bringup_3d':
        args.append(DeclareLaunchArgument('use_zed',default_value='true',choices=['true','false']))
    return LaunchDescription(args+[OpaqueFunction(function=lambda context: setup(context, mode))])
