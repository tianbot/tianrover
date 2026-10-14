from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover_common.config import Config
from tianrover_common.launching import include, package_share


def share():
    return package_share('tianrover_bringup')


def component(kind, cfg, sim=False):
    defaults = {'base': 'tianbot_core', 'lidar': 'rslidar', 'description': 'mars'}
    path = cfg.component(package_share('tianrover_description') if kind == 'description' else share(), kind, defaults[kind])
    return [include(path, use_sim_time=sim)] if path else []


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    ns = cfg.namespace
    if mode in {'base', 'lidar'}:
        return component(mode, cfg, sim)
    if mode == 'zed_mount':
        arguments = []
        for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            arguments.extend(['--' + axis, str(cfg.number('ZED_MOUNT_' + axis.upper(), 0.0))])
        arguments.extend(['--frame-id', cfg.frame('zed_camera_link'),
                          '--child-frame-id', 'zed2i_camera_link'])
        return [Node(package='tf2_ros', executable='static_transform_publisher',
                     name='zed_mount_tf', namespace=ns, output='screen',
                     parameters=[{'use_sim_time': sim}], arguments=arguments)]
    if mode == 'zed':
        return [include(Path(share()) / 'launch/zed_mount.launch.py', use_sim_time=sim),
                include(Path(package_share('zed_wrapper')) / 'launch/zed_camera.launch.py',
                        camera_model='zed2i', camera_name='zed2i', namespace=cfg.namespace,
                        use_sim_time=sim, publish_tf=False, publish_map_tf=False, publish_imu_tf=True,
                        ros_params_override_path=cfg.get('ZED_PARAMS', str(Path(share()) / 'config/zed2i.yaml')))]
    if mode == 'bringup':
        actions = [include(Path(package_share('tianrover_description') if part == 'description' else share()) / 'launch' / (part + '.launch.py'), use_sim_time=sim)
                   for part in ('description', 'base', 'lidar', 'scan')]
        if LaunchConfiguration('use_zed').perform(context).lower() == 'true':
            actions.append(include(Path(share()) / 'launch/zed.launch.py', use_sim_time=sim))
        if LaunchConfiguration('use_ekf').perform(context) == 'true':
            actions.append(include(Path(package_share('tianrover_localization')) / 'launch/ekf.launch.py', use_sim_time=sim,
                                   publish_tf='true', odom_topic=cfg.topic('odom_raw'),
                                   odom_frame=cfg.frame('odom'),
                                   filtered_topic=cfg.topic('odom')))
        return actions
    if mode == 'scan':
        low = cfg.number('SCAN_MIN_HEIGHT', -0.1)
        high = cfg.number('SCAN_MAX_HEIGHT', 0.1)
        if low >= high:
            raise ValueError('SCAN_MIN_HEIGHT must be below SCAN_MAX_HEIGHT')
        return [Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                     name='pointcloud_to_laserscan', namespace=ns, output='screen',
                     parameters=[{'use_sim_time': sim, 'target_frame': cfg.frame('laser'),
                                  'transform_tolerance': 0.2, 'min_height': low, 'max_height': high,
                                  'reliable_cloud': cfg.get('RELIABLE_CLOUD', 'true').lower() == 'true',
                                  'angle_min': -3.141592653589793, 'angle_max': 3.141592653589793,
                                  'angle_increment': 0.008726646259971648, 'scan_time': 0.1,
                                  'range_min': 0.3, 'range_max': cfg.number('SCAN_RANGE', 20.0),
                                  'use_inf': True}],
                     remappings=[('cloud_in', cfg.get('POINTS_TOPIC', cfg.topic('points'))),
                                 ('scan', 'scan')])]
    raise ValueError('Unsupported mode: ' + mode)


def entry(mode):
    cfg = Config()
    args = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    if mode in {'description', 'bringup'}:
        args.append(DeclareLaunchArgument('use_joint_state_publisher', default_value='true',
                    description='Preview zero poses; set false when hardware publishes joint_states.'))
    if mode in {'base', 'bringup'}:
        args.append(DeclareLaunchArgument('use_ekf', default_value=cfg.get('USE_EKF', 'false'),
                    choices=['true', 'false'], description='Use EKF odometry; disable driver TF and retain odom_raw.'))
    if mode == 'bringup':
        args.append(DeclareLaunchArgument('use_zed', default_value=cfg.get('USE_ZED', 'true'),
                    choices=['true', 'false'], description='Start the ZED2i camera.'))
    if mode == 'base':
        args.append(DeclareLaunchArgument('odometry_source', default_value='wheel', choices=['wheel', 'kiss']))
    return LaunchDescription(args + [OpaqueFunction(function=lambda context: setup(context, mode))])
