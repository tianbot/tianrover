"""Stable entry points; hardware extensions live in discoverable adapters."""
import tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from tianrover.config import Config, rewrite_parameters


def share():
    return get_package_share_directory('tianrover')


def generated_yaml(data):
    handle = tempfile.NamedTemporaryFile(mode='w', prefix='tianrover_', suffix='.yaml', delete=False)
    with handle:
        yaml.safe_dump(data, handle)
    def cleanup(context, *args, **kwargs):
        Path(handle.name).unlink(missing_ok=True)
        return []
    return handle.name, RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))


def include(path, **arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                                                      for k, v in arguments.items()}.items())


def component(kind, cfg, sim=False):
    defaults = {'base': 'tianbot_core', 'lidar': 'rslidar', 'description': 'mars'}
    path = cfg.component(share(), kind, defaults[kind])
    return [include(path, use_sim_time=sim)] if path else []


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    ns = cfg.namespace
    if mode in {'base', 'lidar', 'description'}:
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
                include(Path(get_package_share_directory('zed_wrapper')) / 'launch/zed_camera.launch.py',
                        camera_model='zed2i', camera_name='zed2i', namespace=cfg.namespace,
                        use_sim_time=sim, publish_tf=False, publish_map_tf=False, publish_imu_tf=True,
                        ros_params_override_path=cfg.get('ZED_PARAMS', str(Path(share()) / 'config/zed2i.yaml')))]
    if mode == 'bringup':
        actions = [include(Path(share()) / 'launch' / (part + '.launch.py'), use_sim_time=sim)
                   for part in ('description', 'base', 'lidar', 'scan')]
        if LaunchConfiguration('use_zed').perform(context).lower() == 'true':
            actions.append(include(Path(share()) / 'launch/zed.launch.py', use_sim_time=sim))
        if LaunchConfiguration('use_ekf').perform(context) == 'true':
            actions.append(include(Path(share()) / 'launch/ekf.launch.py', use_sim_time=sim,
                                   publish_tf='true', odom_topic=cfg.topic('odom_raw'),
                                   odom_frame=cfg.frame('odom'),
                                   filtered_topic=cfg.topic('odom')))
        return actions
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
        return [cleanup, Node(package='tianrover', executable='ekf_inputs', name='ekf_inputs',
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
    if mode == 'slam':
        path = cfg.get('SLAM_PARAMS', str(Path(share()) / 'config/slam.yaml'))
        with open(path) as stream:
            params = rewrite_parameters(yaml.safe_load(stream), cfg, sim)
        # Wildcard keeps parameters attached to a node in any namespace.
        path, cleanup = generated_yaml({'/**': params['slam_toolbox']})
        return [cleanup, Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                              name='slam_toolbox', namespace=ns, output='screen', parameters=[path],
                              remappings=[('/map', cfg.topic('map')),
                                          ('/map_metadata', cfg.topic('map_metadata'))])]
    if mode in {'navigation', 'navigation_3d'}:
        map_file = None
        if mode == 'navigation':
            map_file = Path(LaunchConfiguration('map').perform(context)).expanduser()
            if not map_file.is_file():
                raise FileNotFoundError(f'Map missing: {map_file}. Run slam.launch.py and map_save.launch.py first.')
        params_path = cfg.get('NAV_PARAMS', str(Path(share()) / 'config/nav2.yaml'))
        with open(params_path) as stream:
            params = rewrite_parameters(yaml.safe_load(stream), cfg, sim)
        follow = params['controller_server']['ros__parameters']['FollowPath']
        speed, yaw = cfg.number('MAX_SPEED', 0.2), cfg.number('MAX_YAW_RATE', 0.4)
        radius = cfg.number('ROBOT_RADIUS', 0.65)
        if min(speed, yaw, radius) <= 0:
            raise ValueError('Speed, yaw rate and robot radius must be positive')
        follow.update(max_vel_x=speed, max_speed_xy=speed, max_vel_theta=yaw)
        for key in ('local_costmap', 'global_costmap'):
            params[key][key]['ros__parameters']['robot_radius'] = radius
        smoother = params['velocity_smoother']['ros__parameters']
        smoother.update(max_velocity=[speed, 0.0, yaw], min_velocity=[-speed, 0.0, -yaw],
                        odom_topic=cfg.topic('odom'))
        params['behavior_server']['ros__parameters']['max_rotational_vel'] = yaw
        params['behavior_server']['ros__parameters']['min_rotational_vel'] = min(0.1, yaw)
        params['behavior_server']['ros__parameters']['rotational_acc_lim'] = 0.5
        if map_file is not None:
            params['map_server']['ros__parameters']['yaml_filename'] = str(map_file)
        params['map_server']['ros__parameters']['frame_id'] = cfg.frame('map')
        path, cleanup = generated_yaml({ns: params} if ns else params)
        # Keep /tf and /tf_static shared; isolate frames, not TF transport topics.
        # Namespaced costmap child nodes are matched by the nested parameter tree.
        executables = {
            'map_server': ('nav2_map_server', 'map_server'),
            'amcl': ('nav2_amcl', 'amcl'),
            'controller_server': ('nav2_controller', 'controller_server'),
            'smoother_server': ('nav2_smoother', 'smoother_server'),
            'planner_server': ('nav2_planner', 'planner_server'),
            'behavior_server': ('nav2_behaviors', 'behavior_server'),
            'bt_navigator': ('nav2_bt_navigator', 'bt_navigator'),
            'waypoint_follower': ('nav2_waypoint_follower', 'waypoint_follower'),
            'velocity_smoother': ('nav2_velocity_smoother', 'velocity_smoother'),
        }
        if mode == 'navigation_3d':
            # RTAB-Map owns map -> odom and the map topic. Do not launch AMCL.
            for name in ('amcl', 'map_server'):
                del executables[name]
            for key in ('local_costmap', 'global_costmap'):
                costmap = params[key][key]['ros__parameters']
                costmap.pop('robot_radius', None)
                # CAD neutral envelope plus commissioning margin. Steering/suspension
                # swept-volume and stopping-distance verification remain required.
                costmap['footprint'] = '[[-0.75,-0.49],[-0.75,0.49],[0.75,0.49],[0.75,-0.49]]'
                layer = costmap['voxel_layer' if key == 'local_costmap' else 'obstacle_layer']
                layer.pop('scan', None)
                layer['observation_sources'] = 'cloud'
                cloud_topic = cfg.topic('lidar/cloud_deskewed') if LaunchConfiguration('frontend').perform(context) == 'fastlio' else cfg.topic('kiss/frame')
                layer['cloud'] = {'topic': cfg.get('NAV_POINTS_TOPIC', cloud_topic), 'data_type': 'PointCloud2',
                                  'marking': True, 'clearing': True, 'min_obstacle_height': 0.15,
                                  'max_obstacle_height': 1.5, 'obstacle_min_range': 0.3,
                                  'obstacle_max_range': 8.0, 'raytrace_min_range': 0.3,
                                  'raytrace_max_range': 10.0}
                if key == 'local_costmap':
                    layer.update(z_resolution=0.1, z_voxels=16, max_obstacle_height=1.5)
        actions = [cleanup]
        for name, (package, executable) in executables.items():
            remaps = []
            if name in {'controller_server', 'behavior_server'}:
                remaps = [('cmd_vel', 'cmd_vel_nav')]
            if name == 'velocity_smoother':
                remaps = [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
            actions.append(Node(package=package, executable=executable, name=name,
                                namespace=ns, output='screen', parameters=[path], remappings=remaps))
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                            name='lifecycle_manager', namespace=ns, output='screen',
                            parameters=[{'use_sim_time': sim, 'autostart': True,
                                         'node_names': list(executables)}]))
        return actions
    if mode == 'map_save':
        filename = Path(LaunchConfiguration('map_name').perform(context)).expanduser()
        filename.parent.mkdir(parents=True, exist_ok=True)
        method = LaunchConfiguration('slam_methods').perform(context)
        if method == 'cartographer':
            return [Node(package='tianrover', executable='cartographer_map_save',
                         name='cartographer_map_save', namespace=ns, output='screen',
                         arguments=['--map-name', str(filename)])]
        return [Node(package='nav2_map_server', executable='map_saver_cli',
                     name='map_saver', namespace=ns, output='screen',
                     parameters=[{'use_sim_time': sim, 'save_map_timeout': 10.0}],
                     arguments=['-t', cfg.topic('map'), '-f', str(filename)])]
    if mode == 'rviz':
        with open(Path(share()) / 'config/view.rviz') as stream:
            text = stream.read().replace('@MAP@', LaunchConfiguration('fixed_frame').perform(context))
            text = text.replace('@SCAN@', cfg.topic('scan'))
            text = text.replace('@MAP_TOPIC@', cfg.topic('map'))
            text = text.replace('@DESCRIPTION_TOPIC@', cfg.topic('robot_description'))
            text = text.replace('@TF_PREFIX@', cfg.namespace)
        handle = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
        with handle:
            handle.write(text)
        def cleanup(context, *args, **kwargs):
            Path(handle.name).unlink(missing_ok=True)
            return []
        return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
                Node(package='rviz2', executable='rviz2', name='rviz', namespace=ns,
                     parameters=[{'use_sim_time': sim}], arguments=['-d', handle.name],
                     remappings=[('goal_pose', cfg.topic('goal_pose')),
                                 ('initialpose', cfg.topic('initialpose'))])]
    raise ValueError('Unknown entry point: ' + mode)


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
    if mode == 'ekf':
        args.extend([DeclareLaunchArgument('publish_tf', default_value='false', choices=['true', 'false']),
                     DeclareLaunchArgument('odom_frame', default_value=cfg.frame('odom_ekf')),
                     DeclareLaunchArgument('odom_topic', default_value=cfg.topic('odom')),
                     DeclareLaunchArgument('filtered_topic', default_value=cfg.topic('odometry/filtered'))])
    if mode == 'navigation':
        args.append(DeclareLaunchArgument('map', default_value=cfg.get('MAP', str(cfg.maps_dir / 'map.yaml'))))
    if mode == 'map_save':
        args.extend([DeclareLaunchArgument('slam_methods', default_value='gmapping',
                                           choices=['gmapping', 'cartographer', 'slam_toolbox']),
                     DeclareLaunchArgument('map_path', default_value=str(cfg.maps_dir)),
                     DeclareLaunchArgument('map_file', default_value=cfg.env.get('TIANBOT_MAP_FILE', 'map')),
                     DeclareLaunchArgument('map_name', default_value=PathJoinSubstitution([
                         LaunchConfiguration('map_path'), LaunchConfiguration('map_file')]))])
    if mode == 'navigation_3d':
        args.append(DeclareLaunchArgument('frontend', default_value=cfg.get('3D_FRONTEND', 'kiss'), choices=['kiss', 'fastlio']))
    if mode == 'rviz':
        args.append(DeclareLaunchArgument('fixed_frame', default_value=cfg.frame('map')))
    return LaunchDescription(args + [OpaqueFunction(function=lambda context: setup(context, mode))])
