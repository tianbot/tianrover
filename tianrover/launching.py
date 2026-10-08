"""Stable entry points; hardware extensions live in discoverable adapters."""
import tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
                                    launch_arguments={k: str(v) for k, v in arguments.items()}.items())


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
    if mode == 'bringup':
        return [include(Path(share()) / 'launch' / (part + '.launch.py'), use_sim_time=sim)
                for part in ('description', 'base', 'lidar', 'scan')]
    if mode == 'scan':
        low = cfg.number('SCAN_MIN_HEIGHT', -0.1)
        high = cfg.number('SCAN_MAX_HEIGHT', 0.1)
        if low >= high:
            raise ValueError('SCAN_MIN_HEIGHT must be below SCAN_MAX_HEIGHT')
        return [Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                     name='pointcloud_to_laserscan', namespace=ns, output='screen',
                     parameters=[{'use_sim_time': sim, 'target_frame': cfg.frame('laser'),
                                  'transform_tolerance': 0.2, 'min_height': low, 'max_height': high,
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
        return [Node(package='nav2_map_server', executable='map_saver_cli',
                     name='map_saver', namespace=ns, output='screen',
                     parameters=[{'use_sim_time': sim, 'save_map_timeout': 10.0}],
                     arguments=['-t', cfg.topic('map'), '-f', str(filename)])]
    if mode == 'rviz':
        with open(Path(share()) / 'config/view.rviz') as stream:
            text = stream.read().replace('@MAP@', cfg.frame('map')).replace('@SCAN@', cfg.topic('scan'))
            text = text.replace('@MAP_TOPIC@', cfg.topic('map'))
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
    if mode == 'navigation':
        args.append(DeclareLaunchArgument('map', default_value=cfg.get('MAP', str(cfg.maps_dir / 'map.yaml'))))
    if mode == 'map_save':
        args.append(DeclareLaunchArgument('map_name', default_value=str(cfg.maps_dir / 'map')))
    return LaunchDescription(args + [OpaqueFunction(function=lambda context: setup(context, mode))])
