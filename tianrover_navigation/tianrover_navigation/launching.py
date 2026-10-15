from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover_common.config import Config, rewrite_parameters
from tianrover_common.launching import generated_yaml, package_share


def share():
    return package_share('tianrover_navigation')


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    ns = cfg.namespace
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
                costmap['footprint'] = costmap.get('footprint', '[[-0.75,-0.49],[-0.75,0.49],[0.75,0.49],[0.75,-0.49]]')
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
        path, cleanup = generated_yaml({ns: params} if ns else params)
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
    raise ValueError('Unsupported mode: ' + mode)


def entry(mode):
    cfg = Config()
    args = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    if mode == 'navigation':
        args.append(DeclareLaunchArgument('map', default_value=cfg.get('MAP', str(cfg.maps_dir / 'map.yaml'))))
    if mode == 'navigation_3d':
        args.append(DeclareLaunchArgument('frontend', default_value=cfg.get('3D_FRONTEND', 'kiss'), choices=['kiss', 'fastlio']))
    return LaunchDescription(args + [OpaqueFunction(function=lambda context: setup(context, mode))])
