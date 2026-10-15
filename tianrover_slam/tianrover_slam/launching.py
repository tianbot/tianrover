from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from tianrover_common.config import Config, rewrite_parameters
from tianrover_common.launching import generated_yaml, package_share


def share():
    return package_share('tianrover_slam')


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    ns = cfg.namespace
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
    if mode == 'map_save':
        filename = Path(LaunchConfiguration('map_name').perform(context)).expanduser()
        filename.parent.mkdir(parents=True, exist_ok=True)
        method = LaunchConfiguration('slam_methods').perform(context)
        if method == 'cartographer':
            return [Node(package='tianrover_slam', executable='cartographer_map_save',
                         name='cartographer_map_save', namespace=ns, output='screen',
                         arguments=['--map-name', str(filename)])]
        return [Node(package='nav2_map_server', executable='map_saver_cli',
                     name='map_saver', namespace=ns, output='screen',
                     parameters=[{'use_sim_time': sim, 'save_map_timeout': 10.0,
                                  'free_thresh_default': cfg.number('MAP_FREE_THRESH', 0.196)}],
                     arguments=['-t', cfg.topic('map'), '-f', str(filename)])]
    raise ValueError('Unsupported mode: ' + mode)


def entry(mode):
    cfg = Config()
    args = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    if mode == 'map_save':
        args.extend([DeclareLaunchArgument('slam_methods', default_value='gmapping',
                                           choices=['gmapping', 'cartographer', 'slam_toolbox']),
                     DeclareLaunchArgument('map_path', default_value=str(cfg.maps_dir)),
                     DeclareLaunchArgument('map_file', default_value=cfg.env.get('TIANBOT_MAP_FILE', 'map')),
                     DeclareLaunchArgument('map_name', default_value=PathJoinSubstitution([
                         LaunchConfiguration('map_path'), LaunchConfiguration('map_file')]))])
    return LaunchDescription(args + [OpaqueFunction(function=lambda context: setup(context, mode))])
