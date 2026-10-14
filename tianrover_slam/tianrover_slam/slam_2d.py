"""Gmapping and Cartographer entry points using the existing scan/odom/TF contract."""
from pathlib import Path
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tianrover_common.config import Config
from tianrover_common.launching import generated_yaml, include
from tianrover_slam.launching import share


def cartographer_config(cfg, localization=False):
    source = Path(cfg.get('CARTOGRAPHER_CONFIG', str(Path(share())/'config/cartographer.lua')))
    text = source.expanduser().read_text()
    for key, frame in [('MAP_FRAME', 'map'), ('TRACKING_FRAME', 'base_footprint'),
                       ('PUBLISHED_FRAME', 'odom'), ('ODOM_FRAME', 'odom')]:
        text = text.replace('@'+key+'@', cfg.frame(frame))
    if localization:
        text = text.replace('-- @LOCALIZATION@',
                            'TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }')
    return text


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    if mode == 'gmapping':
        with open(cfg.get('GMAPPING_PARAMS', str(Path(share())/'config/gmapping.yaml'))) as stream:
            params = yaml.safe_load(stream)
        params.update(use_sim_time=sim, base_frame=cfg.frame('base_footprint'),
                      map_frame=cfg.frame('map'), odom_frame=cfg.frame('odom'),
                      scan_topic=cfg.get('SCAN_TOPIC', cfg.topic('scan')))
        path, cleanup = generated_yaml({'/**': {'ros__parameters': params}})
        return [cleanup, Node(package='slam_gmapping', executable='slam_gmapping',
                              name='slam_gmapping', namespace=cfg.namespace, output='screen', parameters=[path])]
    if mode == 'slam_selector':
        method = LaunchConfiguration('slam_methods').perform(context)
        entry = 'slam.launch.py' if method == 'slam_toolbox' else method+'.launch.py'
        return [include(Path(share())/'launch'/entry, use_sim_time=sim)]
    localization = mode == 'cartographer_localization'
    if mode not in {'cartographer', 'cartographer_localization'}:
        raise ValueError('unknown 2D mapping mode: '+mode)
    upstream = Path(get_package_share_directory('cartographer_ros'))/'configuration_files'
    handle = tempfile.TemporaryDirectory(prefix='tianrover_cartographer_')
    directory = Path(handle.name)
    # Upstream Lua includes must remain available beside the generated config.
    for source in upstream.glob('*.lua'):
        (directory/source.name).symlink_to(source)
    config = directory/'tianrover.lua'
    config.write_text(cartographer_config(cfg, localization))
    def cleanup(context, *args, **kwargs):
        handle.cleanup()
        return []
    arguments = ['-configuration_directory', str(directory), '-configuration_basename', config.name]
    if localization:
        state = Path(LaunchConfiguration('state').perform(context)).expanduser().resolve()
        if not state.is_file() or state.stat().st_size == 0:
            raise FileNotFoundError('Cartographer pbstream missing or empty: '+str(state))
        arguments += ['-load_state_filename', str(state), '-load_frozen_state', 'true']
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
                 namespace=cfg.namespace, output='screen', parameters=[{'use_sim_time': sim}],
                 arguments=arguments, remappings=[('scan', cfg.get('SCAN_TOPIC', cfg.topic('scan'))),
                                                 ('odom', cfg.topic('odom'))]),
            Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                 name='cartographer_occupancy_grid_node', namespace=cfg.namespace, output='screen',
                 parameters=[{'use_sim_time': sim}], arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])]


def entry(mode):
    cfg = Config()
    arguments = [DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'])]
    if mode == 'cartographer_localization':
        arguments.append(DeclareLaunchArgument('state', default_value=str(cfg.maps_dir/'map.pbstream')))
    if mode == 'slam_selector':
        arguments.append(DeclareLaunchArgument('slam_methods', default_value='gmapping',
                                              choices=['gmapping', 'cartographer', 'slam_toolbox']))
    return LaunchDescription(arguments+[OpaqueFunction(function=lambda context: setup(context, mode))])
