from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover_common.config import Config
from tianrover_common.launching import package_share


def share():
    return package_share('tianrover_slam')


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    frontend = LaunchConfiguration('frontend').perform(context)
    fast = frontend == 'fastlio'
    if mode in {'mapping_3d', 'localization_3d'}:
        database = Path(LaunchConfiguration('database').perform(context)).expanduser().resolve()
        if mode == 'localization_3d' and not database.is_file():
            raise FileNotFoundError('Saved RTAB-Map database missing: '+str(database))
        if mode == 'localization_3d':
            from tianrover_slam.map_database import inspect_database
            inspect_database(database)
        database.parent.mkdir(parents=True, exist_ok=True)
        with open(cfg.get('RTABMAP_PARAMS', str(Path(share())/'config/rtabmap.yaml'))) as stream:
            params = yaml.safe_load(stream)
        params.update(use_sim_time=sim, frame_id=cfg.frame('base_link') if fast else cfg.frame('base_footprint'),
                      map_frame_id=cfg.frame('map'), odom_frame_id='', database_path=str(database))
        # The selected frontend supplies continuous odometry and an already
        # deskewed local-frame scan. No second ICP odometry/deskew node is launched.
        rgbd = LaunchConfiguration('use_rgbd').perform(context) == 'true'
        params.update(subscribe_rgb=rgbd, subscribe_depth=rgbd, subscribe_rgbd=False,
                      qos_image=2, qos_camera_info=2)
        if fast:
            params['approx_sync_max_interval'] = 0.1
        params['Mem/IncrementalMemory'] = 'false' if mode == 'localization_3d' else 'true'
        params['Mem/InitWMWithAllNodes'] = 'true' if mode == 'localization_3d' else 'false'
        return [Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
                     namespace=cfg.namespace, output='screen', parameters=[params],
                     remappings=[('odom', 'odom_lidar' if fast else 'kiss/odometry'),
                                 ('scan_cloud', 'lidar/cloud_deskewed' if fast else 'kiss/frame'),
                                 ('rgb/image', cfg.get('ZED_RGB_TOPIC', cfg.topic('zed2i/rgb/color/rect/image'))),
                                 ('depth/image', cfg.get('ZED_DEPTH_TOPIC', cfg.topic('zed2i/depth/depth_registered'))),
                                 ('rgb/camera_info', cfg.get('ZED_INFO_TOPIC', cfg.topic('zed2i/rgb/color/rect/camera_info'))),
                                 ('grid_map', 'map'), ('imu', 'unused_imu')])]
    raise ValueError('Unsupported mode: ' + mode)


def entry(mode):
    cfg = Config()
    args = [DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
            DeclareLaunchArgument('frontend', default_value=cfg.get('3D_FRONTEND', 'kiss'), choices=['kiss','fastlio'])]
    if mode in {'mapping_3d','localization_3d','localize_and_navigate_3d'}:
        args.append(DeclareLaunchArgument('use_rgbd',default_value='true',choices=['true','false']))
    if mode in {'mapping_3d', 'localization_3d', 'localize_and_navigate_3d'}:
        args.append(DeclareLaunchArgument('database',
                    default_value=str(cfg.maps_dir/'rtabmap.db')))
    return LaunchDescription(args+[OpaqueFunction(function=lambda context: setup(context, mode))])
