"""Select one laser frontend and IMU source, retaining wheel odometry."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover.config import Config
from tianrover.launching import include, share


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    frontend = LaunchConfiguration('frontend').perform(context)
    fast = frontend == 'fastlio'
    if mode == 'bringup_3d':
        actions = [include(Path(share())/'launch'/f'{part}.launch.py', use_sim_time=sim,
                        use_ekf='false', odometry_source='wheel' if fast else 'kiss')
                for part in ('description', 'base', 'lidar', 'scan')]
        if LaunchConfiguration('use_zed').perform(context) == 'true':
            actions.append(include(Path(share())/'launch/zed.launch.py', use_sim_time=sim))
        actions.append(include(Path(share())/'launch/laser_odom.launch.py', use_sim_time=sim,
                    frontend=frontend, calibration=LaunchConfiguration('calibration').perform(context),
                    imu_source=LaunchConfiguration('imu_source').perform(context),
                    allow_unverified=LaunchConfiguration('allow_unverified').perform(context),
                    publish_tf='false' if fast else 'true',
                    odom_frame=cfg.frame('odom_lidar') if fast else cfg.frame('odom'),
                    output_topic=cfg.topic('odom_lidar') if fast else cfg.topic('odom')))
        return actions
    if mode == 'localize_and_navigate_3d':
        return [include(Path(share())/'launch/localization_3d.launch.py', use_sim_time=sim,
                        use_rgbd=LaunchConfiguration('use_rgbd').perform(context),
                        database=LaunchConfiguration('database').perform(context), frontend=frontend),
                include(Path(share())/'launch/navigation_3d.launch.py', use_sim_time=sim, frontend=frontend)]
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
        return [Node(package='tianrover', executable='tf_authority_guard', name='tf_authority_guard',
                     namespace=cfg.namespace, output='screen', parameters=[{
                         'owned_odom_frame': frame if publish_tf else '', 'base_frame': cfg.frame('base_footprint')}]),
                Node(package='tianrover', executable='lidar_cloud_gate', name='lidar_cloud_gate',
                     namespace=cfg.namespace, output='screen', parameters=[{
                         'use_sim_time': sim, 'require_time': require_time,
                         'reliable_cloud': params.get('reliable_cloud', True),
                         'laser_frame': cfg.frame('laser'), 'base_frame': cfg.frame('base_footprint'),
                         'owned_odom_frame': frame if publish_tf else ''}],
                     remappings=[('points', cfg.get('POINTS_TOPIC', cfg.topic('points')))]),
                Node(package='kiss_icp', executable='kiss_icp_node', name='kiss_icp_node',
                     namespace=cfg.namespace, output='screen', parameters=[params],
                     remappings=[('pointcloud_topic', 'points_checked')]),
                Node(package='tianrover', executable='lidar_velocity', name='lidar_velocity',
                     namespace=cfg.namespace, output='screen', parameters=[{'use_sim_time': sim}],
                     remappings=[('odom', LaunchConfiguration('output_topic'))])]
    if mode in {'mapping_3d', 'localization_3d'}:
        database = Path(LaunchConfiguration('database').perform(context)).expanduser().resolve()
        if mode == 'localization_3d' and not database.is_file():
            raise FileNotFoundError('Saved RTAB-Map database missing: '+str(database))
        if mode == 'localization_3d':
            from tianrover.map_database import inspect_database
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
    raise ValueError('Unknown 3D mode: '+mode)


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
    if mode in {'mapping_3d','localization_3d','localize_and_navigate_3d'}:
        args.append(DeclareLaunchArgument('use_rgbd',default_value='true',choices=['true','false']))
    if mode == 'laser_odom':
        args += [DeclareLaunchArgument('publish_tf', default_value='false', choices=['true', 'false']),
                 DeclareLaunchArgument('require_point_time', default_value='true', choices=['true', 'false']),
                 DeclareLaunchArgument('odom_frame', default_value=cfg.frame('odom_lidar')),
                 DeclareLaunchArgument('output_topic', default_value=cfg.topic('odom_lidar'))]
    if mode in {'mapping_3d', 'localization_3d', 'localize_and_navigate_3d'}:
        args.append(DeclareLaunchArgument('database',
                    default_value=str(cfg.maps_dir/'rtabmap.db')))
    return LaunchDescription(args+[OpaqueFunction(function=lambda context: setup(context, mode))])
