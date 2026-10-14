from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from tianrover_common.config import Config
from tianrover_common.launching import include, package_share


def share():
    return package_share('tianrover_navigation')


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    frontend = LaunchConfiguration('frontend').perform(context)
    fast = frontend == 'fastlio'
    if mode == 'localize_and_navigate_3d':
        return [include(Path(package_share('tianrover_localization'))/'launch/localization_3d.launch.py', use_sim_time=sim,
                        use_rgbd=LaunchConfiguration('use_rgbd').perform(context),
                        database=LaunchConfiguration('database').perform(context), frontend=frontend),
                include(Path(share())/'launch/navigation_3d.launch.py', use_sim_time=sim, frontend=frontend)]
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
