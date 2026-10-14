import tempfile
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover_common.config import Config
from tianrover_common.launching import package_share


def share():
    return package_share('tianrover_rviz')


def setup(context, mode):
    cfg = Config()
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    ns = cfg.namespace
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
    raise ValueError('Unsupported mode: ' + mode)


def entry(mode):
    cfg = Config()
    args = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    if mode == 'rviz':
        args.append(DeclareLaunchArgument('fixed_frame', default_value=cfg.frame('map')))
    return LaunchDescription(args + [OpaqueFunction(function=lambda context: setup(context, mode))])
