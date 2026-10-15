"""TianRover Mini physical simulation, migrated from the ROS 1 package."""
import os
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover_common.config import Config
from tianrover_common.launching import include


def setup(context):
    cfg = Config({'TIANBOT_NAME': LaunchConfiguration('namespace').perform(context)})
    share = Path(get_package_share_directory('tianrover_gazebo'))
    prefix = cfg.namespace + '/' if cfg.namespace else ''
    description = xacro.process_file(str(share / 'urdf/tianrover_mini_continuous.xacro'),
        mappings={'namespace': cfg.namespace, 'frame_prefix': prefix,
                  'enable_skid_rate': LaunchConfiguration('enable_skid_rate').perform(context)}).toxml()
    # Gazebo resolves collision meshes independently of the ROS package index.
    description = description.replace('package://tianrover_gazebo/', share.as_uri() + '/')
    model_paths = [str(share / 'meshes'), '/usr/share/gazebo-11/models', os.environ.get('GAZEBO_MODEL_PATH', '')]
    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_paths)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', 'file://' + str(share / 'meshes')),
        include(Path(get_package_share_directory('gazebo_ros')) / 'launch/gazebo.launch.py',
                world=LaunchConfiguration('world').perform(context),
                gui=LaunchConfiguration('gui').perform(context),
                verbose=LaunchConfiguration('verbose').perform(context),
                pause=LaunchConfiguration('pause').perform(context)),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=cfg.namespace,
             parameters=[{'robot_description': description, 'frame_prefix': prefix, 'use_sim_time': True}],
             output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', namespace=cfg.namespace,
             arguments=['-entity', LaunchConfiguration('entity'), '-topic', cfg.topic('robot_description'),
                        '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                        '-z', LaunchConfiguration('spawn_z'), '-Y', LaunchConfiguration('spawn_yaw')],
             output='screen')]


def generate_launch_description():
    share = Path(get_package_share_directory('tianrover_gazebo'))
    defaults = {'namespace': Config().namespace, 'entity': 'tianrover_mini',
                'gui': 'true', 'verbose': 'false', 'pause': 'false',
                'world': str(share / 'urdf/Obstacle.world'), 'enable_skid_rate': 'false',
                'spawn_x': '1', 'spawn_y': '4', 'spawn_z': '0.05', 'spawn_yaw': '0'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
