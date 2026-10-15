from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from tianrover_common.launching import include


def generate_launch_description():
    return LaunchDescription([include(Path(get_package_share_directory('tianrover_gazebo')) / 'launch/gazebo.launch.py')])
