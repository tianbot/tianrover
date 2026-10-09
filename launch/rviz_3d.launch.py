from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from tianrover.config import Config
from tianrover.launching import share,generated_yaml
import yaml


def generate_launch_description():
    cfg=Config();data=yaml.safe_load((Path(share())/'rviz/fastlio.rviz').read_text())
    def replace(x):
        if isinstance(x,dict):return {k:replace(v) for k,v in x.items()}
        if isinstance(x,list):return [replace(v) for v in x]
        if isinstance(x,str) and x.startswith('/mars/'):return cfg.topic(x[6:])
        if isinstance(x,str) and x.startswith('mars/'):return cfg.frame(x[5:])
        return x
    path,cleanup=generated_yaml(replace(data))
    return LaunchDescription([cleanup,Node(package='rviz2',executable='rviz2',
        namespace=cfg.namespace,arguments=['-d',path],output='screen')])
