from pathlib import Path
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,RegisterEventHandler,EmitEvent,LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianrover_common.config import Config
from tianrover_localization.launching import share


def setup(context):
    cfg=Config();sim=LaunchConfiguration('use_sim_time').perform(context)=='true'
    source=LaunchConfiguration('imu_source').perform(context)
    chassis=source=='chassis'
    imu_topic=cfg.get('FASTLIO_CHASSIS_IMU_TOPIC',cfg.topic('imu')) if chassis else cfg.get('FASTLIO_IMU_TOPIC',cfg.topic('zed2i/imu/data'))
    imu_frame=cfg.get('FASTLIO_CHASSIS_IMU_FRAME',cfg.frame('imu_link')) if chassis else cfg.get('FASTLIO_IMU_FRAME','zed2i_imu_link')
    calibration=LaunchConfiguration('calibration').perform(context) or cfg.get(
        'FASTLIO_CHASSIS_CALIBRATION' if chassis else 'FASTLIO_CALIBRATION',
        str(Path(share())/'config'/('fastlio_chassis_calibration.yaml' if chassis else 'fastlio_calibration.yaml')))
    temp=tempfile.NamedTemporaryFile(prefix='tianrover-fastlio-',suffix='.yaml',delete=False);temp.close()
    arguments=['--config',calibration,'--imu-source',source,'--imu-frame',imu_frame,'--defaults',
        cfg.get('FASTLIO_PARAMS',str(Path(share())/'config/fastlio.yaml')),'--output',temp.name]
    if LaunchConfiguration('allow_unverified').perform(context)=='true':arguments+=['--allow-unverified']
    if sim:arguments+=['--use-sim-time']
    prepare=Node(package='tianrover_localization',executable='fastlio_prepare',name='fastlio_prepare',
        namespace=cfg.namespace,output='screen',arguments=arguments)
    nodes=[Node(package='tianrover_localization',executable='tf_authority_guard',name='tf_authority_guard',
        namespace=cfg.namespace,output='screen',parameters=[{'use_sim_time':sim,
            'owned_odom_frame':cfg.frame('odom_lidar'),'base_frame':cfg.frame('odom'),
            'expected_node':'fastlio_output','check_any_parent':True}]),
        Node(package='tianrover_localization',executable='fastlio_io',name='fastlio_input',
        namespace=cfg.namespace,output='screen',parameters=[temp.name],
        remappings=[('points',cfg.get('POINTS_TOPIC',cfg.topic('points'))),
                    ('imu_input',imu_topic)]),
        Node(package='fast_lio',executable='fastlio_mapping',name='fastlio_mapping',
        namespace=cfg.namespace,output='screen',parameters=[temp.name],remappings=[
            ('/Odometry',cfg.topic('fastlio/odometry_raw')),('/cloud_registered_body',cfg.topic('fastlio/cloud_body')),
            ('/cloud_registered',cfg.topic('fastlio/cloud_world')),('/path',cfg.topic('fastlio/path')),
            ('/cloud_effected',cfg.topic('fastlio/cloud_effected')),('/Laser_map',cfg.topic('fastlio/map'))]),
        Node(package='tianrover_localization',executable='fastlio_io',name='fastlio_output',
        namespace=cfg.namespace,output='screen',parameters=[temp.name])]
    def ready(event,context):
        return nodes if event.returncode==0 else [LogInfo(msg='ERROR: FAST-LIO calibration/TF preparation failed'),EmitEvent(event=Shutdown(reason='FAST-LIO preflight failed'))]
    return [LogInfo(msg=f'FAST-LIO IMU source={source}, topic={imu_topic}, frame={imu_frame}, calibration={calibration}'),
            RegisterEventHandler(OnProcessExit(target_action=prepare,on_exit=ready)),prepare]


def generate_launch_description():
    cfg=Config()
    return LaunchDescription([DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('allow_unverified',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('imu_source',default_value=cfg.get('FASTLIO_IMU_SOURCE','zed'),choices=['zed','chassis']),
        DeclareLaunchArgument('calibration',default_value=''),
        OpaqueFunction(function=setup)])
