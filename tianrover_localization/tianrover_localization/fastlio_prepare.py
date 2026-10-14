"""Resolve explicit calibration or opt-in nominal TF before starting FAST-LIO."""
import argparse
import copy
import math
import time
from pathlib import Path
import numpy as np
import yaml
from tianrover_common.config import Config
from tianrover_localization.fastlio_io import matrix


def main(args=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener,TransformException
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--defaults',required=True);parser.add_argument('--allow-unverified',action='store_true')
    parser.add_argument('--use-sim-time',action='store_true')
    parser.add_argument('--imu-source',choices=['zed','chassis'],default='zed')
    parser.add_argument('--imu-frame')
    opts=parser.parse_args(remove_ros_args(args=args)[1:])
    cfg=Config();cal=yaml.safe_load(Path(opts.config).expanduser().read_text())
    if cal.get('imu_source',opts.imu_source)!=opts.imu_source:
        raise ValueError('calibration IMU source does not match selected imu_source')
    confirmed=cal.get('calibration_confirmed',False)
    if not isinstance(confirmed,bool):raise ValueError('calibration_confirmed must be a YAML boolean')
    gap=float(cal.get('max_imu_gap',.1))
    if not math.isfinite(gap) or not 0<gap<=1:raise ValueError('max_imu_gap must be finite and in (0,1] seconds')
    if not confirmed and not opts.allow_unverified:
        raise ValueError('FAST-LIO external extrinsics/time offset are pending calibration; supply calibrated YAML or explicitly allow_unverified:=true for preliminary evaluation')
    offset=cal.get('time_offset_lidar_to_imu')
    if offset is None:
        if confirmed:raise ValueError('confirmed calibration must contain measured time_offset_lidar_to_imu')
        offset=0.0
    if not math.isfinite(float(offset)):raise ValueError('non-finite time offset')
    rclpy.init();node=Node('fastlio_prepare',namespace=cfg.namespace,
        parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=opts.use_sim_time)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    laser=cfg.frame('laser');imu=opts.imu_frame or cfg.get('FASTLIO_IMU_FRAME','zed2i_imu_link');base=cfg.frame('base_link')
    deadline=time.monotonic()+30
    while True:
        rclpy.spin_once(node,timeout_sec=.1)
        try:
            base_imu=matrix(buffer.lookup_transform(base,imu,Time()).transform)
            imu_laser=matrix(buffer.lookup_transform(imu,laser,Time()).transform)
            break
        except TransformException:
            if time.monotonic()>deadline:raise ValueError(f'missing base_link / laser / selected IMU static TF ({imu}); start description and the selected sensor (or replay /tf_static)')
    t=cal.get('extrinsic_T');r=cal.get('extrinsic_R')
    if t is None or r is None:
        if confirmed:raise ValueError('confirmed calibration requires extrinsic_T and extrinsic_R (lidar -> IMU)')
        t=imu_laser[:3,3].tolist();r=imu_laser[:3,:3].reshape(-1).tolist()
    r=np.asarray(r,float).reshape(3,3);t=np.asarray(t,float)
    if t.shape!=(3,) or not np.isfinite(t).all() or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(r),1,atol=1e-5):
        raise ValueError('invalid lidar-to-IMU rigid transform')
    # Calibrated lidar-to-IMU plus vehicle's lidar mount determines base-to-IMU;
    # it must not silently revert to a different CAD camera mount.
    imu_laser[:3,:3]=r;imu_laser[:3,3]=t
    base_laser=matrix(buffer.lookup_transform(base,laser,Time()).transform)
    base_imu=base_laser@np.linalg.inv(imu_laser)
    defaults=yaml.safe_load(Path(opts.defaults).read_text())
    defaults.update({'use_sim_time':opts.use_sim_time,'common.lid_topic':cfg.topic('fastlio/points'),
        'common.imu_topic':cfg.topic('fastlio/imu'),'common.time_sync_en':False,
        'common.time_offset_lidar_to_imu':float(offset),'mapping.extrinsic_est_en':False,
        'mapping.extrinsic_T':t.tolist(),'mapping.extrinsic_R':r.reshape(-1).tolist(),
        'frames.odom':cfg.frame('odom_lidar'),'frames.imu':imu,'publish.odom_tf':False})
    common={'calibration_confirmed':confirmed,'points_topic':cfg.get('POINTS_TOPIC',cfg.topic('points')),'use_sim_time':opts.use_sim_time,'laser_frame':laser,'imu_frame':imu,'base_frame':base,
        'wheel_odom_frame':cfg.frame('odom'),'lidar_odom_frame':cfg.frame('odom_lidar'),
        'time_offset_lidar_to_imu':float(offset),'max_imu_gap':gap,
        'base_to_imu_T':base_imu[:3,3].tolist(),'base_to_imu_R':base_imu[:3,:3].reshape(-1).tolist()}
    data={cfg.topic('fastlio_mapping'):{'ros__parameters':defaults},
        cfg.topic('fastlio_input'):{'ros__parameters':dict(copy.deepcopy(common),role='input')},
        cfg.topic('fastlio_output'):{'ros__parameters':dict(copy.deepcopy(common),role='output')}}
    Path(opts.output).write_text('# External calibration: '+('operator-confirmed' if confirmed else 'UNVERIFIED / pending')+'\n'+yaml.safe_dump(data))
    node.get_logger().warning('Calibration status: '+('operator-confirmed' if confirmed else 'UNVERIFIED CAD/factory TF; time offset assumed zero if unset')+'; effective configuration: '+opts.output)
    node.destroy_node();rclpy.shutdown()
