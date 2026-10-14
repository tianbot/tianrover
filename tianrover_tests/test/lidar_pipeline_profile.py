#!/usr/bin/env python3
"""Read-only warmed-up profiling; raw cloud subscriptions avoid Python conversion cost."""
import argparse
import json
import struct
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def cdr_stamp(data):
    if len(data)<12 or data[:2] not in (b'\x00\x01',b'\x00\x00'):
        raise ValueError('unsupported CDR header')
    sec,nano=struct.unpack_from('<iI' if data[1]==1 else '>iI',data,4)
    return sec+nano*1e-9


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace',default='mars');parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--output',required=True);args=parser.parse_args()
    prefix='/'+args.namespace.strip('/')+'/' if args.namespace.strip('/') else '/'
    rclpy.init();node=Node('lidar_pipeline_profile');rows={};audits=[]
    def receive(key):
        def callback(raw):
            stamp=cdr_stamp(raw)
            rows[key].append((stamp,time.monotonic(),node.get_clock().now().nanoseconds*1e-9-stamp,len(raw)))
        return callback
    for topic,kind in [('points',PointCloud2),('points_checked',PointCloud2),('kiss/odometry',Odometry),('odom_lidar',Odometry)]:
        rows[topic]=[]
        node.create_subscription(kind,prefix+topic,receive(topic),
                                 QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE),raw=True)
    node.create_subscription(String,prefix+'lidar/timing',lambda m:audits.append(json.loads(m.data)),10)
    try:
        # Warm discovery and frontend startup before measuring a fixed interval.
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and not all(rows.values()):rclpy.spin_once(node,timeout_sec=.05)
        warmed={k:bool(v) for k,v in rows.items()}
        for values in rows.values():values.clear()
        audits.clear();started=time.monotonic();deadline=started+args.seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        elapsed=time.monotonic()-started;result={'elapsed_s':elapsed,'warmed':warmed,'topics':{}}
        for key,values in rows.items():
            if len(values)<2:
                result['topics'][key]={'count':len(values)};continue
            array=np.asarray(values);gap=np.diff(array[:,0])
            result['topics'][key]=dict(count=len(values),arrival_hz=len(values)/elapsed,
                 stamp_hz=(len(values)-1)/(array[-1,0]-array[0,0]),max_stamp_gap_s=float(gap.max()),
                 age_p50_s=float(np.percentile(array[:,2],50)),age_p95_s=float(np.percentile(array[:,2],95)))
        valid=[a for a in audits if a.get('valid')]
        result['audit']={'valid':len(valid),'errors':[a for a in audits if not a.get('valid')][:5]}
        for key in ('check_seconds','publish_seconds','input_age_seconds'):
            values=[a[key] for a in valid if key in a]
            if values:result['audit'][key]={'p50':float(np.percentile(values,50)),'p95':float(np.percentile(values,95))}
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
