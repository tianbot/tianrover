"""Measured XYZIRT/IMU input gate and independent base-link odometry adapter."""
import copy
import json
import math
import time
from array import array as byte_array
from collections import deque
import numpy as np


class FrontendNotReady(ValueError):
    """Normal DDS discovery wait; data forwarding remains blocked."""


def matrix(t):
    q=t.rotation if hasattr(t,'rotation') else t.orientation
    v=np.asarray([q.x,q.y,q.z,q.w],float)
    if not np.isfinite(v).all() or np.linalg.norm(v)<1e-8:
        raise ValueError('invalid transform quaternion')
    x,y,z,w=v/np.linalg.norm(v)
    a=np.eye(4);a[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    v=t.translation if hasattr(t,'translation') else t.position
    a[:3,3]=[v.x,v.y,v.z]
    if not np.isfinite(a).all():raise ValueError('non-finite pose')
    return a


def quaternion(r):
    # Stable eigenvector method, including rotations close to pi.
    k=np.array([[r[0,0]-r[1,1]-r[2,2],r[0,1]+r[1,0],r[0,2]+r[2,0],r[2,1]-r[1,2]],
        [r[0,1]+r[1,0],r[1,1]-r[0,0]-r[2,2],r[1,2]+r[2,1],r[0,2]-r[2,0]],
        [r[0,2]+r[2,0],r[1,2]+r[2,1],r[2,2]-r[0,0]-r[1,1],r[1,0]-r[0,1]],
        [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],r.trace()]])/3
    vals,vecs=np.linalg.eigh(k);q=vecs[:,np.argmax(vals)]
    return q if q[3]>=0 else -q


def fill_pose(p,a):
    v=p.translation if hasattr(p,'translation') else p.position
    q=p.rotation if hasattr(p,'rotation') else p.orientation
    v.x,v.y,v.z=map(float,a[:3,3]);q.x,q.y,q.z,q.w=map(float,quaternion(a[:3,:3]))


def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9


def h16_cloud(msg):
    from sensor_msgs.msg import PointCloud2,PointField
    from tianrover_localization.lidar_cloud import checked_cloud
    checked,audit=checked_cloud(msg,True)
    fs={f.name:f for f in checked.fields}
    if 'ring' not in fs or fs['ring'].datatype!=PointField.UINT16 or fs['ring'].count!=1:
        raise ValueError('H16 requires measured UINT16 ring')
    if 'timestamp' not in fs or fs['timestamp'].datatype!=PointField.FLOAT64:
        raise ValueError('H16 requires FLOAT64 absolute timestamp in seconds; no inferred units')
    if 'intensity' not in fs or fs['intensity'].datatype!=PointField.FLOAT32:
        raise ValueError('H16 requires FLOAT32 intensity')
    names=['x','y','z','intensity','ring','timestamp']
    if any(fs[n].offset+{7:4,4:2,8:8}[fs[n].datatype]>checked.point_step for n in names):
        raise ValueError('H16 field extends beyond point_step')
    endian='>' if checked.is_bigendian else '<'
    dt=np.dtype({'names':names,'formats':[endian+f for f in ['f4','f4','f4','f4','u2','f8']],
        'offsets':[fs[n].offset for n in names],'itemsize':checked.point_step})
    a=np.frombuffer(bytes(checked.data),dtype=dt)
    if np.any(a['ring']>15):raise ValueError('H16 ring outside 0..15')
    if not np.isfinite(a['intensity']).all():raise ValueError('non-finite H16 intensity')
    times=a['timestamp'];begin=float(times.min());end=float(times.max())
    if begin<1e8 or abs(stamp(msg)-end)>.02:
        raise ValueError('H16 absolute seconds/header-at-scan-end contract violated')
    if not .05 < end-begin < .15:
        raise ValueError('unexpected H16 10 Hz measured scan duration')
    order=np.argsort(times,kind='stable');a=a[order]
    out_dt=np.dtype({'names':['x','y','z','intensity','time','ring'],
        'formats':['<f4','<f4','<f4','<f4','<f4','<u2'],
        'offsets':[0,4,8,12,16,20],'itemsize':24})
    out=np.zeros(len(a),dtype=out_dt)
    for key in ['x','y','z','intensity','ring']:out[key]=a[key]
    out['time']=a['timestamp']-begin
    result=PointCloud2();result.header=copy.deepcopy(msg.header)
    ns=round(begin*1e9);result.header.stamp.sec,result.header.stamp.nanosec=divmod(ns,10**9)
    result.height=1;result.width=len(out);result.point_step=24;result.row_step=24*len(out)
    result.is_dense=True;result.is_bigendian=False
    result.fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in
        [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('time',16,7),('ring',20,4)]]
    result.data=byte_array('B',out.tobytes());audit.update(scan_begin=begin,scan_end=end)
    return result,audit


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2,Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String,Bool
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException

    class IO(Node):
        def __init__(self):
            super().__init__('fastlio_io')
            defaults={'calibration_confirmed':False,'role':'input','laser_frame':'mars/laser','imu_frame':'zed2i_imu_link',
                'base_frame':'mars/base_link','points_topic':'/mars/points','wheel_odom_frame':'mars/odom','lidar_odom_frame':'mars/odom_lidar',
                'time_offset_lidar_to_imu':0.0,'max_imu_gap':.1,'max_pending_age':3.0,
                'max_scan_age':3.0,'publish_alignment_tf':True,
                'base_to_imu_T':[0.,0.,0.],'base_to_imu_R':[1.,0.,0.,0.,1.,0.,0.,0.,1.]}
            for k,v in defaults.items():self.declare_parameter(k,v)
            self.p=lambda k:self.get_parameter(k).value
            self.audit=self.create_publisher(String,'lidar/fastlio_status',10)
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.started_at=time.monotonic()
            self.role=self.p('role');self.last_imu=None;self.last_cloud=None
            self.input_ready=False;self.times=deque(maxlen=2000);self.pending=deque(maxlen=30)
            self.previous=None;self.poses=deque(maxlen=20);self.authority=None
            self.odom_wait=deque(maxlen=40);self.cloud_wait=deque(maxlen=40);self.graph_checked_at=-math.inf
            self.create_subscription(Bool,'lidar/tf_authority_ok',self.authority_received,10)
            if self.role=='input':
                self.cloud_pub=self.create_publisher(PointCloud2,'fastlio/points',10)
                self.imu_pub=self.create_publisher(Imu,'fastlio/imu',400)
                self.create_subscription(PointCloud2,'points',self.cloud,qos_profile_sensor_data)
                self.create_subscription(Imu,'imu_input',self.imu,
                    QoSProfile(depth=400,reliability=ReliabilityPolicy.BEST_EFFORT))
                self.create_timer(.01,self.flush)
            elif self.role=='output':
                self.create_timer(.01,self.flush_odom)
                self.base_imu=np.eye(4);self.base_imu[:3,:3]=np.asarray(self.p('base_to_imu_R')).reshape(3,3)
                self.base_imu[:3,3]=self.p('base_to_imu_T')
                self.odom_pub=self.create_publisher(Odometry,'odom_lidar',10)
                self.cloud_pub=self.create_publisher(PointCloud2,'lidar/cloud_deskewed',10)
                self.tf=TransformBroadcaster(self)
                self.wheel_tf_pub=self.create_publisher(TFMessage,'fastlio/wheel_tf',10)
                self.create_subscription(TFMessage,'/tf',self.wheel_tf,qos_profile_sensor_data)
                self.create_subscription(Odometry,'fastlio/odometry_raw',self.odom,40)
                self.create_subscription(PointCloud2,'fastlio/cloud_body',self.body_cloud,40)
            else:raise ValueError('unknown fastlio_io role')
        def fail(self,error):
            if isinstance(error,FrontendNotReady):
                self.get_logger().info(str(error),throttle_duration_sec=5.)
            else:
                self.get_logger().error(str(error),throttle_duration_sec=5.)
            self.audit.publish(String(data=json.dumps({'valid':False,'error':str(error),'role':self.role})))
        def authority_received(self,msg):
            import time
            self.authority=(time.monotonic(),msg.data)
        def frontend(self):
            import time
            if self.authority is None and time.monotonic()-self.started_at<5.:
                raise FrontendNotReady('Initializing: waiting for TF authority guard discovery')
            if self.authority is None or time.monotonic()-self.authority[0]>.5 or not self.authority[1]:
                raise ValueError('laser/wheel TF authority check missing or conflicting')
            now=time.monotonic()
            if now-self.graph_checked_at<.1:return
            ns=self.get_namespace()
            raw=self.get_publishers_info_by_topic('fastlio/odometry_raw')
            kisses=self.get_publishers_info_by_topic('kiss/odometry')
            if not raw and not kisses and now-self.started_at<5.:
                raise FrontendNotReady('Initializing: waiting for FAST-LIO publisher discovery')
            if kisses or len(raw)!=1 or raw[0].node_name!='fastlio_mapping' or raw[0].node_namespace!=ns:
                raise ValueError('exactly one frontend required: stop KISS/duplicate FAST-LIO before starting')
            if len(self.get_publishers_info_by_topic(self.p('points_topic')))!=1:
                raise ValueError('exactly one H16 points publisher required')
            self.graph_checked_at=now
        def imu(self,msg):
            try:
                self.frontend()
                if msg.header.frame_id!=self.p('imu_frame'):raise ValueError('unexpected selected IMU frame')
                t=stamp(msg)-self.p('time_offset_lidar_to_imu')
                values=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,
                        msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
                if not math.isfinite(t) or t<=0 or not np.isfinite(values).all():raise ValueError('invalid IMU stamp/values')
                if msg.linear_acceleration_covariance[0]<0 or msg.angular_velocity_covariance[0]<0:
                    raise ValueError('IMU acceleration/angular velocity unavailable')
                if self.last_imu is not None and t<=self.last_imu:
                    self.times.clear();self.pending.clear();raise ValueError('IMU clock moved backwards/repeated: restart frontend')
                self.last_imu=t;self.times.append(t);self.imu_pub.publish(msg)
            except ValueError as e:self.fail(e)
        def cloud(self,msg):
            try:
                self.frontend()
                if msg.header.frame_id!=self.p('laser_frame'):raise ValueError('unexpected H16 frame')
                out,a=h16_cloud(msg)
                if self.last_cloud is not None and a['scan_begin']<=self.last_cloud:
                    raise ValueError('lidar clock moved backwards: restart frontend')
                self.last_cloud=a['scan_begin'];self.pending.append((out,a,self.get_clock().now().nanoseconds*1e-9))
            except (ValueError,KeyError,TypeError,BufferError) as e:self.fail(e)
        def flush(self):
            if not self.pending:return
            out,a,queued=self.pending[0];now=self.get_clock().now().nanoseconds*1e-9
            try:
                self.frontend()
                if now-a['scan_end']>self.p('max_scan_age') or now-queued>self.p('max_pending_age'):
                    self.pending.popleft();raise ValueError('IMU coverage timeout/stale scan; check sensor load and clocks')
                if not self.times or self.times[-1]<a['scan_end']:return
                self.pending.popleft();times=np.asarray(self.times)
                left=np.searchsorted(times,a['scan_begin'],side='right')-1
                right=np.searchsorted(times,a['scan_end'],side='left')
                if left<0 and not self.input_ready:
                    raise FrontendNotReady('Initializing: skipping partial scan before first IMU sample')
                if left<0 or right>=len(times):raise ValueError('IMU does not bracket scan start/end')
                gaps=np.diff(times[left:right+1])
                if len(gaps)==0 or gaps.max()>self.p('max_imu_gap'):
                    raise ValueError('IMU measurement gap exceeds configured limit; dropping scan')
                self.input_ready=True
                self.cloud_pub.publish(out);a.update(valid=True,imu_max_gap=float(gaps.max()),calibration_confirmed=self.p('calibration_confirmed'))
                self.audit.publish(String(data=json.dumps(a)))
            except ValueError as e:self.fail(e)
        def wheel_tf(self,msg):
            selected=[t for t in msg.transforms if t.header.frame_id==self.p('wheel_odom_frame')
                      and t.child_frame_id==self.p('base_frame').removesuffix('base_link')+'base_footprint']
            if selected:self.wheel_tf_pub.publish(TFMessage(transforms=selected))
        def odom(self,msg):
            self.odom_wait.append(msg)
        def flush_odom(self):
            if not self.odom_wait:return
            msg=self.odom_wait[0];t=stamp(msg)
            # Both outputs describe exactly one scan end. Wait for the other
            # reliable topic and wheel TF; never publish unmatched pairs.
            cloud=next((c for c in self.cloud_wait if abs(stamp(c)-t)<1e-6),None)
            covered=not self.p('publish_alignment_tf') or self.buffer.can_transform(
                self.p('wheel_odom_frame'),self.p('base_frame'),Time.from_msg(msg.header.stamp))
            if cloud is None or not covered:
                if self.get_clock().now().nanoseconds*1e-9-t>self.p('max_pending_age'):
                    self.odom_wait.popleft()
                    self.fail('scan pair timeout: '+('deskewed cloud missing' if cloud is None else 'wheel TF unavailable'))
                return
            self.odom_wait.popleft()
            self.cloud_wait=deque((c for c in self.cloud_wait if stamp(c)>t+1e-6),maxlen=40)
            if self.publish_odom(msg):self.publish_body(cloud)
        def publish_odom(self,msg):
            try:
                if msg.header.frame_id!=self.p('lidar_odom_frame') or msg.child_frame_id!=self.p('imu_frame'):
                    raise ValueError('unexpected FAST-LIO odometry frames')
                a=matrix(msg.pose.pose)@np.linalg.inv(self.base_imu)
                out=copy.deepcopy(msg);out.child_frame_id=self.p('base_frame');fill_pose(out.pose.pose,a)
                # Pose covariance translation includes the IMU-to-base lever arm.
                r=matrix(msg.pose.pose)[:3,:3];lever=np.linalg.inv(self.base_imu)[:3,3]
                skew=np.array([[0,-lever[2],lever[1]],[lever[2],0,-lever[0]],[-lever[1],lever[0],0]])
                j=np.eye(6);j[:3,3:]=-r@skew;j[3:,3:]=r
                out.pose.covariance=list((j@np.asarray(msg.pose.covariance).reshape(6,6)@j.T).reshape(-1))
                t=stamp(msg)
                now=self.get_clock().now().nanoseconds*1e-9
                if not math.isfinite(t) or t>now+0.1 or now-t>self.p('max_scan_age'):
                    raise ValueError('stale/future FAST-LIO output stamp; check sensor load and clocks')
                if self.previous:
                    pt,pa=self.previous;dt=t-pt
                    if not math.isfinite(dt) or dt<=0:
                        raise ValueError('FAST-LIO clock moved backwards/repeated: restart front/backend')
                    # A dropped frame or delayed subscriber is not a clock reset.
                    # Use the measured interval for diagnostic twist, retaining
                    # the original pose, timestamp, scan pairing and TF checks.
                    if dt>1.:
                        self.get_logger().warning(
                            f'FAST-LIO output gap {dt:.3f}s; twist averaged over measured interval',
                            throttle_duration_sec=5.)
                    v=a[:3,:3].T@((a[:3,3]-pa[:3,3])/dt)
                    out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,v)
                    dq=quaternion(pa[:3,:3].T@a[:3,:3]);angle=2*math.atan2(np.linalg.norm(dq[:3]),dq[3])
                    angular=dq[:3]/max(np.linalg.norm(dq[:3]),1e-12)*angle/dt
                    out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,angular)
                    # Twist is diagnostic; no wheel/EKF feedback.
                self.frontend()
                if self.p('publish_alignment_tf'):
                    wheel=self.buffer.lookup_transform(self.p('wheel_odom_frame'),self.p('base_frame'),Time.from_msg(msg.header.stamp))
                    trans=TransformStamped();trans.header=copy.deepcopy(out.header)
                    trans.child_frame_id=self.p('wheel_odom_frame');fill_pose(trans.transform,a@np.linalg.inv(matrix(wheel.transform)))
                    self.tf.sendTransform(trans)
                self.odom_pub.publish(out)
                self.previous=(t,a);self.poses.append(t)
                return True
            except (ValueError,TransformException) as e:
                self.fail(e)
                return False
        def body_cloud(self,msg):
            self.cloud_wait.append(msg)
        def publish_body(self,msg):
            try:
                if msg.header.frame_id!=self.p('imu_frame'):raise ValueError('unexpected deskewed cloud frame')
                fields={f.name:f for f in msg.fields};endian='>' if msg.is_bigendian else '<'
                if any(n not in fields or fields[n].datatype!=7 for n in ['x','y','z']):
                    raise ValueError('unexpected FAST-LIO point coordinates')
                out=copy.deepcopy(msg);data=bytearray(bytes(msg.data))
                dt=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,
                    'offsets':[fields[n].offset for n in ['x','y','z']],'itemsize':msg.point_step})
                points=np.ndarray((msg.height,msg.width),dtype=dt,buffer=data,strides=(msg.row_step,msg.point_step))
                xyz=np.stack([points[n] for n in ['x','y','z']],axis=-1)
                xyz=xyz@self.base_imu[:3,:3].T+self.base_imu[:3,3]
                for i,n in enumerate(['x','y','z']):points[n]=xyz[...,i]
                out.data=byte_array('B',data);out.header.frame_id=self.p('base_frame');self.cloud_pub.publish(out)
            except (ValueError,TypeError) as e:self.fail(e)
    rclpy.init(args=args);node=IO()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
