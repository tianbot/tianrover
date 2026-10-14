from types import SimpleNamespace as NS
import numpy as np
import pytest
from tianrover_localization.lidar_cloud import checked_cloud
from tianrover_localization.lidar_velocity import derive_velocity


def cloud():
    dtype = np.dtype(dict(names=['x','y','z','timestamp','ring'], formats=['<f4','<f4','<f4','<f8','<u2'],
                         offsets=[0,4,8,16,24], itemsize=32))
    points = np.zeros((2, 100), dtype=dtype)
    points['x'] = 1
    points['timestamp'] = np.linspace(1000,1000.1,200).reshape(2,100)
    points['ring'] = np.arange(200).reshape(2,100)%16
    padded = b''.join(row.tobytes()+b'padding!' for row in points)
    return NS(header=NS(stamp=NS(sec=1000,nanosec=100000000)),
              fields=[NS(name=n, offset=dtype.fields[n][1], datatype=8 if n=='timestamp' else (4 if n=='ring' else 7), count=1)
                      for n in dtype.names], height=2,width=100,point_step=32,row_step=3208,
              data=padded,is_bigendian=False,is_dense=False), points


def test_organized_padded_layout_preserves_ring_and_point_time():
    msg, expected = cloud()
    output, audit = checked_cloud(msg)
    assert output.height == 1 and output.width == 200
    assert bytes(output.data) == expected.tobytes()
    assert audit['span'] == pytest.approx(.1)
    assert msg.height == 2


@pytest.mark.parametrize('kind',['missing_time','zero_span','missing_stamp','short_buffer','integer_coordinate','integer_time'])
def test_bad_point_cloud_rejected(kind):
    msg, points = cloud()
    if kind == 'missing_time':
        msg.fields = [f for f in msg.fields if f.name!='timestamp']
    elif kind == 'zero_span':
        points['timestamp'] = 1000
        msg.row_step=3200
        msg.data=points.tobytes()
    elif kind == 'missing_stamp':
        msg.header.stamp=NS(sec=0,nanosec=0)
    elif kind == 'short_buffer':
        msg.data=b''
    elif kind == 'integer_coordinate':
        msg.fields[0].datatype=6
    else:
        msg.fields[3].datatype=6
    with pytest.raises(ValueError):
        checked_cloud(msg)


def pose(t, x=0, yaw=0, frame='mars/odom'):
    return NS(header=NS(stamp=NS(sec=int(t),nanosec=round((t-int(t))*1e9)),frame_id=frame),
              child_frame_id='mars/base_footprint',pose=NS(pose=NS(position=NS(x=x,y=0,z=0),
              orientation=NS(x=0,y=0,z=np.sin(yaw/2),w=np.cos(yaw/2)))))


def test_lidar_pose_velocity():
    linear, angular=derive_velocity(pose(10),pose(10.1,.02,.03))
    assert np.linalg.norm(linear) == pytest.approx(.2)
    assert angular[2] == pytest.approx(.3)


@pytest.mark.parametrize('next_pose',[pose(10),pose(11),pose(10.1,2),pose(10.1, yaw=1),pose(10.1,frame='other')])
def test_discontinuous_pose_rejected(next_pose):
    with pytest.raises(ValueError):
        derive_velocity(pose(10),next_pose)
