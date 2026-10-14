import numpy as np
import pytest
from test_calibration import fixture
from tianrover_calibration.calibration_alignment import align


def test_exact_planar_arc_and_position_feedback():
    _, p, _, _ = fixture()
    p['wheel_measurement']='continuous_position_rad'
    lt=np.arange(10, 14.01, .1)
    ft=np.arange(9.99,14.03,.01)
    yaw=.1*(lt-10)
    lp=np.column_stack((2*np.sin(yaw), 2*(1-np.cos(yaw)),yaw))
    wheels=(ft-10)[:,None]*np.ones((1,6))*1.6
    steering=np.zeros_like(wheels)
    data=align(lt,lp,ft,wheels,steering,p)
    assert np.allclose(data['velocity'],[.2,0,.1],atol=1e-9)
    assert np.allclose(data['omega'],1.6,atol=1e-9)


@pytest.mark.parametrize('kind',['gap','extrapolate','clock_unknown','mode_unknown'])
def test_alignment_refuses_unknown_or_missing_measurements(kind):
    _, p, _, _ = fixture()
    p['wheel_measurement']='velocity_rad_s'
    lt=np.arange(10,11,.1)
    ft=np.arange(9.9,11.1,.01)
    lp=np.zeros((len(lt),3))
    if kind=='gap':
        ft[50:]+=1
    elif kind=='extrapolate':
        lt+=10
    elif kind=='clock_unknown':
        p['clock_alignment_verified']=False
    else:
        del p['wheel_measurement']
    with pytest.raises(ValueError):
        align(lt,lp,ft,np.zeros((len(ft),6)),np.zeros((len(ft),6)),p)
