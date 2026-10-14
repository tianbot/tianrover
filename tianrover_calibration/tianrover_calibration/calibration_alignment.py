"""Align measured feedback with wheel-free planar laser poses, without extrapolation."""
import numpy as np
from tianrover_calibration.calibration import check_profile


def align(laser_time, laser_pose, feedback_time, wheel_values, steering_angles, profile):
    """Pose columns x,y,yaw; wheel values rad/s or measured shaft radians."""
    check_profile(profile)
    lt, lp, ft, wheels, steer = map(lambda a: np.asarray(a, dtype=float),
                                  (laser_time, laser_pose, feedback_time, wheel_values, steering_angles))
    if (lt.ndim != 1 or ft.ndim != 1 or len(lt) < 2 or len(ft) < 2
            or lp.shape != (len(lt), 3) or wheels.shape != (len(ft), 6)
            or steer.shape != (len(ft), 6)):
        raise ValueError('invalid laser/feedback dimensions')
    if not all(np.isfinite(a).all() for a in (lt, lp, ft, wheels, steer)):
        raise ValueError('non-finite synchronized measurement')
    ft = ft + profile['feedback_time_offset_s']
    if np.any(np.diff(lt) <= 0) or np.any(np.diff(ft) <= 0):
        raise ValueError('measurement timestamps out of order')
    if np.max(np.diff(ft)) > .05:
        raise ValueError('feedback gap exceeds 50 ms')
    dt = np.diff(lt)
    if np.any(dt < .02) or np.any(dt > .3):
        raise ValueError('laser pose interval outside 20..300 ms')
    t = (lt[:-1]+lt[1:])/2
    # Position derivatives need feedback across each complete pose interval.
    if lt[0] < ft[0] or lt[-1] > ft[-1]:
        raise ValueError('feedback does not cover full laser trajectory; extrapolation forbidden')
    yaw = np.unwrap(lp[:, 2])
    dyaw = np.diff(yaw)
    if np.max(np.abs(dyaw)) > .15:
        raise ValueError('laser rotation discontinuity or excessive speed')
    middle = (yaw[:-1]+yaw[1:])/2
    dp = np.diff(lp[:, :2], axis=0)
    c, s = np.cos(middle), np.sin(middle)
    half = dyaw/2
    # Inverse SE(2) translation Jacobian: midpoint projection times theta/(2*sin(theta/2)).
    correction = 1/np.sinc(half/np.pi)
    velocity = np.column_stack(((c*dp[:, 0]+s*dp[:, 1])*correction/dt,
                                (-s*dp[:, 0]+c*dp[:, 1])*correction/dt, dyaw/dt))
    angle = np.unwrap(steer, axis=0)
    aligned_steer = np.column_stack([np.interp(t, ft, angle[:, i]) for i in range(6)])
    mode = profile.get('wheel_measurement')
    if mode == 'velocity_rad_s':
        omega = np.column_stack([np.interp(t, ft, wheels[:, i]) for i in range(6)])
    elif mode in ('continuous_position_rad', 'wrapped_position_rad'):
        position = np.unwrap(wheels, axis=0) if mode == 'wrapped_position_rad' else wheels
        at_pose = np.column_stack([np.interp(lt, ft, position[:, i]) for i in range(6)])
        omega = np.diff(at_pose, axis=0)/dt[:, None]
    else:
        raise ValueError('specify actual wheel measurement and wrap convention')
    return dict(time=t, velocity=velocity, omega=omega, steer=aligned_steer)
