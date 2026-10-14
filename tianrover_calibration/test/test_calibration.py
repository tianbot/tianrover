import copy
import numpy as np
import pytest
from tianrover_calibration.calibration import CODES, check_profile, solve, validate


def fixture(seed=1):
    rng = np.random.default_rng(seed)
    t = np.arange(0, 60, .1)
    v = np.column_stack((.17*np.sin(.25*t), .015*np.sin(.17*t), .12*np.sin(.41*t)))
    xy = [(x, y) for y in (.32, -.32) for x in (.56, 0, -.56)]
    radii = np.linspace(.12, .13, 6)
    offsets = np.linspace(-.07, .06, 6)
    profile = dict(reference='kiss_icp_only', extrinsic_verified=True, flat_ground_verified=True,
                   feedback_measured=True, clock_alignment_verified=True, feedback_topic='/measured',
                   feedback_provenance='hardware_encoder', max_timestamp_jitter_s=.005,
                   feedback_time_offset_s=0., wheels={})
    omega, steer = np.zeros((len(t), 6)), np.zeros((len(t), 6))
    for i, code in enumerate(CODES):
        x, y = xy[i]
        profile['wheels'][code] = dict(wheel_joint='wheel_'+code, steering_joint='steer_'+code,
                                     xy=[x, y], nominal_radius_m=.125)
        wx, wy = v[:, 0]-v[:, 2]*y, v[:, 1]+v[:, 2]*x
        sign = np.where(wx >= 0, 1., -1.)
        omega[:, i] = sign*np.hypot(wx, wy)/radii[i] + rng.normal(0, .003, len(t))
        steer[:, i] = np.arctan2(sign*wy, sign*wx)-offsets[i]+rng.normal(0, .001, len(t))
    return dict(time=t, velocity=v, omega=omega, steer=steer), profile, radii, offsets


def test_recovers_six_independent_parameters_and_validates():
    data, profile, radii, offsets = fixture()
    params, report = solve(data, profile)
    assert report['status'] == 'candidate'
    for i, code in enumerate(CODES):
        assert params['wheels'][code]['effective_radius_m'] == pytest.approx(radii[i], abs=.0005)
        assert params['wheels'][code]['steering_offset_rad'] == pytest.approx(offsets[i], abs=.002)
    independent, _, _, _ = fixture(7)
    assert validate(independent, profile, params)['status'] == 'validated'
    independent['omega'][:, 0] *= 1.5
    with pytest.raises(ValueError, match='validation failed'):
        validate(independent, profile, params)


@pytest.mark.parametrize('field', ['extrinsic_verified', 'flat_ground_verified', 'feedback_measured', 'clock_alignment_verified'])
def test_unverified_hardware_rejected(field):
    _, profile, _, _ = fixture()
    profile[field] = False
    with pytest.raises(ValueError, match=field):
        check_profile(profile)


def test_default_display_and_duplicate_joint_rejected():
    _, p, _, _ = fixture()
    p['feedback_provenance'] = 'joint_state_publisher'
    with pytest.raises(ValueError, match='actual encoder'):
        check_profile(p)
    p['feedback_provenance'] = 'hardware_encoder'
    p['wheels']['LM']['wheel_joint'] = p['wheels']['LF']['wheel_joint']
    with pytest.raises(ValueError, match='exactly one'):
        check_profile(p)


@pytest.mark.parametrize('kind', ['static', 'constant_curvature', 'missing_reverse', 'gap', 'nan'])
def test_bad_recording_rejected(kind):
    data, profile, _, _ = fixture()
    if kind == 'static':
        data['velocity'][:] = 0
    elif kind == 'constant_curvature':
        data['velocity'][:, 2] = data['velocity'][:, 0]*.4
    elif kind == 'missing_reverse':
        data['omega'] = np.abs(data['omega'])
    elif kind == 'gap':
        data['time'][300:] += 1
    else:
        data['steer'][0, 0] = np.nan
    with pytest.raises(ValueError):
        solve(data, profile)


def test_outliers_small_fraction_is_robust():
    data, profile, radii, _ = fixture()
    data['steer'][::40, :] += .3
    params, _ = solve(data, profile)
    assert params['wheels']['LF']['effective_radius_m'] == pytest.approx(radii[0], abs=.001)


def test_low_speed_wrong_radius_cannot_pass_absolute_error_only():
    data, profile, _, _ = fixture()
    params, _ = solve(data, profile)
    params['wheels']['LF']['effective_radius_m'] *= 1.12
    independent, _, _, _ = fixture(9)
    # A 12% error at low speed is smaller than the absolute 0.04 m/s bound.
    with pytest.raises(ValueError, match='independent validation failed'):
        validate(independent, profile, params)
