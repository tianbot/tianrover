"""Six independent rolling constraints against wheel-free laser motion."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np


CODES = ('LF', 'LM', 'LB', 'RF', 'RM', 'RB')


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def check_profile(profile):
    if profile.get('reference') != 'kiss_icp_only':
        raise ValueError('calibration reference must be independent KISS-ICP, without wheel/IMU fusion')
    for key in ('extrinsic_verified', 'flat_ground_verified', 'feedback_measured', 'clock_alignment_verified'):
        if profile.get(key) is not True:
            raise ValueError('unverified requirement: '+key)
    if not profile.get('feedback_topic') or profile.get('feedback_provenance') in (None, '', 'joint_state_publisher'):
        raise ValueError('actual encoder/steering interface is required')
    jitter = profile.get('max_timestamp_jitter_s')
    if not isinstance(jitter, (int, float)) or not 0 <= jitter <= 0.02:
        raise ValueError('feedback timestamp uncertainty exceeds 20 ms or is unknown')
    if not isinstance(profile.get('feedback_time_offset_s'), (int, float)) or not math.isfinite(profile['feedback_time_offset_s']):
        raise ValueError('measured feedback clock/latency offset is required')
    if set(profile.get('wheels', {})) != set(CODES):
        raise ValueError('exactly six wheel configurations required')
    joints = []
    for code in CODES:
        w = profile['wheels'][code]
        if not w.get('wheel_joint') or not w.get('steering_joint'):
            raise ValueError('actual canonical joint mapping missing: '+code)
        if len(w.get('xy', [])) != 2 or not np.isfinite(w['xy']).all():
            raise ValueError('invalid wheel pivot geometry: '+code)
        radius = w.get('nominal_radius_m')
        if not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius <= 0:
            raise ValueError('invalid nominal radius: '+code)
        joints.extend((w['wheel_joint'], w['steering_joint']))
    if len(set(joints)) != 12:
        raise ValueError('each measured joint must map to exactly one active joint')


def check_data(data, profile):
    check_profile(profile)
    t, velocity, omega, steer = (np.asarray(data[k], dtype=float) for k in ('time', 'velocity', 'omega', 'steer'))
    if t.ndim != 1:
        raise ValueError('time must be a one-dimensional measurement sequence')
    n = len(t)
    if n < 100 or velocity.shape != (n, 3) or omega.shape != (n, 6) or steer.shape != (n, 6):
        raise ValueError('insufficient samples or invalid data dimensions')
    if not all(np.isfinite(a).all() for a in (t, velocity, omega, steer)):
        raise ValueError('non-finite calibration measurement')
    dt = np.diff(t)
    if (dt <= 0).any() or dt.max() > 0.3 or t[-1]-t[0] < 15:
        raise ValueError('timestamp gaps, ordering or recording duration insufficient')
    travel = float(np.sum(np.linalg.norm(velocity[:-1, :2], axis=1)*dt))
    rotation = float(np.sum(np.abs(velocity[:-1, 2])*dt))
    excitation = velocity[:, [0, 2]]
    scales = np.linalg.norm(excitation, axis=0)
    if travel < 2 or rotation < 0.5 or scales.min() < 0.2:
        raise ValueError('need translation and curved/rotational excitation')
    singular = np.linalg.svd(excitation/scales, compute_uv=False)
    if singular[-1]/singular[0] < 0.1:
        raise ValueError('unidentifiable motion: only one constant curvature')
    if np.max(np.linalg.norm(velocity[:, :2], axis=1)) > 0.6 or np.max(np.abs(velocity[:, 2])) > 0.6:
        raise ValueError('calibration speed exceeds commissioning limits')
    return t, velocity, omega, steer, {'samples': n, 'travel_m': travel, 'rotation_rad': rotation,
                                       'excitation_condition': float(singular[0]/singular[-1])}


def wheel_target(velocity, steer, xy):
    vx = velocity[:, 0]-velocity[:, 2]*xy[1]
    vy = velocity[:, 1]+velocity[:, 2]*xy[0]
    c, s = np.cos(steer), np.sin(steer)
    return np.column_stack((c*vx+s*vy, -s*vx+c*vy))


def solve(data, profile):
    _, velocity, omega, steer, quality = check_data(data, profile)
    result, report = {}, {}
    for index, code in enumerate(CODES):
        target = wheel_target(velocity, steer[:, index], profile['wheels'][code]['xy'])
        spin = omega[:, index]
        active = np.abs(spin) > 0.15
        if active.sum() < 100 or not (np.any(spin > 0.3) and np.any(spin < -0.3)):
            raise ValueError('wheel needs measured forward/reverse rotation: '+code)
        w, z = spin[active], target[active]
        weights = np.ones(len(w))
        for _ in range(12):
            denominator = np.sum(weights*w*w)
            ab = np.sum(weights[:, None]*w[:, None]*z, axis=0)/denominator
            residual = np.linalg.norm(z-w[:, None]*ab, axis=1)
            weights = np.minimum(1, 0.025/np.maximum(residual, 1e-9))
        radius, offset = float(np.linalg.norm(ab)), float(math.atan2(ab[1], ab[0]))
        rms = float(np.sqrt(np.mean(residual**2)))
        speed_scale = float(np.sqrt(np.mean(np.sum(z*z, axis=1))))
        relative_rms = rms/max(speed_scale, 1e-9)
        inliers = residual < 0.05
        variance = float(np.sum(weights*residual**2)/max(1, 2*inliers.sum()-2)/denominator)
        radius_ci = 1.96*math.sqrt(variance)
        offset_ci = radius_ci/max(radius, 1e-9)
        nominal = float(profile['wheels'][code]['nominal_radius_m'])
        if not 0.7*nominal < radius < 1.3*nominal or abs(offset) > 0.35:
            raise ValueError('physically implausible radius/steering offset: '+code)
        if rms > 0.04 or relative_rms > 0.08 or inliers.mean() < 0.8 or radius_ci/radius > 0.03 or offset_ci > 0.03:
            raise ValueError('slip, poor fit or insufficient parameter confidence: '+code)
        result[code] = {'effective_radius_m': radius, 'effective_diameter_m': 2*radius,
                        'steering_offset_rad': offset}
        report[code] = {'rms_m_s': rms, 'inlier_fraction': float(inliers.mean()),
                        'relative_rms': relative_rms,
                        'radius_95_ci_m': radius_ci, 'offset_95_ci_rad': offset_ci}
    return {'schema': 1, 'wheels': result}, {'status': 'candidate', 'quality': quality, 'wheels': report}


def validate(data, profile, parameters):
    _, velocity, omega, steer, quality = check_data(data, profile)
    report = {}
    for index, code in enumerate(CODES):
        p = parameters['wheels'][code]
        spin = omega[:, index]
        active = np.abs(spin) > 0.15
        if active.sum() < 100 or not (np.any(spin > 0.3) and np.any(spin < -0.3)):
            raise ValueError('validation requires forward/reverse measurements: '+code)
        nominal = profile['wheels'][code]['nominal_radius_m']
        if (not 0.7*nominal < p['effective_radius_m'] < 1.3*nominal
                or not math.isfinite(p['steering_offset_rad']) or abs(p['steering_offset_rad']) > 0.35):
            raise ValueError('invalid candidate parameters: '+code)
        a, b = p['effective_radius_m']*np.cos(p['steering_offset_rad']), p['effective_radius_m']*np.sin(p['steering_offset_rad'])
        observed = wheel_target(velocity, steer[:, index], profile['wheels'][code]['xy'])
        predicted = omega[:, index, None]*np.array([a, b])
        error = np.linalg.norm(predicted-observed, axis=1)[active]
        rms, percentile = float(np.sqrt(np.mean(error**2))), float(np.percentile(error, 95))
        speed_scale = float(np.sqrt(np.mean(np.sum(observed[active]**2, axis=1))))
        relative_rms = rms/max(speed_scale, 1e-9)
        report[code] = {'rms_m_s': rms, 'p95_m_s': percentile, 'relative_rms': relative_rms}
        if rms > 0.04 or percentile > 0.06 or relative_rms > 0.08:
            raise ValueError('independent validation failed: '+code)
    return {'status': 'validated', 'quality': quality, 'wheels': report}


def load_dataset(directory):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text())
    if digest(directory/'samples.npz') != manifest['samples_sha256']:
        raise ValueError('dataset checksum mismatch')
    with np.load(directory/'samples.npz', allow_pickle=False) as source:
        data = {key: source[key].copy() for key in source.files}
    return data, manifest
