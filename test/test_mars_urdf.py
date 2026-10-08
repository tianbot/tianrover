"""Kinematic checks for the six-wheel/six-steering Mars CAD model."""
from math import cos, sin
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


MODEL = Path(__file__).resolve().parents[1] / 'urdf' / 'mars.urdf'


def rotation(axis, angle):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + sin(angle) * skew + (1 - cos(angle)) * (skew @ skew)


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        roll, pitch, yaw = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        transform[:3, :3] = (rotation([0, 0, 1], yaw)
                             @ rotation([0, 1, 0], pitch)
                             @ rotation([1, 0, 0], roll))
    return transform


def forward(model, positions=None):
    positions = positions or {}
    transforms = {'base_footprint': np.eye(4)}
    pending = model.findall('joint')
    while pending:
        remaining = []
        for joint in pending:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                remaining.append(joint)
                continue
            transform = origin(joint.find('origin'))
            if joint.get('type') != 'fixed':
                motion = np.eye(4)
                motion[:3, :3] = rotation(
                    np.fromstring(joint.find('axis').get('xyz'), sep=' '),
                    positions.get(joint.get('name'), 0))
                transform = transform @ motion
            transforms[joint.find('child').get('link')] = transforms[parent] @ transform
        assert len(remaining) < len(pending), 'Disconnected or cyclic joint tree'
        pending = remaining
    return transforms


def test_six_steering_pivots_and_wheel_axes():
    model = ET.parse(MODEL).getroot()
    neutral = forward(model)
    joints = {j.get('name'): j for j in model.findall('joint')}
    for code in ('LF', 'LM', 'LB', 'RF', 'RM', 'RB'):
        steering = joints['Bogie' + code + '_']
        wheel_name = 'WheelRF' if code == 'RF' else 'Wheel' + code + '_'
        wheel = joints[wheel_name]
        bogie_frame, wheel_frame = 'Bogie' + code + '_Link', 'Wheel' + code + '_Link'
        # Physical axes in the neutral robot frame, rather than raw CAD-local axes.
        assert np.allclose(neutral[bogie_frame][:3, :3] @ np.fromstring(
            steering.find('axis').get('xyz'), sep=' '), [0, 0, 1], atol=1e-8)
        assert np.allclose(neutral[wheel_frame][:3, :3] @ np.fromstring(
            wheel.find('axis').get('xyz'), sep=' '), [0, 1, 0], atol=1e-8)
        assert wheel.get('type') == 'continuous'
        # Steering must turn at each wheel, not orbit around the front wheel.
        for angle in (-0.6, 0.6):
            moved = forward(model, {steering.get('name'): angle})[wheel_frame]
            assert np.linalg.norm(moved[:3, 3] - neutral[wheel_frame][:3, 3]) < 1e-4
            assert np.allclose(moved[:3, :3], rotation([0, 0, 1], angle), atol=1e-8)
        center = neutral[wheel_frame][:3, 3]
        assert center[1] > 0 if code.startswith('L') else center[1] < 0
        assert abs(center[2] - 0.125) < 1e-4
    for side in ('L', 'R'):
        xs = [neutral['Wheel' + side + index + '_Link'][0, 3] for index in ('F', 'M', 'B')]
        assert xs[0] > xs[1] > xs[2]
        assert np.allclose(np.diff(xs), [-0.5676, -0.5676], atol=1e-4)


def test_suspension_parentage_and_motion():
    model = ET.parse(MODEL).getroot()
    joints = {j.get('name'): j for j in model.findall('joint')}
    neutral = forward(model)
    for side, label in (('L', 'Left'), ('R', 'Right')):
        assert joints['Bogie' + side + 'B_'].find('parent').get('link') == label + 'MainRocket_Link'
        for index in ('F', 'M'):
            assert joints['Bogie' + side + index + '_'].find('parent').get('link') == label + 'ViceRocket_Link'
        rocked = forward(model, {label + 'MainRocket_': 0.2})
        for index in ('F', 'M', 'B'):
            frame = 'Wheel' + side + index + '_Link'
            assert np.linalg.norm(rocked[frame][:3, 3] - neutral[frame][:3, 3]) > 0.02
        opposite = 'Wheel' + ('R' if side == 'L' else 'L') + 'B_Link'
        assert np.allclose(rocked[opposite], neutral[opposite])
    for joint in joints.values():
        limit = joint.find('limit')
        if limit is not None:
            assert float(limit.get('effort')) > 0


def test_virtual_footprint_and_sensor_frames():
    model = ET.parse(MODEL).getroot()
    footprint = model.find("link[@name='base_footprint']")
    assert len(footprint) == 0  # CAD top panel belongs to the body, not the ground frame.
    frames = forward(model)
    assert np.allclose(frames['base_link'][:3, 3], [0.0043743, 0, 0.354])
    assert np.allclose(frames['laser'][:3, 3], [0.004107431474011556, 0, 0.67731477961933])
    assert np.allclose(frames['laser'][:3, :3], np.eye(3))
    assert np.allclose(frames['body_top_link'], np.eye(4))
