import math
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tianrover_localization.ekf_inputs import prepare_odom, prepare_imu


def test_unknown_velocity_covariance_becomes_explicit_without_mutating_raw():
    raw = Odometry()
    raw.twist.twist.linear.x = 0.3
    raw.twist.twist.linear.y = -0.1
    prepared = prepare_odom(raw, 0.0025)
    assert prepared.twist.covariance[0] == prepared.twist.covariance[7] == 0.0025
    assert prepared.twist.twist.linear.y == -0.1
    assert list(raw.twist.covariance) == [0] * 36


def test_supplied_velocity_uncertainty_is_preserved():
    raw = Odometry()
    raw.twist.covariance[0], raw.twist.covariance[7] = 0.1, 0.2
    raw.twist.covariance[1] = raw.twist.covariance[6] = 0.01
    assert list(prepare_odom(raw, 0.0025).twist.covariance) == list(raw.twist.covariance)


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_nonfinite_velocity_is_rejected(bad):
    raw = Odometry()
    raw.twist.twist.linear._x = bad
    with pytest.raises(ValueError):
        prepare_odom(raw, 0.0025)


def test_normalize_attitude_and_fill_unknown_covariance():
    raw = Imu()
    raw.orientation.z = raw.orientation.w = 0.7
    prepared = prepare_imu(raw, 0.0025)
    assert math.isclose(prepared.orientation.z, math.sqrt(0.5))
    assert prepared.orientation_covariance[8] == 0.0025
    assert raw.orientation.z == 0.7


def test_supplied_yaw_uncertainty_is_preserved():
    raw = Imu()
    raw.orientation.w = 1.0
    raw.orientation_covariance[8] = 0.02
    assert list(prepare_imu(raw, 0.0025).orientation_covariance) == list(raw.orientation_covariance)


@pytest.mark.parametrize('unavailable', [False, True])
def test_absent_orientation_is_not_invented(unavailable):
    raw = Imu()
    raw.orientation.w = 0.0
    if unavailable:
        raw.orientation.w = 1.0
        raw.orientation_covariance[0] = -1
    with pytest.raises(ValueError):
        prepare_imu(raw, 0.0025)


def test_relative_topic_cannot_feed_filtered_output_back_into_filter(monkeypatch):
    from launch import LaunchContext
    from tianrover_localization.launching import setup
    monkeypatch.setenv('TIANBOT_NAME', 'ekf_loop_check')
    context = LaunchContext()
    context.launch_configurations.update(use_sim_time='false', publish_tf='false',
                                        odom_topic='odom', filtered_topic='/ekf_loop_check/odom')
    with pytest.raises(ValueError, match='must differ'):
        setup(context, 'ekf')
