import math
from unittest.mock import patch
import pytest
import rclpy
from tianrover_teleop.keyboard import Keyboard, MOVES, make_twist


@pytest.mark.parametrize('key', list(MOVES))
def test_planar_command(key):
    msg = make_twist(MOVES[key], 0.2, 0.4)
    assert math.hypot(msg.linear.x, msg.linear.y) <= 0.200001
    assert msg.linear.z == msg.angular.x == msg.angular.y == 0.0
    assert msg.angular.z == MOVES[key][2] * 0.4


@pytest.mark.parametrize('key, linear, angular', [
    ('q', 0.2, 0.4), ('e', 0.2, -0.4),
    ('z', -0.2, 0.4), ('c', -0.2, -0.4)])
def test_arc_commands(key, linear, angular):
    msg = make_twist(MOVES[key], 0.2, 0.4)
    assert msg.linear.x == linear
    assert msg.linear.y == 0.0
    assert msg.angular.z == angular


def test_keys_stop_timeout_and_limits():
    rclpy.init()
    node = Keyboard()
    try:
        with patch.object(node.publisher, 'publish') as pub:
            node.key('q')
            node.publish()
            assert pub.call_args[0][0].linear.x == 0.2
            assert pub.call_args[0][0].angular.z == 0.4
            assert node.speed == 0.2 and node.turn == 0.4
            node.key('j')
            node.publish()
            assert pub.call_args[0][0].linear.y > 0
            node.last_key -= 1
            node.publish()
            assert pub.call_args[0][0].linear.y == 0
            node.key('i')
            node.key(' ')
            node.publish()
            assert pub.call_args[0][0].linear.x == 0
            for _ in range(50):
                node.key('t')
            assert node.speed == node.limits['speed_limit']
            assert node.turn == node.limits['turn_limit']
    finally:
        node.destroy_node()
        rclpy.shutdown()
