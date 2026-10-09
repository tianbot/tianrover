"""Run the kiss synthetic fixture through ROS 2 launch_testing."""
import sys
import unittest
from pathlib import Path
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util


def generate_test_description():
    fixture = launch.actions.ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name('kiss_runtime_check.py')), '--bringup'],
        output='screen')
    return launch.LaunchDescription([launch_testing.util.KeepAliveProc(), fixture,
                                     launch_testing.actions.ReadyToTest()]), {'fixture': fixture}


class RuntimeContract(unittest.TestCase):
    def test_fixture_completes(self, proc_info, fixture):
        proc_info.assertWaitForShutdown(process=fixture, timeout=120)


@launch_testing.post_shutdown_test()
class RuntimeExit(unittest.TestCase):
    def test_fixture_passed(self, proc_info, fixture):
        launch_testing.asserts.assertExitCodes(proc_info, process=fixture)
