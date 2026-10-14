"""Launch-test the EKF fixture; it never opens a hardware device."""
import sys
import unittest
from pathlib import Path
import launch
import launch_testing
import launch_testing.actions
import launch_testing.util
import launch_testing.asserts


def generate_test_description():
    fixture = launch.actions.ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name('ekf_runtime_check.py'))], output='screen')
    return launch.LaunchDescription([launch_testing.util.KeepAliveProc(), fixture,
                                     launch_testing.actions.ReadyToTest()]), {'fixture': fixture}


class EkfContract(unittest.TestCase):
    def test_fixture_completes(self, proc_info, fixture):
        proc_info.assertWaitForShutdown(process=fixture, timeout=240)


@launch_testing.post_shutdown_test()
class EkfExit(unittest.TestCase):
    def test_fixture_passed(self, proc_info, fixture):
        launch_testing.asserts.assertExitCodes(proc_info, process=fixture)
