import time
from unittest import TestCase
from unittest.mock import Mock

from bumblebee.errors import SafetyError
from bumblebee.watchdog import EmergencyStopWatchdog


class EmergencyStopWatchdogTests(TestCase):
    def test_watchdog_sends_immediately_and_repeats(self) -> None:
        connection = Mock()

        with EmergencyStopWatchdog(connection, interval=0.01):
            time.sleep(0.035)

        self.assertGreaterEqual(
            connection.cf.supervisor.send_emergency_stop_watchdog.call_count,
            3,
        )

    def test_send_failure_is_exposed_as_safety_error(self) -> None:
        connection = Mock()
        connection.cf.supervisor.send_emergency_stop_watchdog.side_effect = [
            None,
            RuntimeError("radio failed"),
        ]

        with EmergencyStopWatchdog(connection, interval=0.01) as watchdog:
            time.sleep(0.025)
            with self.assertRaises(SafetyError):
                watchdog.assert_healthy()
