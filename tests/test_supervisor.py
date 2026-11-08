import warnings
from unittest import TestCase
from unittest.mock import Mock

from bumblebee.supervisor import (
    request_arming,
    send_emergency_stop_watchdog,
)


LEGACY_WARNING = (
    "The supervisor subsystem requires CRTP protocol version 12 or later. "
    "Connected Crazyflie reports version 9. Update your Crazyflie firmware. "
    "Using legacy fallback."
)


class SupervisorCompatibilityTests(TestCase):
    def test_arming_uses_fallback_without_noisy_known_warning(self) -> None:
        connection = Mock()
        connection.cf.supervisor.send_arming_request.side_effect = (
            lambda _do_arm: warnings.warn(LEGACY_WARNING, UserWarning)
        )

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            request_arming(connection, False)

        self.assertEqual([], caught)
        connection.cf.supervisor.send_arming_request.assert_called_once_with(False)

    def test_watchdog_uses_same_supported_fallback(self) -> None:
        connection = Mock()
        connection.cf.supervisor.send_emergency_stop_watchdog.side_effect = (
            lambda: warnings.warn(LEGACY_WARNING, UserWarning)
        )

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            send_emergency_stop_watchdog(connection)

        self.assertEqual([], caught)
        connection.cf.supervisor.send_emergency_stop_watchdog.assert_called_once()

    def test_unrelated_warning_is_not_hidden(self) -> None:
        connection = Mock()
        connection.cf.supervisor.send_arming_request.side_effect = (
            lambda _do_arm: warnings.warn("different warning", UserWarning)
        )

        with self.assertWarnsRegex(UserWarning, "different warning"):
            request_arming(connection, True)
