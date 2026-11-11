from pathlib import Path
from unittest import TestCase
from unittest.mock import Mock, patch

from bumblebee.errors import SafetyError
from bumblebee.radio import connected_crazyflie


class RadioConnectionTests(TestCase):
    @patch("bumblebee.radio.Crazyflie")
    @patch("bumblebee.radio.SyncCrazyflie")
    def test_connection_disables_unused_latency_ping_worker(
        self,
        sync_crazyflie: Mock,
        crazyflie_factory: Mock,
    ) -> None:
        crazyflie = crazyflie_factory.return_value
        original_start = crazyflie.link_statistics.start
        original_stop = crazyflie.link_statistics.stop

        with (
            patch("bumblebee.radio.cache_directory", return_value=Path("/tmp")),
            connected_crazyflie("radio://0/80/2M"),
        ):
            crazyflie.link_statistics.start()
            crazyflie.link_statistics.stop()

        original_start.assert_not_called()
        original_stop.assert_not_called()
        sync_crazyflie.assert_called_once_with(
            "radio://0/80/2M",
            cf=crazyflie,
        )

    @patch("bumblebee.radio.Crazyflie")
    @patch("bumblebee.radio.SyncCrazyflie")
    def test_connection_failure_becomes_readable_safety_error(
        self,
        sync_crazyflie: Mock,
        _crazyflie: Mock,
    ) -> None:
        connection = sync_crazyflie.return_value
        connection.open_link.side_effect = Exception(
            "Cannot find a Crazyradio Dongle\ntrace"
        )

        with (
            patch("bumblebee.radio.cache_directory", return_value=Path("/tmp")),
            self.assertRaisesRegex(
                SafetyError,
                "Could not connect.*Cannot find a Crazyradio Dongle",
            ),
            connected_crazyflie("radio://0/80/2M"),
        ):
            pass
