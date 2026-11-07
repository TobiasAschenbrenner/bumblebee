from pathlib import Path
from unittest import TestCase
from unittest.mock import Mock, patch

from bumblebee.errors import SafetyError
from bumblebee.radio import connected_crazyflie


class RadioConnectionTests(TestCase):
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
