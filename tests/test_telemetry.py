from unittest import TestCase
from unittest.mock import Mock

from bumblebee.errors import SafetyError
from bumblebee.telemetry import TimedTelemetryLogger


class TimedTelemetryLoggerTests(TestCase):
    @staticmethod
    def _logger(timeout: float = 0.01) -> tuple[TimedTelemetryLogger, Mock, Mock]:
        connection = Mock()
        config = Mock()
        return TimedTelemetryLogger(connection, config, timeout), connection, config

    def test_yields_received_telemetry(self) -> None:
        logger, _connection, config = self._logger()

        with logger:
            callback = config.data_received_cb.add_callback.call_args.args[0]
            callback(42, {"stateEstimate.z": 0.3}, "flight")

            self.assertEqual(
                (42, {"stateEstimate.z": 0.3}, "flight"),
                next(logger),
            )

    def test_missing_telemetry_becomes_safety_error(self) -> None:
        logger, _connection, _config = self._logger(timeout=0.001)

        with logger, self.assertRaisesRegex(SafetyError, "No flight telemetry"):
            next(logger)

    def test_radio_disconnect_becomes_safety_error(self) -> None:
        logger, connection, _config = self._logger()

        with logger:
            callback = connection.cf.disconnected.add_callback.call_args.args[0]
            callback("radio://0/80/2M")

            with self.assertRaisesRegex(SafetyError, "radio link disconnected"):
                next(logger)
