from unittest import TestCase

from bumblebee.models import (
    FlightRequirements,
    PreflightSnapshot,
    Telemetry,
)
from bumblebee.preflight import preflight_issues


def snapshot(
    *,
    voltage: float = 3.90,
    power_state: int = 0,
    flow_deck: bool = True,
    supervisor_info: int | None = 1 << 3,
    acro_firmware_version: int | None = None,
) -> PreflightSnapshot:
    return PreflightSnapshot(
        flow_deck_attached=flow_deck,
        ai_deck_attached=True,
        firmware_low_voltage=3.50,
        telemetry=Telemetry(
            voltage=voltage,
            power_state=power_state,
            supervisor_info=supervisor_info,
            downward_range_mm=42,
        ),
        acro_firmware_version=acro_firmware_version,
    )


class PreflightTests(TestCase):
    def test_safe_snapshot_has_no_issues(self) -> None:
        issues = preflight_issues(snapshot(), FlightRequirements(flow_deck=True))
        self.assertEqual([], issues)

    def test_missing_required_deck_is_reported(self) -> None:
        issues = preflight_issues(
            snapshot(flow_deck=False),
            FlightRequirements(flow_deck=True),
        )
        self.assertTrue(any("Flow deck" in issue for issue in issues))

    def test_usb_power_and_low_voltage_are_reported(self) -> None:
        issues = preflight_issues(
            snapshot(voltage=3.55, power_state=1),
            FlightRequirements(flow_deck=True),
        )
        self.assertTrue(any("disconnect" in issue for issue in issues))
        self.assertTrue(any("battery" in issue for issue in issues))

    def test_old_firmware_is_reported(self) -> None:
        issues = preflight_issues(
            snapshot(supervisor_info=None),
            FlightRequirements(flow_deck=True),
        )
        self.assertTrue(any("supervisor.info" in issue for issue in issues))

    def test_acro_flight_requires_bumblebee_firmware(self) -> None:
        issues = preflight_issues(
            snapshot(),
            FlightRequirements(
                flow_deck=True,
                minimum_acro_firmware_version=2,
            ),
        )
        self.assertTrue(any("acro firmware" in issue for issue in issues))

    def test_acro_firmware_version_satisfies_requirement(self) -> None:
        issues = preflight_issues(
            snapshot(acro_firmware_version=2),
            FlightRequirements(
                flow_deck=True,
                minimum_acro_firmware_version=2,
            ),
        )
        self.assertEqual([], issues)

    def test_unsafe_acro_firmware_version_is_rejected(self) -> None:
        issues = preflight_issues(
            snapshot(acro_firmware_version=1),
            FlightRequirements(
                flow_deck=True,
                minimum_acro_firmware_version=2,
            ),
        )

        self.assertTrue(any("unsafe" in issue for issue in issues))
