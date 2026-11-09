# AI-deck firmware

Bumblebee normally uses official Bitcraze firmware. The optional UDP camera
experiment is built from pinned revisions of the community-maintained LARICS
forks:

- ESP32: `larics/aideck-esp-firmware-udp@2b7152366048bbfa98eea343f68264c52ba0bd0d`
- GAP8: `larics/aideck-gap8-examples@124729b2a9f98809c5caed841a29701aab3221fb`

The manual `Build Bumblebee firmware` GitHub Actions workflow builds both deck
applications plus matching Crazyflie firmware images and publishes a
checksummed `bumblebee-firmware` artifact:

- `cf2-2025.02-aideck-recovery.bin` temporarily leaves Wi-Fi setup to an
  already-installed GAP8 image. It is only needed when recovering from a
  conflicting deck configuration.
- `cf2-2025.02-aideck-ap.bin` is the final Crazyflie image and starts the open
  `Bumblebee` access point.
- `cf2-2025.02-bumblebee-acro-aideck-ap.bin` adds Bumblebee's onboard flight
  experiment app and a time-limited supervisor permit. It starts the same open
  `Bumblebee` access point and is required for lift measurements and Acro
  routines.
- `aideck_esp_udp.bin` forwards camera packets only after a viewer registers
  itself with the `FER` UDP handshake. This Bumblebee patch prevents an absent
  viewer from flooding the Crazyflie link with failed sends.
- `aideck_gap8_udp_streamer.bin` captures and encodes the camera frames without
  attempting to create a second Wi-Fi access point.

No private Wi-Fi credentials are included.

## Acro firmware safety model

The experiment app is idle until it receives a one-shot Bumblebee request.
Firmware v8 rejects the unsafe v6 direct-thrust lift request. Its flip profile
uses the stock position controller for the initial lift, followed by separate
rate-controlled spin-up, coast, and active counter-rotation braking phases.
Stock position control resumes for recovery only after the Crazyflie is upright
and its pitch rate has slowed. Any unstable onboard phase requests an immediate
disarm before releasing control.

Flip profiles use a permit that is available only while flying, is capped at
1.5 seconds, and expires without any Mac-side cleanup.
Emergency stop, the emergency watchdog, the commander watchdog, arming, crash,
and deck checks stay active. The normal `cf2-2025.02-aideck-ap.bin` remains
available as the non-Acro fallback.

The first profile is intentionally marked experimental. Its phase structure is
informed by [Crazyflie backflip research](https://arxiv.org/abs/2209.14652), but
the implementation is original and uses the stock rate controller. It must be
tuned through padded tests for this specific AI-deck and Flow-deck airframe
before it can be considered reliable.

Acro app version 1 must not be used: its absolute gyro integration could count
reversal as forward rotation and apply recovery thrust before the Crazyflie was
upright. Version 2 measures signed net rotation, confirms the end attitude, and
holds zero thrust if either check fails. Version 3 lowers the experimental
under-load start threshold to 3.50 V for padded testing. Version 4 adds more
vertical boost, hands control back sooner, records altitude recovery, and no
longer reports success unless stable flight is recovered. Version 4 must not be
used because it failed to start reliably on the test airframe. Version 5 keeps
the revised recovery profile, removes the extra startup telemetry that was added
in version 4, and validates the final altitude and vertical speed through the
Mac's standard flight telemetry. Version 5 completed a full rotation on the
AI-deck and Flow-deck test airframe, but reached recovery at 0.33 m while still
descending at 3.14 m/s and crashed into the padded surface. Version 6 disables
the flip trigger and adds an upright lift/recovery measurement. That test gained
only 0.01 m and 0.12 m/s from its 200 ms pulse, sagged to 3.605 V, then lost
control sideways. The Mac now rejects both experimental routines on v6. They
must not be repeated with older host code. Version 7 enables the new phased
flip only. Its bounded boost, thrust allocation, braking point, and recovery
duration parameters are volatile and are written by the Mac before every run,
so later profiles can be tuned without reflashing the Crazyflie. It also records
the minimum altitude and voltage, lift gain, braking angle and end rate, and
recovery handoff altitude for the next iteration.
Version 7's earliest and strongest guarded passive brake still left 434°/s of
pitch rate at 410° of rotation. It also demonstrated that a zero-collective
rate setpoint can leave individual motors correcting attitude after impact.
Version 8 replaces that brake with a bounded opposite-rate command and requests
an onboard disarm for unstable boost, rotation, and recovery failures.

## Restore official firmware

The community firmware can be replaced over Crazyradio without JTAG while the
existing deck bootloaders remain intact. Restore these official releases in
this order:

1. ESP32 `aideck_esp.bin` from
   [aideck-esp-firmware 2025.02](https://github.com/bitcraze/aideck-esp-firmware/releases/tag/2025.02)
2. GAP8 `aideck_gap8_wifi_img_streamer_with_ap.bin` from
   [aideck-gap8-examples 2026.10.1](https://github.com/bitcraze/aideck-gap8-examples/releases/tag/2026.10.1)
3. The preferred official Crazyflie 2.x firmware from
   [crazyflie-firmware releases](https://github.com/bitcraze/crazyflie-firmware/releases)

```bash
cfloader flash aideck_esp.bin deck-bcAI:esp-fw -w radio://0/80/2M
cfloader flash aideck_gap8_wifi_img_streamer_with_ap.bin deck-bcAI:gap8-fw -w radio://0/80/2M
cfloader flash cf2.bin stm32-fw -w radio://0/80/2M
```

Keep the Crazyflie powered throughout each command and wait for `100%` before
starting the next one.
