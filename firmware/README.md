# AI-deck firmware

Bumblebee normally uses official Bitcraze firmware. The optional UDP camera
experiment is built from pinned revisions of the community-maintained LARICS
forks:

- ESP32: `larics/aideck-esp-firmware-udp@2b7152366048bbfa98eea343f68264c52ba0bd0d`
- GAP8: `larics/aideck-gap8-examples@124729b2a9f98809c5caed841a29701aab3221fb`

The manual `Build AI-deck UDP firmware` GitHub Actions workflow builds both
deck applications plus matching Crazyflie firmware images and publishes a
checksummed artifact:

- `cf2-2025.02-aideck-recovery.bin` temporarily leaves Wi-Fi setup to an
  already-installed GAP8 image. It is only needed when recovering from a
  conflicting deck configuration.
- `cf2-2025.02-aideck-ap.bin` is the final Crazyflie image and starts the open
  `Bumblebee` access point.
- `aideck_esp_udp.bin` forwards camera packets only after a viewer registers
  itself with the `FER` UDP handshake. This Bumblebee patch prevents an absent
  viewer from flooding the Crazyflie link with failed sends.
- `aideck_gap8_udp_streamer.bin` captures and encodes the camera frames without
  attempting to create a second Wi-Fi access point.

No private Wi-Fi credentials are included.

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
