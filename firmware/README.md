# AI-deck firmware

Bumblebee normally uses official Bitcraze firmware. The optional UDP camera
experiment is built from pinned revisions of the community-maintained LARICS
forks:

- ESP32: `larics/aideck-esp-firmware-udp@2b7152366048bbfa98eea343f68264c52ba0bd0d`
- GAP8: `larics/aideck-gap8-examples@124729b2a9f98809c5caed841a29701aab3221fb`

The manual `Build AI-deck UDP firmware` GitHub Actions workflow builds both
applications and publishes a checksummed artifact. No Wi-Fi credentials are
included; the GAP8 application creates the `WiFi streaming example` access
point.

## Restore official firmware

The community firmware can be replaced over Crazyradio without JTAG while the
existing deck bootloaders remain intact. Restore these official releases in
this order:

1. ESP32 `aideck_esp.bin` from
   [aideck-esp-firmware 2025.02](https://github.com/bitcraze/aideck-esp-firmware/releases/tag/2025.02)
2. GAP8 `aideck_gap8_wifi_img_streamer_with_ap.bin` from
   [aideck-gap8-examples 2026.10.1](https://github.com/bitcraze/aideck-gap8-examples/releases/tag/2026.10.1)

```bash
cfloader flash aideck_esp.bin deck-bcAI:esp-fw -w radio://0/80/2M
cfloader flash aideck_gap8_wifi_img_streamer_with_ap.bin deck-bcAI:gap8-fw -w radio://0/80/2M
```

Keep the Crazyflie powered throughout each command and wait for `100%` before
starting the next one.
