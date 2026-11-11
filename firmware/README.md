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
Firmware v9 rejects the unsafe v6 direct-thrust lift request. Its flip profile
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
an onboard disarm for unstable boost, rotation, and recovery failures. Its
first flight completed the rotation but reached the floor before recovery; the
normal controller briefly restored enough thrust after impact for a small hop.
Version 9 issues an immediate motor stop on unstable failures and prevents the
normal controller from running while disarmed. This firmware uses explicit
arming instead of the stock Crazyflie 2.1 auto-arm behavior; all Bumblebee
flight routines already arm before takeoff and disarm during cleanup. A
stationary startup check must
verify that all four motor commands reached zero before the Mac will permit an
Acro flight. It also starts active braking near 100° instead of 210°, based on
the measured v8 pitch rate and deceleration.
Version 10 adds an explicit two-metre recovery-height experiment. It keeps the
normal Safe-mode ceiling at 1.5 m and retains the version 9 motor cutoff. The
high test uses the last flown v8 rotation profile, changing only the staging
height so the result can show whether additional recovery room is sufficient.
That test and a later 1.5 m run showed that extra height and earlier PID braking
were not enough: the controller remained in its 500 ms braking window until
impact and never reached the recovery phase.
Version 11 replaces the stock rate-PID rotation window with Bumblebee's
out-of-tree force/torque controller. Normal flight still delegates to the stock
PID. The flip uses bounded direct pitch torque for spin-up and braking, retains
low collective thrust while coasting, and hands control back to the PID only
after measured rate and attitude are both suitable for recovery. A 12 cm
airborne cutoff is enforced inside the stabilizer loop and latches the direct
controller off until the next power cycle.
Version 12 widens only the permitted pre-rotation staging altitude to
0.75–1.65 m. The first v11 test reached 1.55 m for a 1.50 m target and was
correctly rejected by the previous strict boundary; it then completed a stable,
controlled landing without beginning the rotation.
Version 13 applies the 12 cm direct-control cutoff only while the estimated
attitude is within 45 degrees of upright. The first v12 rotation stopped near
inversion, where Flow/Kalman altitude is not a trustworthy ground-proximity
signal. Version 13 also distinguishes an upright altitude cutoff and an expired
Acro permit from a supervisor stop in its onboard result telemetry.
The first v13 test confirmed that both altitude and estimated attitude can jump
to false ground/upright values during the high-rate rotation. Version 14 removes
altitude shutdown from the bounded direct-control window entirely and retains
the 12 cm check for upright position recovery. It also starts direct-torque
braking at 250 degrees instead of 290 degrees after v13 measured a 1450 deg/s
peak pitch rate.
Version 14 reached braking at 251 degrees and reduced pitch rate from 1400 to
742 deg/s, but still crossed the guarded 410-degree limit. Version 15 uses that
measured deceleration to begin braking at 140 degrees and raises only the brake
torque authority from 85% to the guarded 95% maximum. The spin phase and all
other flight and watchdog limits remain unchanged.
Version 15 completed a rotation and made a soft, immediate-stop contact with the
padded floor, but entered braking at a 1269 deg/s peak and still measured
910 deg/s when braking timed out. Version 16 reduces spin torque authority from
85% to 55% and hands off from spin-up at 600 instead of 700 deg/s. Early 140
degree braking and 95% brake authority remain unchanged.
Version 16 completed a full forward rotation physically but spent too long in
the reduced spin phase and contacted the floor while braking was still active;
the impact then reversed the logged angular direction. Version 17 restores the
85% spin pulse but ends it at 450 deg/s and 10 degrees, before motor lag can
build the previous peak rate. It also latches direct control off after a 4 g
impact once at least 60 degrees of rotation has begun, independent of inverted
altitude estimates.
Version 17 visually completed a clean flip, reduced pitch rate to -77 deg/s,
and stopped immediately on padded-floor contact. Its signed angle was distorted
by the final braking reversal, so version 18 also accepts at least 300 degrees
of raw travel when the airframe is upright and slow. It then keeps the stock
attitude and position PID active while enforcing a 200 ms collective floor at
120% of learned hover thrust. The 4 g impact cutoff remains active throughout
that recovery pulse.
Version 18 visually finished upright but never activated recovery because the
high-rate attitude estimate missed the strict 25-degree gate. Version 19 keeps
that strict signed-angle path and adds a guarded raw-travel path requiring at
least 320 degrees, at most 180 deg/s pitch rate, and attitude within 60 degrees
of upright before the same stabilized recovery boost can start.
Version 19 again finished visually upright but the estimator never entered the
60-degree gate before impact. Version 20 accepts the raw-travel path only inside
a 320–380 degree window and below 180 deg/s, then applies an estimator-independent
120% collective pulse with zero pitch torque for 75–150 ms. It hands off to the
stock PID and existing 200 ms thrust floor only if the attitude estimate settles
within 60 degrees during that pulse. Impact cutoff remains active throughout.

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
