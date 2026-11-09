# Bumblebee 🐝

Bumblebee is a small terminal companion for my Bitcraze Crazyflie 2.1+.
It provides friendly diagnostics, live battery telemetry, AI-deck camera streaming,
and guarded flight routines.

---

## ✨ Features

- Pre-flight diagnostics
- Battery monitoring
- Live AI-deck camera viewing and recording
- A flight selector for current and future routines
- A guarded takeoff, hover, and landing routine
- Automatic landing when a safety check fails
- A shared 1.5 m altitude ceiling and 1.5 m flight radius
- Separate Safe and explicitly unlocked Acro flight modes
- A stock-controller motor and power measurement for Acro development
- An experimental onboard flip routine with unsafe profiles version-gated

---

## 🔩 Hardware

- Crazyflie 2.1+
- Flow deck V2
- AI-deck 1.1
- Crazyradio PA

The configured radio link is `radio://0/80/2M`.

---

## 🚀 Getting Started

### Prerequisites

- Python 3.10 or newer
- [uv](https://docs.astral.sh/uv/)
- Current Crazyflie firmware

Install the project dependencies:

```bash
uv sync
```

Before flying, close `cfclient`, disconnect the Crazyflie's USB cable, and place
the drone on a level, textured surface with plenty of clear space around it.

---

## ▶️ Commands

```bash
./diagnose   # read-only preflight report
./battery    # live battery dashboard
./camera     # view and record the AI-deck camera
./fly        # choose a flight routine
./hover      # open the hover routine directly
```

For the camera, power on the Crazyflie and connect the Mac to the AI-deck's
`Bumblebee` Wi-Fi network. The network has no internet access, so if macOS
switches back to your usual Wi-Fi, select `Bumblebee` again. Then run:

```bash
./camera
```

The camera window opens after the first frame arrives. Press **R** to start or
stop recording and **Q** or **Escape** to close it. Recordings are saved under
`recordings/` and are ignored by Git. The camera command never connects to the
Crazyradio or sends motor commands. It automatically detects the official TCP
streamer or Bumblebee's optional UDP streamer; use `--transport tcp` or
`--transport udp` to select one explicitly while troubleshooting.

Run `./diagnose` first and continue only when Bumblebee reports
**READY FOR FLIGHT**.

The default hover rises to 0.30 m, holds for five seconds, and lands. Requested
heights may not exceed 1.50 m, and live position telemetry triggers landing if
the drone leaves its 1.50 m altitude or takeoff-radius boundary:

```bash
./hover --height 0.25 --seconds 3
```

During flight:

- Press **Return** to land early.
- Press **Control-C** to request landing and disarm cleanup.

### Lift and recovery measurement — disabled

Firmware v6 temporarily disabled the flip and added an upright measurement
routine. Its first test produced only 0.01 m of altitude gain and 0.12 m/s of
upward speed, followed by a sideways loss of control. The routine is therefore
locked. Firmware v8 also rejects this direct-thrust request, and the Mac
requires a future v9 or newer implementation before it can be armed.

Do not restore an older Mac revision to repeat the v6 lift test. The next step
is a normal-controller motor and battery health measurement, not another direct
thrust pulse.

### Motor and power test

Before another flip profile, measure the real airframe under the stock flight
controller:

```bash
./fly --flight power-test
```

The test holds 0.40 m, makes one one-second climb at the normal Safe speed
limit, and lands. It records all four actual and requested motor outputs,
battery sag, tilt, climb response, and motor saturation. It never sends direct
thrust or disables tumble protection. The result tells us whether the current
battery and AI-deck/Flow-deck airframe have enough clean motor headroom to tune
the next phase-based flip profile.

### Experimental flip

The flip is deliberately separate from the normal Safe routines. It requires
Bumblebee's custom Crazyflie firmware, a freshly charged battery, eye
protection, and a padded test area with at least 4 × 4 m of clear floor and 3 m
overhead.

```bash
./fly --flight flip
```

The command stages at 1.15 m, learns the current airframe's hover thrust, and
asks for exact `ACRO` and `FLIP` confirmations before triggering anything. The
maneuver triggers only after a one-second low-drift window and aborts if the
staging hover moves more than 0.5 m from takeoff. It then runs on the Crazyflie
so radio timing cannot split the flip into half-finished phases. Only the tumble
detector is suspended, for at most 1.5 seconds; the other supervisor checks
remain active. A radio-loss watchdog cuts motor power if its Mac heartbeat
disappears. Power-cycle the Crazyflie after every Acro run to reset that
watchdog.

Version 5 completed a full rotation on the AI-deck and Flow-deck airframe but
lost too much height to recover safely. Version 6's direct lift measurement
then destabilized sideways. Firmware v7 introduced a phase-controlled profile,
but padded tests showed that passive zero-rate braking still left more than
430°/s of pitch rate after a complete rotation. Firmware v8 begins braking
earlier with an active counter-rotation command and uses an onboard disarm for
every unstable abort, so shutdown no longer depends on the radio or Mac. The
Mac records live altitude, tilt, vertical speed, phase transitions, motor
saturation, and battery sag. Guarded phase values are sent before every run.
Do not restore an older Acro profile.

Safe flights and the lift test land at 3.50 V. Any future flip profile must also
pass an under-load battery check before its rotation can begin.

Do not run the flip indoors near people, pets, walls, windows, or furniture.

---

## 📁 Project Structure

```text
bumblebee/
├── battery, camera, diagnose, fly, hover
│                                      Friendly command shortcuts
├── firmware/                         Reproducible camera and Acro firmware
├── src/bumblebee/
│   ├── camera/                      AI-deck streaming and recording
│   ├── commands/                    Command-line workflows
│   ├── flights/                     Independent flight routines
│   ├── ui/                          Terminal dashboards
│   └── *.py                         Radio, telemetry, and safety logic
└── tests/                           Hardware-independent tests
```

---

## 🧪 Quality Checks

```bash
uv run python -m unittest discover -s tests
uv run python -m compileall -q src tests
```

These checks do not connect to the drone or command its motors.
