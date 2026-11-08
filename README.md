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

The default hover rises to 0.30 m, holds for five seconds, and lands:

```bash
./hover --height 0.25 --seconds 3
```

During flight:

- Press **Return** to land early.
- Press **Control-C** to request landing and disarm cleanup.

---

## 📁 Project Structure

```text
bumblebee/
├── battery, camera, diagnose, fly, hover
│                                      Friendly command shortcuts
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
