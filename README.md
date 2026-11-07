# Bumblebee 🐝

Bumblebee is a small terminal companion for my Bitcraze Crazyflie 2.1+.
It provides friendly diagnostics, live battery telemetry, and guarded flight routines.

---

## ✨ Features

- Styled preflight diagnostics
- Live, human-readable battery monitoring
- A flight selector for current and future routines
- A guarded takeoff, hover, and landing routine
- Automatic landing when a safety check fails
- Flow deck V2 support with the AI-deck left safely idle

---

## 🔩 Hardware

- Crazyflie 2.1+
- Flow deck V2
- AI-deck 1.1
- Crazyradio PA
- macOS computer

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
./fly        # choose a flight routine
./hover      # open the hover routine directly
```

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
├── battery, diagnose, fly, hover    Friendly command shortcuts
├── src/bumblebee/
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
