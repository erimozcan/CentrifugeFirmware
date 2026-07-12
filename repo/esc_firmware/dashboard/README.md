# Centrifuge spindle dashboard (Phase 3)

Localhost web UI that drives the B-G431B-ESC1 SimpleFOC firmware and shows the
live **absolute encoder angle**, **velocity vs setpoint**, and **ramp** controls.
Talks to the ESC over the ST-LINK Virtual COM Port — the same USB cable used to
flash it. No internet/CDN needed; the UI is self-contained.

## Run

```
cd repo/esc_firmware/dashboard
pip install -r requirements.txt
python app.py                 # auto-detects the ST-LINK COM port, lists all ports
# or pin the port explicitly:
python app.py --port COM5
```

Then open <http://127.0.0.1:8000>.

The app keeps serving even if the ESC isn't plugged in yet — the badge reads
*disconnected* and flips to *connected* once the serial port opens (it auto-retries).

## What you get

- **Absolute angle gauge** — `ang` from the AS5047P, 0–360°.
- **Velocity plot** — measured `vel` (blue) vs ramped `tgt` setpoint (grey), rolling.
- **Controls** — Enable / Disable, Target (rev/s), Ramp accel (rev/s²), Stop (ramp to 0).

## How it maps to the firmware protocol

Buttons send the single-line commands the firmware parses
([../src/main.cpp](../src/main.cpp)): `E`, `D`, `V <rev/s>`, `A <rev/s²>`, `S`.
Telemetry comes from the `STAT ang=.. vel=.. tgt=.. cmd=.. en=..` lines streamed
at ~50 Hz; the browser refreshes at 20 Hz over a websocket.

## Bring-up tip

Start the firmware first and confirm `STAT` lines in a plain serial monitor, then
launch this. First real spin: Enable → set Target = 1 rev/s → watch the angle
gauge rotate and the velocity track the setpoint smoothly up from zero.
