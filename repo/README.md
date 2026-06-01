# Centrifuge Gen1 Barebones Repository

## Architecture

This repository is split into:
- `firmware/`: Arduino Due firmware with deterministic 1 kHz scheduling.
- `python/`: pyserial library + CLI.
- `WIRING.md`: full hardware wiring layout and bring-up checks.
- `PINOUT.md`: concise pin-by-pin connection map for Arduino Due.

Firmware modules:
- `SystemContext` and `PendingCommand`
- `StateMachine`
- `MotorInterface`
- `HardwareGuard`
- `CommandInterface`
- `Protocol`

Key design points:
- no blocking calls (`delay`, blocking reads, `Serial.flush` not used)
- fixed-point RPM ramps (`RPM_SCALE`)
- atomic pending-command handoff at tick start
- strict state transition enforcement
- hard stop drops enable and latches fault
- lock is sensor-confirmed, not actuator-assumed

## Bring-up Procedure

1. Configure pins in `firmware/src/config.h`.
2. Upload firmware from `firmware/src/main.ino` to Arduino Due.
3. Install Python dependencies:

```bash
cd python
python -m venv .venv
. .venv/Scripts/activate
pip install -r requirements.txt
```

4. Test command path:

```bash
python cli.py --port COM5 ping
python cli.py --port COM5 version
python cli.py --port COM5 status
python cli.py --port COM5 init
```

5. Lock and run a low-speed profile:

```bash
python cli.py --port COM5 lock
python cli.py --port COM5 run --lift 120 --final 300 --seat 500 --hold 1000 --rampup 1200 --rampdown 1200
```

## Safety Warning

`MAX_RPM_BAREBONES` is hard-limited in firmware and defaults to `500` RPM.
Do not raise this value until mechanical containment, lock validation, and staged hardware safety validation are complete.
