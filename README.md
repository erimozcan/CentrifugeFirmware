# Benchtop Centrifuge — Firmware & Operator UI

Complete software for the Tripathi Lab benchtop swing-bucket centrifuge:
master-controller firmware, spindle motor-controller (ESC) firmware, the
browser-based operator console, and the voice-line audio files.

```
[Operator PC / browser]
        │ USB (Web Serial, Chrome/Edge)
        ▼
[Arduino Nano ESP32 — master controller]  repo/firmware/
   • safety interlocks + state machine (1 kHz tick)
   • door motor + hall sensors, gantry lock, fan, LED ring, voice audio
        │ UART, 3.3 V (high-level verbs: SPIN / STOP / ESTOP / PING / STATUS?)
        ▼
[ST B-G431B-ESC1 — spindle ESC]           repo/esc_firmware/
   • SimpleFOC closed-loop velocity control, AS5047P encoder (ABI)
   • autonomous ramping + comms watchdog (fail-safe ramp-down)
        │ 3-phase
        ▼
[EMAX GT2215/10 spindle motor + rotor]
```

## Quick start — operating the device over USB

1. Connect the centrifuge's USB cable to a PC with **Chrome or Edge**
   (Web Serial is not available in Firefox/Safari).
2. Open the operator console — either:
   - **`repo/ui/centrifuge-usb.html`** — fully self-contained single file
     (videos embedded); open it directly in the browser, or
   - **`repo/ui/index.html`** — same console, served from disk:
     `python -m http.server 8080 --directory repo/ui` then
     browse to `http://localhost:8080/`.
3. Click **Connect** and pick the device's COM port.
4. Press **⏻ Power on**. Fan and lighting come on; the console unlocks.
5. Use the three panels: **Door** (open/close), **Tube** (rotate the gantry to
   tube 1–4), **Centrifuge** (speed + time → Run). **E-STOP** is always in the
   header. Diagnostics and a raw serial console are in the dropdowns at the
   bottom of the page.

The device also has a WiFi mode (it broadcasts an access point and serves the
same console at `http://192.168.4.1/`), but **USB is the supported operator
path**.

## Repository layout

| Path | What it is |
|---|---|
| `repo/firmware/` | Master-controller firmware (Arduino Nano ESP32, PlatformIO). The state machine, interlocks, door/lock/fan/LED/audio drivers, and the ESC bridge. |
| `repo/esc_firmware/` | Spindle ESC firmware (B-G431B-ESC1, PlatformIO + SimpleFOC). Owns all closed-loop motor control. |
| `repo/ui/` | Operator console (single HTML file + video assets) and the self-contained `centrifuge-usb.html`. |
| `CentrifugeVoiceLines/` | Source MP3 voice lines loaded onto the DFPlayer PRO's internal storage. |

Detailed docs live next to the code:

- [repo/firmware/README.md](repo/firmware/README.md) — build/flash, serial
  command reference, service/diagnostic firmwares.
- [repo/firmware/PINMAP_NANO_ESP32.md](repo/firmware/PINMAP_NANO_ESP32.md) —
  the full pin map and power-rail layout of the device as built.
- [repo/esc_firmware/README.md](repo/esc_firmware/README.md) — ESC build
  facts, protocol, reflashing.
- [repo/esc_firmware/WIRING.md](repo/esc_firmware/WIRING.md) and
  [BRINGUP.md](repo/esc_firmware/BRINGUP.md) — spindle wiring and the original
  staged bring-up procedure (kept for service reference).

## Rebuilding / reflashing

Both firmware projects use [PlatformIO](https://platformio.org/).

**Master controller** (Nano ESP32, over its USB port):

```
pio run -d repo/firmware -e nano_esp32 -t upload
```

**Spindle ESC** — requires an external ST-LINK wired to the SWD pads on the
ESC main board (the ST-LINK daughterboard was removed because it blocks the
UART link to the master — see `repo/esc_firmware/README.md`):

```
pio run -d repo/esc_firmware -t upload
```

⚠️ The device as shipped is flashed and working. Do not reflash unless you are
changing behavior deliberately; if you do, keep the build flags in each
`platformio.ini` exactly as they are — several are load-bearing (see the ESC
README's "critical build facts").

## Safety model (summary)

- The **master controller owns all interlocks**: the spindle can only run with
  the door closed (hall-sensor confirmed); the door only moves while the rotor
  is stopped; the gantry lock is engaged whenever the machine is at rest and
  released only while spinning/indexing.
- The **ESC is fail-safe on its own**: if it stops hearing from the master for
  ~750 ms it ramps the motor down autonomously; `ESTOP` cuts torque instantly.
- A hard stop latches a fault state; recovery is a power cycle.
