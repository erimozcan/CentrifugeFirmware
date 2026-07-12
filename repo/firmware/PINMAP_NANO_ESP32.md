# Centrifuge master controller — Arduino Nano ESP32 pin map (FINAL)

Target board: **Arduino Nano ESP32** (u-blox NORA-W106 = ESP32-S3). 3.3 V logic,
native USB to PC, 3 hardware UARTs. This is the intended FINAL master controller
(replaces the Arduino Due prototype). Architecture unchanged: PC web UI ⇄ Nano
⇄ (UART) B-G431B-ESC1 ⇄ spindle motor, with the Nano also driving door, lock,
fan, lights, and audio + owning the safety interlocks/state machine.

## Verified board facts (Arduino docs)
- Dx→GPIO: D2..D13 = GPIO 5,6,7,8,9,10,17,18,21,38,47,48. A0..A7 = GPIO 1,2,3,4,11,12,13,14.
  D0/D1 = GPIO 44/43 (UART0 / USB-console TX-RX silk).
- **ADC1 (usable with WiFi on) = GPIO 1–10 only.** ADC2 (GPIO 11–20) is unreliable
  when WiFi/BLE is active — do NOT use A4–A7 for analog if radio is on.
- **Strapping pin on the headers: A2 = GPIO3** (JTAG sel). Left unused.
- GPIO 0/45/46 = onboard RGB LED, GPIO 48 = onboard yellow LED (also D13/SCK),
  GPIO 19/20 = native USB (internal). None used for I/O here.
- ESP32-S3 GPIOs are **NOT 5 V tolerant** — every sensor/output feeding a GPIO
  must be ≤ 3.3 V. PWM is via LEDC (any output GPIO); servo via the ESP32 Servo lib.

## Pin assignments

| Function | Nano pin | GPIO | Peripheral | Notes |
|---|---|---|---|---|
| ESC link **TX** → ESC RX | D2 | 5 | Serial1 (UART1) | 3.3 V, crossover, common GND |
| ESC link **RX** ← ESC TX | D3 | 6 | Serial1 | 3.3 V |
| DFPlayer Pro **TX** → DF RX | D4 | 7 | Serial2 (UART2) @115200 | direct (3.3 V, no resistor needed) |
| DFPlayer Pro **RX** ← DF TX | D5 | 8 | Serial2 | direct |
| Door motor DRV8871 **IN1** | D6 | 9 | LEDC PWM ~20 kHz | bidirectional |
| Door motor DRV8871 **IN2** | D7 | 10 | LEDC PWM ~20 kHz | bidirectional |
| Fan DRV8871 **IN1** | D8 | 17 | LEDC PWM | speed control |
| Fan DRV8871 **IN2** | D9 | 18 | LEDC PWM | may tie LOW to save pin |
| Lock actuator **servo signal** | D10 | 21 | LEDC 50 Hz (Servo) | PQ12-R, 6 V power, 1–2 ms pulse |
| **WS2812** LED strip data | D11 | 38 | RMT (Adafruit_NeoPixel / led_strip) | see level-shift note |
| Door hall **OPEN** | A0 | 1 | digital in (INPUT_PULLUP) | sensor powered at 3.3 V |
| Door hall **CLOSED** | A1 | 2 | digital in (INPUT_PULLUP) | sensor powered at 3.3 V |
| *(A4 free)* | A4 | 11 | — | DFPlayer Pro has no BUSY pin (status via AT query) |
| *(reserved for NTC temp)* | A3 | 4 | ADC1_CH3 | dropped for now; ADC1-safe slot kept |
| *(spare)* | D0,D1,D12,D13,A2,A5,A6,A7 | 44,43,47,48,3,12,13,14 | — | D0/D1 = USB-console; A2 strapping (leave) |

LEDC channel budget (S3 has 8 ch / 4 timers): Door+Fan 4 ch on one ~20 kHz timer,
servo 1 ch on a 50 Hz timer; WS2812 uses RMT (no LEDC) → comfortably fits.

## Power rails
- **12 V** — fan, door motor (DRV8871 VM), ESC bus.
- **5 V** — lock actuator (PQ12-63-6-R), DFPlayer VCC, WS2812 strip. **Single buck from 12 V.**
- **3.3 V** — Nano logic, door hall sensors. Common ground across everything.

> **Power decision (2026-07-07):** the separate **6 V buck and the diode-drop scheme are
> cancelled**. The lock actuator, DFPlayer, and WS2812 strip all run off **one shared 5 V
> rail**. Consequences accepted/handled:
> - Actuator is a 6 V unit → ~17 % less force/speed at 5 V (fine for the lock).
> - No diodes anywhere. **DFPlayer keeps its decoupling caps** (1000 µF + 100 nF at VCC) —
>   the caps address the amp's current spikes + motor/LED noise on the shared rail, not the
>   voltage level, so they stay.
> - Size the 5 V buck for the sum: actuator stall ~0.55 A + WS2812 (~60 mA × LED count) +
>   DFPlayer ~0.2 A + margin. Keep the actuator (a motor) decoupled from the DFPlayer.

## Per-component wiring notes
- **ESC (motor sub-assembly):** Nano D2(TX)→ESC RX, ESC TX→Nano D3(RX), GND-GND.
  Both 3.3 V — no level shifter. ESC runs the proven SPIN/STOP/ESTOP/PING/STATUS?
  protocol; Nano firmware speaks it on Serial1 (port of the existing Due bridge).
- **DRV8871 ×2:** logic-high threshold ~2 V, so 3.3 V drives them fine. PWM one IN,
  set/PWM the other for direction (door) or hold LOW (fan). VM = motor supply.
- **PQ12-63-6-R lock:** 3-wire RC servo — **WHITE = signal (→ D10), RED = +V, BLACK = GND**
  (Actuonix datasheet Rev D; ⚠️ reversing RED/BLACK can damage it). Power V+ at **5 V**
  (shared rail). Pulse: **1.0 ms = fully EXTEND = LOCKED, 2.0 ms = fully RETRACT = UNLOCKED**
  (config.h set to match). **No level shifter — the actuator accepts the Nano's 3.3 V pulse
  directly** (the earlier "needs 5 V" conclusion was traced to a firmware pin-remap bug, not
  signal level; confirmed working at 3.3 V on the bench).
  **No position feedback on the cable** → lock state is open-loop (trust the command).
- **Door halls ×2:** power at **3.3 V** (NOT 5 V — S3 not tolerant). Use INPUT_PULLUP;
  HIGH/LOW polarity per existing config (open hall HIGH = OPEN, closed hall HIGH = CLOSED).
- **DFPlayer PRO v1.0 (DFR0768)** — NOT the Mini (different pinout/protocol). USB-C onboard
  128 MB flash (load MP3s by plugging USB-C into a PC → mounts as a U-disk; no SD card).
  12-pin: VIN(1) GND(2) RX(3) TX(4) DACR(5) DACL(6) L+(7) L-(8) R+(9) R-(10) PLAY(11) KEY(12).
  UART **115200, AT commands** (`DFRobot_DF1201S` library — NOT DFPlayerMini). Nano D4→Pro RX,
  Pro TX→Nano D5, **direct (3.3 V, no 1 kΩ needed)**. VCC **5 V (shared rail)**. **Keep
  1000 µF + 100 nF at VIN/GND** at the module (stereo amp still spikes the rail). Speaker
  (mono) across **L+/L-** (built-in amp; 8 Ω 2-3 W recommended — gentler on the 5 V rail than
  4 Ω). No BUSY pin (status via AT query) → A4 free. DACR/DACL = line-out to an external amp.
- **WS2812 strip:** 1 data wire from D11, powered from the shared **5 V** rail (sized for
  full-white ~60 mA/LED). Data is 3.3 V into a 5 V strip → with the 4.3–4.5 V diode trick
  now gone, the first-pixel-glitch risk returns; if it misbehaves add a 74AHCT125 / level
  shifter on the data line (a spare BSS138 channel works) or use a 3.3 V-tolerant strip.

## Firmware port notes (Due → Nano ESP32)
- New PlatformIO env: `platform = espressif32`, `board = arduino_nano_esp32`,
  `framework = arduino`. `Serial` = USB CDC (PC); `Serial1` = ESC; `Serial2` = DFPlayer.
- Replace `analogWrite()` (Due) with LEDC (`ledcSetup`/`ledcAttachPin`/`ledcWrite`
  or core 3.x `ledcAttach`). Use the ESP32 `Servo` lib for the lock.
- Keep config.h pin names, just repoint the numbers to the table above.
- Carry over motor_interface (ESC verbs), hardware_guard (DRV8871/servo/LED),
  command_interface/protocol/state_machine largely as-is.
