# Driving the centrifuge from an Opentrons OT-2

The centrifuge's master controller (Arduino Nano ESP32) speaks a simple
sequence-numbered line protocol over USB — the same one the browser console uses
over Web Serial. Plug that USB into the OT-2's Raspberry Pi and a protocol can
drive the machine directly.

| File | What it is |
| --- | --- |
| `centrifuge.py` | The client library. Import it in Jupyter, or on any PC. |
| `centrifuge_control.py` | **Upload this to the Opentrons App.** Control panel with runtime parameters. |
| `console.py` | Interactive text console — the UI's buttons, typed. For debugging. |
| `tools/sync_inline.py` | Copies `centrifuge.py` into the protocol's inline block. |

The Opentrons App uploads a protocol as a **single file**, so
`centrifuge_control.py` carries a verbatim copy of `centrifuge.py` inside it. If
you edit the client, run `python3 repo/opentrons/tools/sync_inline.py` — the test
suite fails if you forget.

## What you need to do

**1. Plug the Nano's USB into any USB port on the OT-2's Raspberry Pi.**
The centrifuge's own 24 V supply still powers the motor and ESC as usual; USB
only carries data (and powers the Nano itself).

**2. Check the robot's software version.** Runtime parameters — the Force/Time
inputs — need OT-2 software **7.3.0 or newer**. The Opentrons App shows this
under the robot's settings. On an older version the upload is rejected with an
API-level error; updating the robot is the fix.

**3. Install pyserial on the robot.** SSH into the OT-2 (set up an SSH key from
the Opentrons App's advanced settings if you haven't) and run:

```bash
python3 -c "import serial; print(serial.__version__)"   # already there?
pip install pyserial                                     # only if that failed
```

Note this can be wiped by a robot software update — if protocols suddenly fail
with a pyserial error after an update, reinstall it.

**4. Upload `centrifuge_control.py`** in the Opentrons App (Protocols → Import).

**5. Run "Report status only" first.** It opens the port, prints the machine's
state into the run log, and touches nothing. If that works, everything else will.
Then try Open door, Go to tube, and finally a short spin.

## Using it

In the app's setup screen for the protocol, pick an **Action** and press Run:

- **Spin (Force + Time)** — closes the door, spins, sweeps the buckets down,
  re-locks the gantry, opens the door. Force is in × g (capped at 6000, matching
  the browser console), Time is the hold at speed, Ramp is spin-up/down seconds.
- **Open door** / **Close door**
- **Go to tube** — indexes the gantry so that tube faces the pipette.
- **Lock gantry** / **Unlock gantry** — debug controls that force the lock in or
  out, the same override the console's buttons use.
- **Set lock travel** — debug: drive the pin to an explicit percentage (0 =
  retracted, 100 = fully in). Used to tune the bucket-sweep depth; the browser
  console has the same thing as a slider in its debug drawer.
- **Report status only**

Before any door or rotor motion the protocol calls `protocol.home()` to get the
pipette out of the way. The centrifuge's own interlocks still apply — it refuses
to spin with the door open and the lock only extends onto a verified detent.

### Interactive debugging

`console.py` runs anywhere Python and pyserial can see the Nano — SSH'd into the
robot, or on the wet-lab PC:

```
$ python3 console.py
connected on /dev/serial/by-id/usb-Arduino_Arduino_Nano_ESP32_...
SAFE IDLE   door closed  tube 1  lock in     0 rpm (0 x g)
centrifuge> open
centrifuge> tube 3
centrifuge> run 4000 5
centrifuge> unlock
```

### From a Jupyter notebook on the robot

The OT-2 serves Jupyter on port 48888. Copy `centrifuge.py` next to your
notebook and drive the machine cell by cell:

```python
from centrifuge import Centrifuge
cf = Centrifuge()
cf.door_open()
cf.rotate(2)
cf.spin(rcf=4000, minutes=5)
```

## Things worth knowing

**Only one program can hold the serial port.** While a protocol is running, the
browser console on the PC cannot connect, and vice versa. To watch the machine
during a robot run, use the WiFi console instead: join the `Centrifuge` access
point and open <http://192.168.4.1/>.

**The client will not grab the robot's own motion controller.** It prefers
stable `/dev/serial/by-id/` names, skips anything that looks like the
smoothieboard, and requires a `VERSION` handshake before it will send commands.
Set `CENTRIFUGE_PORT` to name a port explicitly if you ever need to override
that.

**Faults surface immediately.** Any wait aborts the moment the machine latches a
fault, so a stuck door or a `detent mismatch` fails the protocol with a readable
error instead of hanging until timeout.

## Not done yet: pipetting into the tubes

This protocol deliberately loads no labware and moves no liquid. To have the
robot actually pipette into the carousel you still need a **custom labware
definition** for the centrifuge (via the Opentrons Labware Creator) describing
the tube position at the load point, plus a **Labware Position Check** to
calibrate the offset. Once that exists, the workflow is:

```python
cf.door_open()
for tube in range(1, 5):
    cf.rotate(tube)
    pipette.transfer(volume, source, centrifuge_labware["A1"])
cf.spin(rcf=4000, minutes=5)     # closes the door, spins, opens it again
```

That labware definition depends on the machine's real geometry, so it has to be
measured rather than guessed.
