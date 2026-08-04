#!/usr/bin/env python3
"""Interactive text console for the centrifuge -- the browser UI's buttons, typed.

Runs anywhere Python and pyserial can see the Nano: SSH'd into the OT-2, or on
the wet-lab PC. Handy when the robot is mid-protocol-development and you just
want to drive the machine by hand.

    python3 repo/opentrons/console.py            # auto-detect the port
    python3 repo/opentrons/console.py /dev/ttyACM0

Only one program can hold the serial port, so close the browser console (or stop
a running protocol) first.
"""

import sys
import traceback

try:
    from centrifuge import (Centrifuge, CentrifugeError, MAX_RCF, TUBE_COUNT)
except ImportError:                                    # run from the repo root
    sys.path.insert(0, __file__.rsplit("/", 1)[0])
    from centrifuge import (Centrifuge, CentrifugeError, MAX_RCF, TUBE_COUNT)


HELP = """
  open                     open the door
  close                    close the door
  tube <1-%d>               index the gantry to a tube
  run <rcf> <min> [ramp]   full spin cycle (ramp defaults to 120 s)
  abort                    ramp a running spin down
  estop                    immediate torque cut (latches a fault)
  lock / unlock            DEBUG: force the gantry lock in / out
  home                     set the current angle as tube 1's detent
  power on / power off     device power (fan + lights)
  clear                    clear a latched fault
  status                   print the full STATUS line
  raw <verb ...>           send any firmware verb verbatim
  help / quit
""" % TUBE_COUNT


def _run_spin(cf, args):
    if len(args) < 2:
        print("usage: run <rcf> <minutes> [ramp_seconds]")
        return
    rcf, minutes = float(args[0]), float(args[1])
    ramp = float(args[2]) if len(args) > 2 else 120.0
    if rcf > MAX_RCF:
        print("refusing: %g x g is above the %d x g operator cap" % (rcf, MAX_RCF))
        return
    print("spinning %g x g for %g min (%g s ramps) -- Ctrl-C aborts" % (rcf, minutes, ramp))
    try:
        cf.spin(rcf, minutes, ramp_seconds=ramp,
                on_poll=lambda s: print("   ", cf.describe()))
        print("done;", cf.describe())
    except KeyboardInterrupt:
        print("\naborting...")
        cf.abort()


def dispatch(cf, line):
    parts = line.split()
    if not parts:
        return True
    verb, args = parts[0].lower(), parts[1:]

    if verb in ("quit", "exit", "q"):
        return False
    if verb in ("help", "?"):
        print(HELP)
    elif verb == "open":
        cf.door_open()
    elif verb == "close":
        cf.door_close()
    elif verb == "tube":
        cf.rotate(int(args[0]))
    elif verb == "run":
        _run_spin(cf, args)
    elif verb == "abort":
        cf.abort()
    elif verb == "estop":
        cf.hard_stop()
    elif verb == "lock":
        cf.lock()
    elif verb == "unlock":
        cf.unlock()
    elif verb == "home":
        cf.home()
    elif verb == "power":
        (cf.power_on if args and args[0].lower() == "on" else cf.power_off)()
    elif verb == "clear":
        cf.clear_fault()
    elif verb == "status":
        for key, value in sorted(cf.status().items()):
            print("  %-18s %s" % (key, value))
    elif verb == "raw":
        print(" ", cf.command(" ".join(args)))
    else:
        print("unknown command %r -- try 'help'" % verb)
        return True

    if verb not in ("status", "help", "?", "run"):
        print(" ", cf.describe())
    return True


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        cf = Centrifuge(port=port)
    except CentrifugeError as exc:
        sys.exit(str(exc))

    print("connected on %s" % cf.port)
    print(cf.describe())
    print(HELP)
    try:
        while True:
            try:
                line = input("centrifuge> ")
            except (EOFError, KeyboardInterrupt):
                print()
                break
            try:
                if not dispatch(cf, line):
                    break
            except CentrifugeError as exc:
                print("!", exc)
            except (ValueError, IndexError):
                print("! bad arguments -- try 'help'")
            except Exception:
                traceback.print_exc()
    finally:
        cf.close()


if __name__ == "__main__":
    main()
