import argparse
import sys

from centrifuge import CentrifugeClient, RunProfile
from centrifuge.exceptions import CentrifugeError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Centrifuge serial CLI")
    parser.add_argument("--port", required=True, help="Serial port (example: COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=1.0, help="Command timeout in seconds")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ping")
    sub.add_parser("version")
    sub.add_parser("status")
    sub.add_parser("init")

    run = sub.add_parser("run")
    run.add_argument("--lift", type=int, required=True)
    run.add_argument("--final", type=int, required=True)
    run.add_argument("--seat", type=int, required=True)
    run.add_argument("--hold", type=int, required=True)
    run.add_argument("--rampup", type=int, required=True)
    run.add_argument("--rampdown", type=int, required=True)

    sub.add_parser("abort")
    sub.add_parser("hardstop")
    sub.add_parser("clear-fault")
    sub.add_parser("lock")
    sub.add_parser("unlock")
    sub.add_parser("door-open")
    sub.add_parser("door-close")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        with CentrifugeClient(port=args.port, baudrate=args.baud, timeout_s=args.timeout) as client:
            if args.command == "ping":
                response = client.ping()
                print(response)
            elif args.command == "version":
                response = client.version()
                print(response)
            elif args.command == "status":
                status = client.status()
                print(status)
            elif args.command == "init":
                response = client.init()
                print(response)
            elif args.command == "run":
                profile = RunProfile(
                    lift=args.lift,
                    final=args.final,
                    seat=args.seat,
                    hold=args.hold,
                    rampup=args.rampup,
                    rampdown=args.rampdown,
                )
                response = client.run(profile)
                print(response)
            elif args.command == "abort":
                response = client.abort()
                print(response)
            elif args.command == "hardstop":
                response = client.hardstop()
                print(response)
            elif args.command == "clear-fault":
                response = client.clear_fault()
                print(response)
            elif args.command == "lock":
                response = client.lock()
                print(response)
            elif args.command == "unlock":
                response = client.unlock()
                print(response)
            elif args.command == "door-open":
                response = client.door_open()
                print(response)
            elif args.command == "door-close":
                response = client.door_close()
                print(response)
            else:
                parser.error("unknown command")
    except CentrifugeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
