# Python Client and CLI

## Install

```bash
python -m venv .venv
. .venv/Scripts/activate
pip install -r requirements.txt
```

## CLI Usage

```bash
python cli.py --port COM5 ping
python cli.py --port COM5 version
python cli.py --port COM5 status
python cli.py --port COM5 init
python cli.py --port COM5 lock
python cli.py --port COM5 run --lift 120 --final 300 --seat 500 --hold 2000 --rampup 1500 --rampdown 1500
python cli.py --port COM5 abort
python cli.py --port COM5 hardstop
python cli.py --port COM5 clear-fault
python cli.py --port COM5 unlock
python cli.py --port COM5 door-open
python cli.py --port COM5 door-close
```

## Library Example

```python
from centrifuge import CentrifugeClient, RunProfile

with CentrifugeClient(port="COM5") as client:
    client.init()
    client.lock()
    client.run(RunProfile(lift=120, final=300, seat=500, hold=2000, rampup=1500, rampdown=1500))
    print(client.status())
```

Client guarantees:
- one in-flight command at a time
- input buffer reset before each command
- mismatched sequence replies are discarded
- timeout is evaluated while waiting for matching sequence replies
