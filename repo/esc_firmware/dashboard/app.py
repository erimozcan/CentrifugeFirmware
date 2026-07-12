"""
Centrifuge spindle dashboard - localhost bridge between the browser UI and the
B-G431B-ESC1 SimpleFOC firmware over serial (the ST-LINK Virtual COM Port).

  pip install -r requirements.txt
  python app.py                      # auto-lists ports, serves http://127.0.0.1:8000
  python app.py --port COM5          # pin a specific serial port
  ESC_PORT=COM5 python app.py        # ...or via env var

Protocol (matches repo/esc_firmware/src/main.cpp):
  send  -> "E"/"D"/"V <rev/s>"/"A <rev/s2>"/"S"
  recv  <- "STAT ang=<rad> vel=<rev/s> tgt=<rev/s> cmd=<rev/s> en=<0|1>"
"""
from __future__ import annotations

import argparse
import asyncio
import os
import threading
import queue
import time
from pathlib import Path

import serial
import serial.tools.list_ports
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

BAUD = 115200
STATIC = Path(__file__).parent / "static"

# Shared state between the serial thread and the websocket handlers.
_telemetry = {"ang": 0.0, "vel": 0.0, "tgt": 0.0, "cmd": 0.0, "en": 0, "connected": False}
_telemetry_lock = threading.Lock()
_outgoing: "queue.Queue[str]" = queue.Queue()
_stop = threading.Event()


def _parse_stat(line: str):
    """'STAT ang=1.23 vel=0.5 ...' -> dict of floats, or None."""
    if not line.startswith("STAT"):
        return None
    out = {}
    for tok in line.split()[1:]:
        if "=" in tok:
            k, _, v = tok.partition("=")
            try:
                out[k] = float(v)
            except ValueError:
                pass
    return out or None


def serial_worker(port: str):
    """Owns the serial port: reads telemetry, writes queued commands, reconnects."""
    while not _stop.is_set():
        try:
            ser = serial.Serial(port, BAUD, timeout=0.1)
        except (serial.SerialException, OSError) as e:
            with _telemetry_lock:
                _telemetry["connected"] = False
            print(f"[serial] cannot open {port}: {e}; retrying in 2s")
            time.sleep(2)
            continue

        print(f"[serial] connected on {port}")
        with _telemetry_lock:
            _telemetry["connected"] = True
        try:
            while not _stop.is_set():
                # Drain any pending commands first.
                try:
                    while True:
                        cmd = _outgoing.get_nowait()
                        ser.write((cmd.strip() + "\n").encode())
                except queue.Empty:
                    pass

                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                stat = _parse_stat(line)
                if stat:
                    with _telemetry_lock:
                        _telemetry.update(stat)
        except (serial.SerialException, OSError) as e:
            print(f"[serial] lost {port}: {e}")
        finally:
            with _telemetry_lock:
                _telemetry["connected"] = False
            try:
                ser.close()
            except Exception:
                pass


app = FastAPI()


@app.get("/")
async def index():
    return HTMLResponse((STATIC / "index.html").read_text(encoding="utf-8"))


@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()

    async def push():
        while True:
            with _telemetry_lock:
                snap = dict(_telemetry)
            await sock.send_json(snap)
            await asyncio.sleep(0.05)  # 20 Hz UI refresh

    async def recv():
        while True:
            msg = await sock.receive_json()
            cmd = msg.get("cmd")
            if cmd:
                _outgoing.put(str(cmd))

    pusher = asyncio.create_task(push())
    try:
        await recv()
    except WebSocketDisconnect:
        pass
    finally:
        pusher.cancel()


def pick_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = list(serial.tools.list_ports.comports())
    print("[serial] available ports:")
    for p in ports:
        print(f"   {p.device}  {p.description}")
    # Heuristic: prefer something that looks like an ST-LINK VCP.
    for p in ports:
        if "STLink" in p.description or "ST-Link" in p.description or "Virtual COM" in p.description:
            return p.device
    return ports[0].device if ports else "COM5"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=os.environ.get("ESC_PORT"), help="serial port, e.g. COM5")
    ap.add_argument("--http-port", type=int, default=8000)
    args = ap.parse_args()

    com = pick_port(args.port)
    print(f"[serial] using {com}")
    threading.Thread(target=serial_worker, args=(com,), daemon=True).start()

    print(f"[http] open http://127.0.0.1:{args.http_port}")
    uvicorn.run(app, host="127.0.0.1", port=args.http_port, log_level="warning")
