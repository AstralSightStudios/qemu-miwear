#!/usr/bin/env python3
"""Inject touches through a QEMU QMP TCP port (absolute pointer events).

The touch controller model maps the host absolute pointer onto the panel;
the left button is the finger.

Start QEMU with: -qmp tcp:127.0.0.1:<port>,server,nowait
usage: emu/touch.py <port> tap X Y
       emu/touch.py <port> swipe X0 Y0 X1 Y1 [steps] [ms]
Coordinates are panel pixels (default 336x480, see --size).
"""
import argparse
import json
import socket
import time

ABS_MAX = 0x7FFF


class Qmp:
    def __init__(self, port):
        s = socket.create_connection(("127.0.0.1", port))
        self.f = s.makefile("rw")
        json.loads(self.f.readline())
        self.cmd("qmp_capabilities")

    def cmd(self, name, args=None):
        msg = {"execute": name}
        if args is not None:
            msg["arguments"] = args
        self.f.write(json.dumps(msg) + "\n")
        self.f.flush()
        while True:
            r = json.loads(self.f.readline())
            if "return" in r:
                return r["return"]
            if "error" in r:
                raise RuntimeError(r["error"])


def abs_events(x, y, w, h):
    return [
        {"type": "abs", "data": {"axis": "x",
                                 "value": int((x + 0.5) * (ABS_MAX + 1) / w)}},
        {"type": "abs", "data": {"axis": "y",
                                 "value": int((y + 0.5) * (ABS_MAX + 1) / h)}},
    ]


def btn(down):
    return {"type": "btn", "data": {"down": down, "button": "left"}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", type=int)
    ap.add_argument("action", choices=["tap", "swipe"])
    ap.add_argument("coords", type=float, nargs="+")
    ap.add_argument("--size", default="336x480")
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.split("x"))
    q = Qmp(a.port)
    send = lambda ev: q.cmd("input-send-event", {"events": ev})

    if a.action == "tap":
        x, y = a.coords[:2]
        send(abs_events(x, y, w, h) + [btn(True)])
        time.sleep(0.1)
        send([btn(False)])
    else:
        x0, y0, x1, y1 = a.coords[:4]
        steps = int(a.coords[4]) if len(a.coords) > 4 else 10
        ms = a.coords[5] if len(a.coords) > 5 else 300
        send(abs_events(x0, y0, w, h) + [btn(True)])
        for i in range(1, steps + 1):
            time.sleep(ms / 1000.0 / steps)
            t = i / steps
            send(abs_events(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, w, h))
        send([btn(False)])


if __name__ == "__main__":
    main()
