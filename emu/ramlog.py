#!/usr/bin/env python3
"""Dump the guest's in-RAM syslog through a QEMU QMP TCP port.

Once log_service starts, the firmware's syslog no longer reaches uart2; it
is kept in a RAM ring buffer instead. This saves guest memory with QMP
pmemsave, finds the largest cluster of "[MM/DD hh:mm:ss]" log lines and
prints it (or writes it to a file).

Start QEMU with: -qmp tcp:127.0.0.1:<port>,server,nowait
usage: emu/ramlog.py <port> [--base 0x3c000000] [--size 0x2000000] [--out f]
"""
import argparse
import json
import os
import re
import socket
import sys
import tempfile

LINE = re.compile(rb"\[\d\d/\d\d \d\d:\d\d:\d\d\] ")


def pmemsave(port, addr, size, path):
    s = socket.create_connection(("127.0.0.1", port))
    f = s.makefile("rw")
    json.loads(f.readline())
    f.write(json.dumps({"execute": "qmp_capabilities"}) + "\n")
    f.flush()
    json.loads(f.readline())
    f.write(json.dumps({"execute": "pmemsave", "arguments": {
        "val": addr, "size": size, "filename": path}}) + "\n")
    f.flush()
    while True:
        r = json.loads(f.readline())
        if "return" in r:
            return
        if "error" in r:
            sys.exit("pmemsave: %s" % r["error"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", type=int)
    ap.add_argument("--base", type=lambda v: int(v, 0), default=0x3C000000)
    ap.add_argument("--size", type=lambda v: int(v, 0), default=0x2000000)
    ap.add_argument("--out")
    a = ap.parse_args()

    fd, path = tempfile.mkstemp(suffix=".bin")
    os.close(fd)
    try:
        pmemsave(a.port, a.base, a.size, path)
        data = open(path, "rb").read()
    finally:
        os.unlink(path)

    hits = [m.start() for m in LINE.finditer(data)]
    if not hits:
        sys.exit("no log lines found")
    clusters = []
    for h in hits:
        if clusters and h - clusters[-1][1] < 4096:
            clusters[-1][1] = h
            clusters[-1][2] += 1
        else:
            clusters.append([h, h, 1])
    start, end, _ = max(clusters, key=lambda c: c[2])
    end = data.find(b"\n", end)
    text = data[start:end + 1].replace(b"\0", b"").decode("latin1")
    print("# ring buffer around 0x%08x..0x%08x" % (a.base + start, a.base + end),
          file=sys.stderr)
    if a.out:
        open(a.out, "w").write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
