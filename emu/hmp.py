#!/usr/bin/env python3
"""Run HMP commands through a QEMU QMP TCP port (clean output).

Start QEMU with: -qmp tcp:127.0.0.1:<port>,server,nowait
usage: emu/hmp.py <port> <hmp command> [<hmp command> ...]
"""
import json
import socket
import sys


def main():
    s = socket.create_connection(("127.0.0.1", int(sys.argv[1])))
    f = s.makefile("rw")
    json.loads(f.readline())                        # greeting
    f.write(json.dumps({"execute": "qmp_capabilities"}) + "\n")
    f.flush()
    json.loads(f.readline())
    for cmd in sys.argv[2:]:
        f.write(json.dumps({"execute": "human-monitor-command",
                            "arguments": {"command-line": cmd}}) + "\n")
        f.flush()
        while True:
            r = json.loads(f.readline())
            if "return" in r or "error" in r:
                break
        print("## " + cmd)
        print(r.get("return", r.get("error")), end="")


if __name__ == "__main__":
    main()
