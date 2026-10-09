#!/usr/bin/env python3
"""Sample guest PC/LR via a QEMU HMP monitor on a TCP port.

usage: emu/sample_pc.py <port> <count> <interval_s>
"""
import re
import socket
import sys
import time


def hmp(port, cmd):
    s = socket.create_connection(("127.0.0.1", port))
    s.settimeout(2)
    buf = b""
    try:
        while b"(qemu)" not in buf:
            buf += s.recv(4096)
        s.sendall(cmd.encode() + b"\n")
        buf = b""
        while buf.count(b"(qemu)") < 1:
            buf += s.recv(65536)
    except socket.timeout:
        pass
    s.close()
    return buf.decode(errors="replace")


def main():
    port, count, interval = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
    for _ in range(count):
        out = hmp(port, "info registers")
        regs = dict(re.findall(r"(R\d\d)=([0-9a-f]{8})", out))
        xpsr = re.search(r"XPSR=([0-9a-f]{8})", out)
        print("pc=%s lr=%s sp=%s xpsr=%s" % (regs.get("R15"), regs.get("R14"),
              regs.get("R13"), xpsr.group(1) if xpsr else "?"))
        time.sleep(interval)


if __name__ == "__main__":
    main()
