#!/usr/bin/env python3
"""Catch the first firmware assertion on any core through the gdbstub.

Sets a breakpoint on each core's NuttX _assert(filename, linenum, msg,
regs) entry, resumes the machine and, when one is hit, prints which core,
the file / line / message and the registers. All cores sit in one gdb
process (QEMU attaches the CPUs of every cluster), so one breakpoint list
covers them; memory is read through the core that stopped.

The default entries are those of the q63 firmware 4.101.020:
  ap      0x1086E6F4  (vela_ap.bin)
  cp      0x1B33DED4  (vela_cp.bin, loaded at 0x1B300000)
  sensor  0x0C061494  (vela_sensor.bin, BTH core code view)
Each is the function that prints "Assertion failed ... at file:"; find
it again for another build from that format string.

Start QEMU paused with a gdbstub (-S -gdb tcp:127.0.0.1:<port>); this
script starts it.
usage: emu/assertwatch.py <port> [--bp NAME=ADDR ...] [--out FILE]
"""
import argparse
import re
import socket
import struct
import sys
import time

DEFAULT = ["ap=0x1086E6F4", "cp=0x1B33DED4", "sensor=0x0C061494"]


class Rsp:
    def __init__(self, port):
        for _ in range(200):
            try:
                self.s = socket.create_connection(("127.0.0.1", port))
                break
            except OSError:
                time.sleep(0.1)
        else:
            sys.exit("assertwatch: no gdbstub on port %d" % port)
        self.buf = b""

    def read(self, timeout=None):
        self.s.settimeout(timeout)
        while True:
            i = self.buf.find(b"$")
            j = self.buf.find(b"#", i + 1) if i >= 0 else -1
            if i >= 0 and j >= 0 and len(self.buf) >= j + 3:
                pkt = self.buf[i + 1:j]
                self.buf = self.buf[j + 3:]
                self.s.sendall(b"+")
                return pkt.decode("latin1")
            d = self.s.recv(65536)
            if not d:
                raise EOFError
            self.buf += d

    def send(self, cmd, wait=True):
        c = cmd.encode("latin1")
        self.s.sendall(b"$%s#%02x" % (c, sum(c) & 0xFF))
        return self.read(10) if wait else None

    def reg(self, n):
        return struct.unpack("<I", bytes.fromhex(self.send("p%x" % n)))[0]

    def mem(self, addr, n):
        try:
            return bytes.fromhex(self.send("m%x,%x" % (addr, n)))
        except ValueError:
            return b""

    def threads(self):
        out, r = [], self.send("qfThreadInfo")
        while r and r[0] == "m":
            out += r[1:].split(",")
            r = self.send("qsThreadInfo")
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", type=int)
    ap.add_argument("--bp", action="append", default=None,
                    help="NAME=ADDR, repeatable (default: q63 4.101.020)")
    ap.add_argument("--out", default="-")
    a = ap.parse_args()
    bps = {}
    for b in a.bp or DEFAULT:
        name, addr = b.split("=")
        bps[int(addr, 0)] = name
    out = sys.stdout if a.out == "-" else open(a.out, "w")

    def say(*x):
        print(*x, file=out, flush=True)

    g = Rsp(a.port)
    g.send("qSupported:multiprocess+")
    for pid in sorted({t.split(".")[0] for t in g.threads()}):
        g.send("vAttach;%s" % pid[1:])
    for addr in bps:
        if g.send("Z0,%x,2" % addr) != "OK":
            say("cannot set a breakpoint at %#x" % addr)
    g.send("c", wait=False)
    try:
        stop = g.read(None)
    except EOFError:
        say("gdbstub closed (QEMU exited) without an assertion")
        return
    if stop.startswith(("W", "X")):
        say("QEMU exited without an assertion")
        return
    m = re.search(r"thread:([^;]+);", stop)
    if m:
        g.send("Hg" + m.group(1))
    pc = g.reg(15)

    def cstr(addr):
        return g.mem(addr, 0x80).split(b"\0")[0].decode("latin1") \
            if addr else ""

    r = [g.reg(n) for n in range(15)]
    say("core %s: %s:%d %s" % (bps.get(pc, "pc %#x" % pc), cstr(r[0]),
                               r[1], cstr(r[2])))
    say("lr %#x sp %#x" % (r[14], r[13]))
    say(" ".join("r%d=%08x" % (n, r[n]) for n in range(13)))
    words = g.mem(r[13], 0x100)
    say("stack " + " ".join("%08x" % w for w in
                            struct.unpack("<%dI" % (len(words) // 4), words)))


if __name__ == "__main__":
    main()
