#!/usr/bin/env python3
"""Interactive NuttX shell (NSH) on the emulated band's console UART.

The production firmware has no shell: its init task runs
/etc/init.d/rcS through NSH, and the script's last line is "exit", so the
init NSH ends before it ever reads the console (no TTY_LAUNCH key either).

`inject` works like a debug probe on the real board.  It attaches to the
gdbstub of a machine started paused (-S -gdb tcp:...), lets the firmware
boot, and right after nx_bringup has spawned the init task it calls the
firmware's own task_spawn() once more, with the same attributes, to start
its built-in "sh" (nsh_system: an interactive session without rcS).  The
new task inherits the boot console, so the shell runs on the console UART
next to the normal boot.  No firmware byte is changed and nothing is
hard-coded; everything is located in the image:
  - the image layout from its header and boot info (as hw/misc/bes_image.c)
  - nx_bringup's task_spawn call: the literal load of the "nsh_main" task
    name followed by a BL
  - "sh": the NuttX builtin table ({name, priority, stack size, main})

`console` connects a raw terminal to the UART's TCP port (Ctrl-] quits).

emu/run.sh does both with NSH=<port>.
usage: emu/nsh.py inject <gdb-port> <vela_ap.bin>
       emu/nsh.py console <port>
"""
import os
import re
import select
import socket
import struct
import sys
import termios
import time
import tty


class Image:
    """Firmware layout, parsed like hw/misc/bes_image.c."""

    def __init__(self, path):
        self.d = open(path, "rb").read()
        mapped = self.u32_off(0x64)
        self.xip = (mapped & ~1) - 0x14
        cfg = {}
        for key in (b"FLASH_BASE", b"FLASH_NC_BASE", b"OTA_CODE_OFFSET"):
            i = self.d.find(key + b"=")
            end = min(self.d.find(c, i) for c in (b"\n", b"\0"))
            cfg[key] = int(self.d[i + len(key) + 1:end], 0)
        ota = cfg[b"OTA_CODE_OFFSET"]
        self.aliases = (self.xip, cfg[b"FLASH_BASE"] + ota,
                        cfg[b"FLASH_NC_BASE"] + ota)

    def u32_off(self, off):
        return struct.unpack_from("<I", self.d, off)[0]

    def off(self, addr):
        for base in self.aliases:
            if base <= addr < base + len(self.d):
                return addr - base
        return None

    def u16(self, addr):
        return struct.unpack_from("<H", self.d, self.off(addr))[0]

    def cstr(self, addr):
        o = self.off(addr)
        if o is None:
            return None
        return self.d[o:self.d.find(b"\0", o)]


def builtin(img, name):
    """main and name pointer of builtin `name` (NuttX g_builtins table:
    {name, priority, stack size, main} per entry)."""
    ent = (rb"[\s\S]{4}[\s\S]\x00\x00\x00[\s\S]{2}[\x00-\x0f]\x00"
           rb"[\s\S]{3}" + re.escape(bytes([img.xip >> 24])))
    for m in re.finditer(rb"(?:%s){8,}" % ent, img.d):
        off = m.start() + (-m.start() % 4)
        while off + 16 <= m.end():
            nm, _, _, main = struct.unpack_from("<IIII", img.d, off)
            if main & 1 and img.cstr(nm) == name.encode():
                return main, nm
            off += 16
    sys.exit("nsh: builtin %r not found" % name)


def bl_target(img, at):
    h1, h2 = img.u16(at), img.u16(at + 2)
    if (h1 & 0xF800) != 0xF000 or (h2 & 0xD000) != 0xD000:
        return None
    s = (h1 >> 10) & 1
    i1 = ~((h2 >> 13) & 1 ^ s) & 1
    i2 = ~((h2 >> 11) & 1 ^ s) & 1
    imm = (s << 24) | (i1 << 23) | (i2 << 22) | ((h1 & 0x3FF) << 12) | \
        ((h2 & 0x7FF) << 1)
    if s:
        imm -= 1 << 25
    return at + 4 + imm


def bringup_spawn(img):
    """(BL address, task_spawn) of nx_bringup's init task creation."""
    sites = set()
    i = img.d.find(b"nsh_main\0")
    while i >= 0:
        for base in img.aliases:
            lit = struct.pack("<I", base + i)
            j = img.d.find(lit)
            while j >= 0:
                if j % 4 == 0:
                    sites.add(img.xip + j)
                j = img.d.find(lit, j + 1)
        i = img.d.find(b"nsh_main\0", i + 1)
    found = set()
    for lit in sites:
        for pc in range(max(img.xip, lit - 1020) & ~1, lit, 2):
            h = img.u16(pc)
            # LDR r0, [pc, #imm8 * 4]
            if (h & 0xFF00) != 0x4800 or \
                    ((pc + 4) & ~3) + (h & 0xFF) * 4 != lit:
                continue
            for at in range(pc + 2, pc + 24, 2):
                t = bl_target(img, at)
                if t is not None:
                    found.add((at, t))
                    break
    if len(found) != 1:
        sys.exit("nsh: nx_bringup task_spawn call not found (%d)" % len(found))
    return found.pop()


class Rsp:
    """Minimal GDB remote serial protocol client."""

    def __init__(self, port):
        for _ in range(100):
            try:
                self.s = socket.create_connection(("127.0.0.1", port))
                break
            except OSError:
                time.sleep(0.1)
        else:
            sys.exit("nsh: no gdbstub on port %d" % port)
        self.buf = b""

    def _read(self):
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
                sys.exit("nsh: gdbstub closed")
            self.buf += d

    def send(self, cmd):
        c = sum(cmd.encode("latin1")) & 0xFF
        self.s.sendall(b"$%s#%02x" % (cmd.encode("latin1"), c))
        return self._read()

    def reg(self, n):
        return struct.unpack("<I", bytes.fromhex(self.send("p%x" % n)))[0]

    def setreg(self, n, v):
        self.send("P%x=%s" % (n, struct.pack("<I", v).hex()))

    def mem32(self, addr):
        return struct.unpack("<I", bytes.fromhex(self.send("m%x,4" % addr)))[0]

    def run_to(self, addr, sp=None):
        self.send("Z0,%x,2" % addr)
        while True:
            self.send("c")
            if self.reg(15) == addr and (sp is None or self.reg(13) == sp):
                break
        self.send("z0,%x,2" % addr)


def inject(port, fw):
    img = Image(fw)
    sh_main, sh_name = builtin(img, "sh")
    call, spawn = bringup_spawn(img)
    ret = call + 4
    g = Rsp(port)
    g.send("?")
    g.run_to(call)
    attr = g.reg(3)                     # posix_spawnattr of the init task
    g.run_to(ret)
    saved = {n: g.reg(n) for n in list(range(16)) + [25]}
    # task_spawn("sh", sh_main, NULL, attr, argv, envp): argv / envp are
    # still on the stack where nx_bringup put them for the first call
    g.setreg(0, sh_name)
    g.setreg(1, sh_main)
    g.setreg(2, 0)
    g.setreg(3, attr)
    g.setreg(14, ret | 1)
    g.setreg(15, spawn & ~1)
    g.run_to(ret, sp=saved[13])
    pid = g.reg(0)
    for n, v in saved.items():
        g.setreg(n, v)
    g.send("D")
    if pid >= 0x80000000:
        sys.exit("nsh: task_spawn failed (%d)" % (pid - (1 << 32)))
    print("nsh: shell started (pid %d)" % pid)


def console(port):
    s = socket.create_connection(("127.0.0.1", port))
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("connected to 127.0.0.1:%d, Ctrl-] quits" % port)
    tty.setraw(fd)
    try:
        while True:
            r, _, _ = select.select([s, fd], [], [])
            if s in r:
                d = s.recv(4096)
                if not d:
                    break
                os.write(sys.stdout.fileno(), d)
            if fd in r:
                d = os.read(fd, 1024)
                if b"\x1d" in d:
                    break
                s.sendall(d)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "inject":
        inject(int(sys.argv[2]), sys.argv[3])
    elif len(sys.argv) == 3 and sys.argv[1] == "console":
        console(int(sys.argv[2]))
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
