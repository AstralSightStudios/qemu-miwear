#!/usr/bin/env python3
"""List and extract files from the exFAT partitions of an eMMC image.

The best1700 firmware formats its nv, data and userlog partitions as exFAT
(mount -t fatfs -o autoformat); once log_service runs, the syslog goes to
files under /log (the userlog partition). This reads them straight from
the image built by emu/mkemmc.py, without mounting anything.

usage: emu/emmcfs.py <image> <partition> [ls [DIR] | cat PATH | get PATH OUT]
       emu/emmcfs.py emu/emmc-q63.img userlog ls /
       emu/emmcfs.py emu/emmc-q63.img userlog cat /some/file.log
"""

import struct
import sys

SECTOR = 512


def gpt_partitions(f):
    f.seek(SECTOR)
    hdr = f.read(92)
    if hdr[:8] != b"EFI PART":
        sys.exit("no GPT")
    lba, count, size = struct.unpack_from("<QII", hdr, 72)
    f.seek(lba * SECTOR)
    parts = {}
    for _ in range(count):
        e = f.read(size)
        first, last = struct.unpack_from("<QQ", e, 32)
        if first:
            name = e[56:128].decode("utf-16-le").rstrip("\0")
            parts[name] = (first * SECTOR, (last - first + 1) * SECTOR)
    return parts


class ExFat:
    def __init__(self, f, base):
        self.f, self.base = f, base
        b = self.read(0, SECTOR)
        if b[3:11] != b"EXFAT   ":
            sys.exit("not exFAT")
        self.fat_off = struct.unpack_from("<I", b, 0x50)[0]
        self.heap_off = struct.unpack_from("<I", b, 0x58)[0]
        self.root = struct.unpack_from("<I", b, 0x60)[0]
        self.bps = 1 << b[0x6C]
        self.cluster = self.bps << b[0x6D]

    def read(self, off, n):
        self.f.seek(self.base + off)
        return self.f.read(n)

    def chain(self, first, length, contiguous):
        n = max(1, -(-length // self.cluster)) if length else None
        c, out = first, []
        while c >= 2 and c < 0xFFFFFFF7 and (n is None or len(out) < n):
            out.append(c)
            if contiguous:
                c += 1
            else:
                c = struct.unpack("<I", self.read(self.fat_off * self.bps
                                                  + c * 4, 4))[0]
        return out

    def data(self, first, length, contiguous):
        buf = b"".join(self.read(self.heap_off * self.bps +
                                 (c - 2) * self.cluster, self.cluster)
                       for c in self.chain(first, length, contiguous))
        return buf[:length] if length is not None else buf

    def entries(self, first, length=None, contiguous=False):
        raw = self.data(first, length, contiguous)
        i = 0
        while i + 32 <= len(raw):
            t = raw[i]
            if t == 0:
                break
            if t == 0x85:
                n = raw[i + 1]
                attr = struct.unpack_from("<H", raw, i + 4)[0]
                st = raw[i + 32:i + 64]
                flags = st[1]
                namelen = st[3]
                size = struct.unpack_from("<Q", st, 24)[0]
                clus = struct.unpack_from("<I", st, 20)[0]
                name = b"".join(raw[i + 64 + 32 * k + 2:i + 64 + 32 * k + 32]
                                for k in range(n - 1))
                name = name.decode("utf-16-le")[:namelen]
                yield name, bool(attr & 0x10), clus, size, bool(flags & 2)
                i += 32 * (n + 1)
            else:
                i += 32

    def lookup(self, path):
        clus, size, contig, isdir = self.root, None, False, True
        for comp in [p for p in path.split("/") if p]:
            for name, d, c, s, nc in self.entries(clus, size, contig):
                if name == comp:
                    clus, size, contig, isdir = c, s, nc, d
                    break
            else:
                sys.exit("%s: not found" % path)
        return clus, size, contig, isdir


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    img, part = sys.argv[1], sys.argv[2]
    cmd = sys.argv[3] if len(sys.argv) > 3 else "ls"
    path = sys.argv[4] if len(sys.argv) > 4 else "/"
    with open(img, "rb") as f:
        parts = gpt_partitions(f)
        if part not in parts:
            sys.exit("partitions: %s" % ", ".join(parts))
        fs = ExFat(f, parts[part][0])
        clus, size, contig, isdir = fs.lookup(path)
        if cmd == "ls":
            for name, d, c, s, nc in fs.entries(clus, size, contig):
                print("%s %10d %s" % ("d" if d else "-", s, name))
        elif cmd in ("cat", "get"):
            data = fs.data(clus, size, contig)
            if cmd == "cat":
                sys.stdout.buffer.write(data)
            else:
                open(sys.argv[5], "wb").write(data)
        else:
            sys.exit(__doc__)


if __name__ == "__main__":
    main()
