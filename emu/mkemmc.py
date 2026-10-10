#!/usr/bin/env python3
"""Build the eMMC backing image for a best1700 device (q63, Xiaomi Watch S5).

The AP firmware probes /dev/mmcsd0 on the SDHCI host and parses a GUID
partition table on it ("EFI PART", fs/partition/fs_partition.c); each GPT
entry becomes /dev/<name>, which rc.sysinit mounts:
  romfs  vendor system app health misc i18n font watchface quickapp
         cp_image audio_image
  fatfs  nv data userlog  (mount -o autoformat: the firmware formats them)
The OTA script (ota.sh in the firmware package) also writes sensor and
store; ap and tee live in the NOR flash, not here.

Every romfs partition gets the matching vela_<name>.bin from the firmware
directory; the fatfs ones are left zeroed. The image is sparse.

usage: emu/mkemmc.py <firmware dir> [--out FILE] [--size GiB]
                     [--data MiB] [--nv MiB] [--userlog MiB]
"""

import argparse
import binascii
import os
import struct
import sys
import uuid

SECTOR = 512
ALIGN = 1 << 20
ENTRIES = 128
ENTRY_SIZE = 128
BASIC_DATA = uuid.UUID("ebd0a0a2-b9e5-4433-87c0-68b6b72699c7")
NAMESPACE = uuid.UUID("6f8a3c8e-1b70-4d2e-9a5e-0e3b7d1c1700")

IMAGES = ["vendor", "system", "app", "health", "misc", "i18n", "font",
          "watchface", "quickapp", "cp_image", "audio_image", "sensor",
          "store"]


def align(n, a=ALIGN):
    return (n + a - 1) // a * a


def entry(name, first, last):
    return struct.pack("<16s16sQQQ72s", BASIC_DATA.bytes_le,
                       uuid.uuid5(NAMESPACE, name).bytes_le, first, last, 0,
                       name.encode("utf-16-le"))


def header(my, alt, first, last, entries_lba, entries_crc, disk_guid):
    h = struct.pack("<8sIIIIQQQQ16sQIII", b"EFI PART", 0x00010000, 92, 0, 0,
                    my, alt, first, last, disk_guid.bytes_le, entries_lba,
                    ENTRIES, ENTRY_SIZE, entries_crc)
    crc = binascii.crc32(h) & 0xffffffff
    h = h[:16] + struct.pack("<I", crc) + h[20:]
    return h.ljust(SECTOR, b"\0")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("fwdir")
    ap.add_argument("--out", default="emu/emmc-q63.img")
    ap.add_argument("--size", type=int, default=4, help="GiB")
    ap.add_argument("--data", type=int, default=1024, help="MiB")
    ap.add_argument("--nv", type=int, default=32, help="MiB")
    ap.add_argument("--userlog", type=int, default=64, help="MiB")
    args = ap.parse_args()

    size = args.size << 30
    nsect = size // SECTOR
    first_usable = 34
    last_usable = nsect - 34

    parts = []      # (name, offset, size, source file or None)
    off = ALIGN
    for name in IMAGES:
        src = os.path.join(args.fwdir, "vela_%s.bin" % name)
        if not os.path.exists(src):
            sys.exit("missing %s" % src)
        n = align(os.path.getsize(src))
        parts.append((name, off, n, src))
        off += n
    for name, mib in (("nv", args.nv), ("data", args.data),
                      ("userlog", args.userlog)):
        parts.append((name, off, mib << 20, None))
        off += mib << 20
    if off > last_usable * SECTOR:
        sys.exit("partitions need %d MiB, more than --size" % (off >> 20))

    table = b"".join(entry(name, o // SECTOR, (o + n) // SECTOR - 1)
                     for name, o, n, _ in parts)
    table = table.ljust(ENTRIES * ENTRY_SIZE, b"\0")
    tcrc = binascii.crc32(table) & 0xffffffff
    disk = uuid.uuid5(NAMESPACE, "disk")

    with open(args.out, "wb") as f:
        f.truncate(size)
        # protective MBR
        mbr = bytearray(SECTOR)
        mbr[446:462] = struct.pack("<BBBBBBBBII", 0, 0, 2, 0, 0xEE, 0xFF,
                                   0xFF, 0xFF, 1, min(nsect - 1, 0xffffffff))
        mbr[510:512] = b"\x55\xaa"
        f.seek(0)
        f.write(mbr)
        f.seek(SECTOR)
        f.write(header(1, nsect - 1, first_usable, last_usable, 2, tcrc, disk))
        f.write(table)
        f.seek((nsect - 33) * SECTOR)
        f.write(table)
        f.write(header(nsect - 1, 1, first_usable, last_usable, nsect - 33,
                       tcrc, disk))
        for name, o, n, src in parts:
            if src:
                with open(src, "rb") as s:
                    f.seek(o)
                    while True:
                        buf = s.read(1 << 20)
                        if not buf:
                            break
                        f.write(buf)
    for name, o, n, src in parts:
        print("%-12s 0x%09x %6d MiB %s" % (name, o, n >> 20,
                                           os.path.basename(src) if src
                                           else "(fatfs, autoformat)"))
    print("wrote %s (%d GiB)" % (args.out, args.size))


if __name__ == "__main__":
    main()
