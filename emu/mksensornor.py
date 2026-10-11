#!/usr/bin/env python3
"""Build the sensor hub NOR backing image for a best1700 device (q63).

The sensor hub core (BTHC0) runs vela_sensor.bin in place from its own
8 MiB NOR; the machine installs that firmware at the start of the image
on every start (-M sensor-nor=<file>), so the image carries what the
factory line programs besides it.

The sensor firmware's setup_block_device_driver (0x0C00D608) cuts two
partitions out of /dev/bes_flash (MTD blocks of 256 bytes):
  bth_misc     blocks 0x7800 + 0x400  = 0x780000, 256 KiB, littlefs,
               mounted at /misc without autoformat ("mount misc fail")
  bth_factory  blocks 0x7C00 + 0x10   = 0x7C0000, 4 KiB
The AP mounts the sensor's /misc over rpmsgfs (mount -t rpmsgfs -o
cpu=sensor,fs=/misc /misc) and its property service loads
/misc/factory.prop (then /nv/app_nv.prop and /etc/build.prop) into kvdb,
as on p67 (see emu/mknor.py for the properties the Bluetooth bind path
reads).

littlefs geometry: block 4096 bytes x 64 (the NOR's erase sector), read /
program 256, name_max 32, disk version 2.0.

Requires littlefs-python (emu/venv).

usage: emu/mksensornor.py [--out FILE] [--did N] [--psn S] [--mac-bt ADDR]
                          [--prop KEY=VALUE ...]
"""

import argparse
import os
import re
import sys

try:
    import littlefs
except ImportError:
    sys.exit("emu/mksensornor.py needs littlefs-python (emu/venv)")

FLASH_SIZE = 0x800000
MISC_OFFSET = 0x7800 * 256
MISC_SIZE = 0x400 * 256
MISC_BLOCK = 4096
MISC_IO = 256
MISC_NAME_MAX = 32
LFS_DISK_VERSION = 0x00020000


def misc_partition(props, block):
    fs = littlefs.LittleFS(block_size=block, block_count=MISC_SIZE // block,
                           read_size=MISC_IO, prog_size=MISC_IO,
                           name_max=MISC_NAME_MAX,
                           disk_version=LFS_DISK_VERSION)
    text = "".join("%s=%s\n" % kv for kv in props)
    for name in ("/factory.prop", "/factory.prop.bkup"):
        with fs.open(name, "w") as f:
            f.write(text)
    return bytes(fs.context.buffer)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default="emu/sensor-nor-q63.img")
    ap.add_argument("--did", default="1700000001",
                    help="ro.factory.did (decimal)")
    ap.add_argument("--psn", default="QEMU17000000000001",
                    help="ro.factory.psn (serial number)")
    ap.add_argument("--mac-bt", default="C0:FF:EE:15:03:01",
                    help="ro.factory.mac_bt (match the bes.btss bdaddr)")
    ap.add_argument("--prop", action="append", default=[],
                    metavar="KEY=VALUE", help="additional factory property")
    ap.add_argument("--block", type=int, default=MISC_BLOCK,
                    help="littlefs block size")
    args = ap.parse_args()

    if not args.did.isdigit():
        sys.exit("--did must be decimal")
    if not 0 < len(args.psn) < 32:
        sys.exit("--psn must have 1..31 characters")
    if not re.fullmatch(r"([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}", args.mac_bt):
        sys.exit("--mac-bt must look like XX:XX:XX:XX:XX:XX")
    props = [("ro.factory.did", args.did), ("ro.factory.psn", args.psn),
             ("ro.factory.mac_bt", args.mac_bt.upper())]
    for kv in args.prop:
        key, sep, value = kv.partition("=")
        if not sep or not key:
            sys.exit("--prop expects KEY=VALUE, got %r" % kv)
        props.append((key, value))

    image = bytearray(b"\xff" * FLASH_SIZE)
    misc = misc_partition(props, args.block)
    image[MISC_OFFSET:MISC_OFFSET + len(misc)] = misc
    tmp = args.out + ".tmp"
    with open(tmp, "wb") as out:
        out.write(image)
    os.replace(tmp, args.out)
    print("%s: %d MiB sensor NOR, /misc/factory.prop at 0x%x: %s" %
          (args.out, FLASH_SIZE >> 20, MISC_OFFSET,
           ", ".join(k for k, _ in props)))


if __name__ == "__main__":
    main()
