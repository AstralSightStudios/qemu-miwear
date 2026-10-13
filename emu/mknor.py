#!/usr/bin/env python3
"""Build the NOR flash backing image for the best1503 machine.

The image is the logical 16 MiB view of the dual-chip NOR (the bes.flash
"image" property, -M best1503,flash-image=<file>), erased state 0xFF. The
machine installs the AP firmware given with -kernel at OTA_CODE_OFFSET on
every start, so the image carries what a factory line programs besides
the firmware.

Factory content: the "nv" partition, mounted by the firmware's rcS as
`mount -t littlefs -o autoformat /dev/nv /misc`, holds /factory.prop
(and its /factory.prop.bkup copy), the key=value file the factory line
writes and the property service loads into kvdb at boot (sub_C6FFC58,
backup restore sub_C6B79A0). Properties the Bluetooth bind path reads:
  ro.factory.did     decimal device ID; the bind state machine reports it
                     in BindInfo and fails without it (get_did_ sub_C702AD8,
                     auth_event_handle event 37 sub_C707AC4)
  ro.factory.psn     product serial number, at most 31 characters
                     (miwear_device_serial_number_get sub_C702744)
  ro.factory.mac_bt  "XX:XX:XX:XX:XX:XX", the controller address
                     (sub_C23454C); keep it equal to the bes.btss "bdaddr"
                     property (default C0:FF:EE:15:03:01)
The defaults identify an emulated unit; they are not any real band's.

Geometry: the /misc partition (bes_partition_init boot log, 512-byte MTD
blocks) is 128 KiB, littlefs disk version 2.1, block 8 KiB x 16,
read/prog 512, name_max 32, as found in the superblock each firmware
itself writes:
  p67 (Band 10 Pro): "nv" first=32512, offset 0xFE0000, mounted
      `-o autoformat /dev/nv /misc`
  q66 (Band 11): "facmode" first=32256, offset 0xFC0000, mounted
      `-o ro /dev/facmode /misc` (its "nv" at 0xFE0000 is /nv, which
      the firmware formats itself)

p65 (REDMI Watch 6) has no /misc: its AP builds the ro.factory.*
properties at boot (sub_C630050) from a 256-byte record at NOR 0xCC0000
(read through 0x28CC0000) that the factory line programs:
   8..13  Bluetooth address, most significant byte first ("xt_test_log
          mac", sub_C963724; also the controller address)
  14..21  DID, big-endian 64-bit (ro.factory.did in decimal, did_cqr in hex)
  22..37  key (16 characters)          39..58  PSN (NUL padded)
  63..76  FSN                          77..85  SKU
  168..   sensor / motor / audio calibration (left 0)
--prop has no place there and is refused for p65.

Requires littlefs-python (pip install littlefs-python) for p67 / q66.

usage: emu/mknor.py [--profile p67|q66|p65] [--out FILE] [--did N]
                    [--psn S] [--mac-bt ADDR] [--prop KEY=VALUE ...]
"""

import argparse
import os
import re
import sys

import struct

FLASH_SIZE = 0x1000000
MISC_OFFSET = {"p67": 32512 * 512, "q66": 32256 * 512}
P65_FACTORY_OFFSET = 0xCC0000
NV_BLOCK = 8192
NV_BLOCKS = 16
NV_IO = 512
NV_NAME_MAX = 32
LFS_DISK_VERSION = 0x00020001


def nv_partition(props):
    try:
        import littlefs
    except ImportError:
        sys.exit("emu/mknor.py needs littlefs-python: "
                 "pip install littlefs-python")
    fs = littlefs.LittleFS(block_size=NV_BLOCK, block_count=NV_BLOCKS,
                           read_size=NV_IO, prog_size=NV_IO,
                           name_max=NV_NAME_MAX,
                           disk_version=LFS_DISK_VERSION)
    text = "".join("%s=%s\n" % kv for kv in props)
    for name in ("/factory.prop", "/factory.prop.bkup"):
        with fs.open(name, "w") as f:
            f.write(text)
    return bytes(fs.context.buffer)


def p65_factory_record(args):
    rec = bytearray(256)
    rec[8:14] = bytes.fromhex(args.mac_bt.replace(":", ""))
    rec[14:22] = struct.pack(">Q", int(args.did))
    rec[22:38] = args.key.encode().ljust(16, b"0")[:16]
    rec[39:59] = args.psn.encode()[:20].ljust(20, b"\0")
    rec[63:77] = args.fsn.encode()[:14].ljust(14, b"\0")
    rec[77:86] = args.sku.encode()[:9].ljust(9, b"\0")
    return bytes(rec)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--profile", choices=sorted(MISC_OFFSET) + ["p65"],
                    default="p67", help="board the image is for")
    ap.add_argument("--out", default="emu/nor.img")
    ap.add_argument("--did", default="1503000001",
                    help="ro.factory.did (decimal)")
    ap.add_argument("--psn", default="QEMU15030000000001",
                    help="ro.factory.psn (serial number)")
    ap.add_argument("--mac-bt", default="C0:FF:EE:15:03:01",
                    help="ro.factory.mac_bt (match the bes.btss bdaddr)")
    ap.add_argument("--key", default="0123456789ABCDEF",
                    help="ro.factory.key (p65, 16 characters)")
    ap.add_argument("--fsn", default="QEMU1502P00001",
                    help="ro.factory.fsn (p65)")
    ap.add_argument("--sku", default="M2509W1",
                    help="ro.factory.sku (p65)")
    ap.add_argument("--prop", action="append", default=[],
                    metavar="KEY=VALUE", help="additional factory property")
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
    if args.profile == "p65":
        if args.prop:
            sys.exit("--prop: p65's factory record has fixed fields only")
        offset = P65_FACTORY_OFFSET
        image[offset:offset + 256] = p65_factory_record(args)
    else:
        offset = MISC_OFFSET[args.profile]
        nv = nv_partition(props)
        image[offset:offset + len(nv)] = nv
    tmp = args.out + ".tmp"
    with open(tmp, "wb") as out:
        out.write(image)
    os.replace(tmp, args.out)
    print("%s: %d MiB NOR, %s at 0x%x: %s" %
          (args.out, FLASH_SIZE >> 20,
           "factory record" if args.profile == "p65" else
           "/misc/factory.prop", offset, ", ".join(k for k, _ in props)))


if __name__ == "__main__":
    main()
