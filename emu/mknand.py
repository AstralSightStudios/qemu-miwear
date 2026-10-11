#!/usr/bin/env python3
"""Build the SPI NAND backing image for the best1503 machine.

The image is page-major: every NAND page is stored as <page> data bytes
followed by <oob> spare bytes, erased state 0xFF. This is the format the
bes.flash NAND model (nand-image property) expects.

Factory content: the resource romfs (vela_resource.bin, normally written
by OTA with `dd of=/dev/resource`) is placed at the start of the
"resource" partition. Partition layout from the firmware's
bes_board_nand_initialize, 512MB table:
  p67 (Band 10 Pro, sub_C20AB44, table 0x2CCC72C0): resource first=0
      nblocks=0x19000 pages, nand_data/coredump/nand_nv after
  q66 (Band 11, boot log): resource first=0 nblocks=65536 pages,
      nand_data first=65536 nblocks=196608

usage: emu/mknand.py [--profile p67|q66] [--resource FILE] [--out FILE]
                     [--first-page N]
"""

import argparse
import os
import sys

PAGE = 2048
OOB = 64
PPB = 64
BLOCKS = 4096
RESOURCE_FIRST = 0
RESOURCE_PAGES = {"p67": 0x19000, "q66": 65536}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--profile", choices=sorted(RESOURCE_PAGES),
                    default="p67", help="board the image is for")
    ap.add_argument("--resource", help="romfs image for /dev/resource")
    ap.add_argument("--out", default="emu/nand.img")
    ap.add_argument("--first-page", type=lambda x: int(x, 0),
                    default=RESOURCE_FIRST,
                    help="first page of the resource partition")
    args = ap.parse_args()

    res = b""
    if args.resource:
        with open(args.resource, "rb") as f:
            res = f.read()
        pages = RESOURCE_PAGES[args.profile]
        if len(res) > pages * PAGE:
            sys.exit("resource image (%d bytes) does not fit the %d-page "
                     "resource partition" % (len(res), pages))

    oob = b"\xff" * OOB
    erased_block = (b"\xff" * (PAGE + OOB)) * PPB
    res_pages = (len(res) + PAGE - 1) // PAGE
    tmp = args.out + ".tmp"
    with open(tmp, "wb") as out:
        for blk in range(BLOCKS):
            first = blk * PPB
            if first + PPB <= args.first_page or \
               first >= args.first_page + res_pages:
                out.write(erased_block)
                continue
            parts = []
            for row in range(first, first + PPB):
                p = row - args.first_page
                if 0 <= p < res_pages:
                    data = res[p * PAGE:(p + 1) * PAGE]
                    parts.append(data.ljust(PAGE, b"\xff"))
                else:
                    parts.append(b"\xff" * PAGE)
                parts.append(oob)
            out.write(b"".join(parts))
    os.replace(tmp, args.out)
    print("%s: %d MiB NAND, resource %d bytes at page %d" %
          (args.out, BLOCKS * PPB * PAGE >> 20, len(res), args.first_page))


if __name__ == "__main__":
    main()
