#!/usr/bin/env python3
"""Build the SPI NAND backing image for the best1503 machine.

The image is page-major: every NAND page is stored as <page> data bytes
followed by <oob> spare bytes, erased state 0xFF. This is the format the
bes.flash NAND model (nand-image property) expects.

Factory content depends on the board (bes_board_nand_initialize partition
tables, 512MB part, page units):
  p67 (Band 10 Pro, sub_C20AB44, table 0x2CCC72C0): "resource" first=0
      nblocks=0x19000 holds the resource romfs (vela_resource.bin, written
      by OTA with `dd of=/dev/resource`); nand_data/coredump/nand_nv after
  p65 (REDMI Watch 6, best1502p, partition table record "resource"):
      "resource" first=0 nblocks=112640, nand_data first=112640;
  q65 (REDMI Watch 7, best1502p, partition table record "resource"):
      "resource" first=0 nblocks=120320, nand_data after;
  q66 (Band 11, boot log): "resource" first=0 nblocks=65536, romfs as p67;
      nand_data first=65536 nblocks=196608
  n66 (Band 9) / o66 (Band 10), boot log: "nand_system" first=0
      nblocks=65536, nand_data after. The firmware mounts it with
      `mount -t yaffs -o ro /dev/nand_system /system` and loop-mounts the
      package's romfs files from it (/system/app.bin on /resource/app,
      font.bin on /font, ...), so it gets a YAFFS2 file system holding the
      package's resource files (ota.json "main_res" sections).

YAFFS2 as the firmware configures it (yaffs_vfs bind, n66 sub_C39C7F4):
2048-byte chunks with inband tags, 64 chunks per block, start block 0
(internally block 1). The last 16 bytes of every chunk are the packed
tags (seq_number, obj_id, chunk_id, n_bytes), so a chunk carries 2032
data bytes; the spare area is not used. Object headers are laid out as
the firmware writes them on /data (checked against an image it wrote):
type, parent, name at 10, mode at 268, times, file size at 292, unused
fields 0xFF. Header tags carry the extra info (chunk_id = 0x80000000 |
parent, obj_id | type << 28, n_bytes = file size). Blocks get rising
sequence numbers from 0x1001, objects ids from 257.

usage: emu/mknand.py [--profile p67|q66|n66|o66] [--resource FILE]
                     [--system DIR] [--out FILE] [--first-page N]
"""

import argparse
import json
import os
import struct
import sys

PAGE = 2048
OOB = 64
PPB = 64
BLOCKS = 4096

# board: (kind, first page, pages)
PARTITIONS = {
    "p67": ("resource", 0, 0x19000),
    "q66": ("resource", 0, 65536),
    "n66": ("system", 0, 65536),
    "o66": ("system", 0, 65536),
    "p65": ("resource", 0, 112640),
    "q65": ("resource", 0, 120320),
}

YAFFS_TAGS = 16
YAFFS_DATA = PAGE - YAFFS_TAGS
YAFFS_FIRST_SEQ = 0x1001
YAFFS_FIRST_OBJ = 257
YAFFS_ROOT = 1
YAFFS_TYPE_FILE = 1
YAFFS_EXTRA_HEADER = 0x80000000
YAFFS_TYPE_SHIFT = 28
YAFFS_FILE_MODE = 0o644
YAFFS_TIME = 946684800          # 2000-01-01, the files' times


def system_files(fwdir):
    """the package's resource files, in ota.json order"""
    names = []
    try:
        with open(os.path.join(fwdir, "ota.json")) as f:
            for sec in json.load(f)["sections"]:
                if sec.get("section_type") == "main_res":
                    names.append(sec["location_path"])
    except FileNotFoundError:
        names = sorted(n for n in os.listdir(fwdir)
                       if (n.endswith(".bin") and not n.startswith("vela_"))
                       or n == "version")
    files = []
    for n in names:
        p = os.path.join(fwdir, n)
        if os.path.isfile(p):
            files.append((n, p))
    if not files:
        sys.exit("%s: no resource files" % fwdir)
    return files


def yaffs_header(parent, name, size):
    h = bytearray(b"\xff" * YAFFS_DATA)
    raw = name.encode()
    if len(raw) > 255:
        sys.exit("file name too long: %s" % name)
    struct.pack_into("<II", h, 0, YAFFS_TYPE_FILE, parent)
    h[10:266] = raw.ljust(256, b"\0")
    struct.pack_into("<6I", h, 268, YAFFS_FILE_MODE, 0, 0,
                     YAFFS_TIME, YAFFS_TIME, YAFFS_TIME)
    struct.pack_into("<II", h, 292, size & 0xFFFFFFFF, 0xFFFFFFFF)
    struct.pack_into("<7I", h, 460, 0, YAFFS_TIME, 0, YAFFS_TIME, 0,
                     YAFFS_TIME, 0)
    struct.pack_into("<I", h, 488, 0)               # inband_shadowed_obj_id
    struct.pack_into("<I", h, 496, size >> 32)      # file_size_high
    struct.pack_into("<II", h, 504, 0, 0)           # shadows_obj, is_shrink
    return bytes(h)


def yaffs_chunks(files):
    """(obj_id, chunk_id, n_bytes, data) for every chunk, in write order"""
    for i, (name, path) in enumerate(files):
        oid = YAFFS_FIRST_OBJ + i
        size = os.path.getsize(path)
        yield (oid | YAFFS_TYPE_FILE << YAFFS_TYPE_SHIFT,
               YAFFS_EXTRA_HEADER | YAFFS_ROOT, size & 0xFFFFFFFF,
               yaffs_header(YAFFS_ROOT, name, size))
        with open(path, "rb") as f:
            cid = 1
            while True:
                data = f.read(YAFFS_DATA)
                if not data:
                    break
                yield oid, cid, len(data), data
                cid += 1


def yaffs_pages(files):
    """the partition's pages, one per chunk, the block sequence in tags"""
    for n, (oid, cid, nbytes, data) in enumerate(yaffs_chunks(files)):
        seq = YAFFS_FIRST_SEQ + n // PPB
        yield (data.ljust(YAFFS_DATA, b"\xff") +
               struct.pack("<IIII", seq, oid, cid, nbytes))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--profile", choices=sorted(PARTITIONS),
                    default="p67", help="board the image is for")
    ap.add_argument("--resource", help="romfs image for /dev/resource "
                    "(p67, q66)")
    ap.add_argument("--system", metavar="DIR", help="firmware package "
                    "directory whose resource files go to /system (n66, o66)")
    ap.add_argument("--out", default="emu/nand.img")
    ap.add_argument("--first-page", type=lambda x: int(x, 0),
                    help="first page of the partition")
    args = ap.parse_args()

    kind, first_page, pages = PARTITIONS[args.profile]
    if args.first_page is not None:
        first_page = args.first_page
    if kind == "resource" and args.system:
        sys.exit("--system is for n66/o66; %s takes --resource" % args.profile)
    if kind == "system" and args.resource:
        sys.exit("--resource is for p67/q66; %s takes --system" % args.profile)

    content = iter(())
    what = "empty"
    if args.resource:
        with open(args.resource, "rb") as f:
            res = f.read()
        if len(res) > pages * PAGE:
            sys.exit("resource image (%d bytes) does not fit the %d-page "
                     "resource partition" % (len(res), pages))
        content = (res[p * PAGE:(p + 1) * PAGE].ljust(PAGE, b"\xff")
                   for p in range((len(res) + PAGE - 1) // PAGE))
        what = "resource %d bytes" % len(res)
    elif args.system:
        files = system_files(args.system)
        need = sum(1 + (os.path.getsize(p) + YAFFS_DATA - 1) // YAFFS_DATA
                   for _, p in files)
        if need > pages:
            sys.exit("/system needs %d chunks, the partition has %d" %
                     (need, pages))
        content = yaffs_pages(files)
        what = "/system yaffs2 %d files, %d chunks (%s)" % (
            len(files), need, ", ".join(n for n, _ in files))

    oob = b"\xff" * OOB
    erased_page = b"\xff" * PAGE + oob
    tmp = args.out + ".tmp"
    with open(tmp, "wb") as out:
        page = 0
        out.write(erased_page * first_page)
        page = first_page
        for data in content:
            out.write(data)
            out.write(oob)
            page += 1
        while page < BLOCKS * PPB:
            n = min(BLOCKS * PPB - page, 4096)
            out.write(erased_page * n)
            page += n
    os.replace(tmp, args.out)
    print("%s: %d MiB NAND, %s at page %d" %
          (args.out, BLOCKS * PPB * PAGE >> 20, what, first_page))


if __name__ == "__main__":
    main()
