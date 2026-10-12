#!/usr/bin/env python3
"""IDA 9.4 loader for Xiaomi Vela AP images on BES chips (best1503, best1700).

Generalised from best1503_vela.py; the chip is the CHIP= line of the image's
build-info block. Differences handled for best1700 (q63, Cortex-M55):
  - the cached flash alias can coincide with the execute alias
    (FLASH_BASE 0x10000000 + OTA 0x1C0000 = XIP 0x101C0000): mapped once
  - the startup literals may describe an empty .bss
  - the processor is ARMv8.1-M (Helium/MVE) instead of ARMv8-M

A Vela AP image is not linked against one flat address. The same flash bytes
are visible through three CPU aliases, with the OTA slot offset added to each:

    execute/XIP alias     0x0C000000 + OTA_CODE_OFFSET
    cached flash alias    FLASH_BASE + OTA_CODE_OFFSET
    non-cached alias      FLASH_NC_BASE + OTA_CODE_OFFSET

For the observed BEST1503 image those become 0x0C0C0000, 0x2C0C0000, and
0x280C0000. Branch targets use the execute alias, ordinary rodata references
mostly use the cached alias, and boot metadata identifies the non-cached alias.
The loader maps the file at all three addresses, but only the execute alias is
marked executable so IDA does not discover three copies of every function.

The 16-byte image header is followed by Thumb startup code. Its fourth word is
a non-cached pointer to the boot/build-info block near the end of the image,
not an image checksum. The last word of the file gives the mapped non-cached
image base. The build-info text supplies FLASH_BASE, FLASH_NC_BASE, and
OTA_CODE_OFFSET, allowing the aliases to be derived without hard-coding p67.

The startup literal pool also describes the initialized-data copy, BSS range,
and main-stack bounds. When those values validate, the loader creates DATA,
BSS, and MSP_STACK segments and copies the initialized data bytes to their RAM
VMA. Images without validated footer metadata retain the conservative legacy
single-XIP mapping.
"""

import struct

import ida_bytes
import ida_entry
import ida_ida
import ida_idp
import ida_loader
import ida_name
import ida_nalt
import ida_segment
import ida_segregs


FORMAT_NAME = "BES Vela image"
PROCESSOR_NAME = "ARM:ARMv8-M"
# chips whose AP core is a Cortex-M55 (ARMv8.1-M with Helium)
PROCESSOR_BY_CHIP = {"best1700": ("ARM:ARMv8.1-M", "ARM:ARMv8-M")}

HEADER_SIZE = 0x10
ENTRY_STUB_OFFSET = 0x10
MAPPED_ENTRY_OFFSET = 0x14
MAPPED_ENTRY_LITERAL_OFFSET = 0x64

MSP_LITERAL_OFFSET = 0x68
MSPLIM_LITERAL_OFFSET = 0x6C
DATA_LMA_LITERAL_OFFSET = 0x70
DATA_START_LITERAL_OFFSET = 0x74
DATA_END_LITERAL_OFFSET = 0x78
BSS_START_LITERAL_OFFSET = 0x7C
BSS_END_LITERAL_OFFSET = 0x80

BOOT_TRAILER_SIZE = 8

# Common prefix seen in both p67 AP and BL2 images.
HEADER_MAGIC0 = 0xFFFFFFFF
# best1502p packages (REDMI Watch 6) carry the boot-struct magic instead
HEADER_MAGICS = (HEADER_MAGIC0, 0xBE57EC1C)
HEADER_VERSION = 0x00050000
HEADER_RESERVED = 0x00000000
ENTRY_STUB = b"\x14\x48\x00\x47"  # LDR R0,[PC,#0x50]; BX R0

MIN_IMAGE_SIZE = BSS_END_LITERAL_OFFSET + 4
MAX_REASONABLE_IMAGE_SIZE = 0x20000000


class ImageLayout:
    def __init__(self, size, xip_base, mapped_entry, boot_info_ptr):
        self.size = size
        self.xip_base = xip_base
        self.mapped_entry = mapped_entry
        self.entry_stub = xip_base + ENTRY_STUB_OFFSET
        self.boot_info_ptr = boot_info_ptr

        self.ota_offset = None
        self.flash_base = None
        self.flash_nc_base = None
        self.cached_base = None
        self.nc_base = None
        self.boot_info_offset = None
        self.boot_text_end_offset = None
        self.boot_config = {}

        self.msp = None
        self.msplim = None
        self.data_lma = None
        self.data_lma_offset = None
        self.data_start = None
        self.data_end = None
        self.bss_start = None
        self.bss_end = None

    @property
    def image_end(self):
        return self.xip_base + self.size

    @property
    def has_aliases(self):
        return self.cached_base is not None and self.nc_base is not None

    @property
    def has_ram_layout(self):
        return self.data_lma_offset is not None


def _read_exact(li, offset, size):
    li.seek(offset)
    data = li.read(size)
    return data if data is not None and len(data) == size else None


def _read_u32le(li, offset):
    data = _read_exact(li, offset, 4)
    return None if data is None else struct.unpack("<I", data)[0]


def _checked_add_u32(left, right):
    value = left + right
    return value if 0 <= value <= 0xFFFFFFFF else None


def _parse_boot_config(data):
    """Extract simple KEY=value lines from the NUL-terminated build-info text."""
    text = data.split(b"\x00", 1)[0].decode("ascii", errors="ignore")
    config = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if key:
            config[key] = value
    return config


def _parse_hex_config(config, key):
    value = config.get(key)
    if value is None:
        return None
    try:
        return int(value, 0)
    except ValueError:
        return None


def _probe_alias_layout(li, layout, header_boot_info_ptr):
    """Validate the footer and populate cached/non-cached flash aliases."""
    if layout.size < BOOT_TRAILER_SIZE:
        return

    mapped_nc_base = _read_u32le(li, layout.size - 4)
    if mapped_nc_base is None or header_boot_info_ptr < mapped_nc_base:
        return

    boot_info_offset = header_boot_info_ptr - mapped_nc_base
    if not (HEADER_SIZE <= boot_info_offset < layout.size - BOOT_TRAILER_SIZE):
        return

    boot_blob = _read_exact(li, boot_info_offset, layout.size - boot_info_offset)
    if boot_blob is None:
        return
    nul_offset = boot_blob.find(b"\x00")
    if nul_offset < 0:
        return
    config = _parse_boot_config(boot_blob)

    flash_base = _parse_hex_config(config, "FLASH_BASE")
    flash_nc_base = _parse_hex_config(config, "FLASH_NC_BASE")
    ota_offset = _parse_hex_config(config, "OTA_CODE_OFFSET")
    if flash_base is None or flash_nc_base is None or ota_offset is None:
        return

    cached_base = _checked_add_u32(flash_base, ota_offset)
    nc_base = _checked_add_u32(flash_nc_base, ota_offset)
    if cached_base is None or nc_base is None or nc_base != mapped_nc_base:
        return
    if header_boot_info_ptr != nc_base + boot_info_offset:
        return

    for base in (cached_base, nc_base):
        end = _checked_add_u32(base, layout.size)
        if end is None or end <= base:
            return

    layout.ota_offset = ota_offset
    layout.flash_base = flash_base
    layout.flash_nc_base = flash_nc_base
    layout.cached_base = cached_base
    layout.nc_base = nc_base
    layout.boot_info_offset = boot_info_offset
    layout.boot_text_end_offset = boot_info_offset + nul_offset + 1
    layout.boot_config = config


def _probe_ram_layout(li, layout):
    """Validate startup literals describing .data, .bss, and the main stack."""
    if not layout.has_aliases:
        return

    values = [
        _read_u32le(li, offset)
        for offset in (
            MSP_LITERAL_OFFSET,
            MSPLIM_LITERAL_OFFSET,
            DATA_LMA_LITERAL_OFFSET,
            DATA_START_LITERAL_OFFSET,
            DATA_END_LITERAL_OFFSET,
            BSS_START_LITERAL_OFFSET,
            BSS_END_LITERAL_OFFSET,
        )
    ]
    if any(value is None for value in values):
        return

    msp, msplim, data_lma, data_start, data_end, bss_start, bss_end = values
    data_lma_offset = data_lma - layout.cached_base
    data_size = data_end - data_start

    if not (
        msplim < msp
        and data_start < data_end
        and data_end == bss_start
        and bss_start <= bss_end
        and 0 <= data_lma_offset < layout.size
        and data_size > 0
        and data_lma_offset + data_size <= layout.size
    ):
        return

    # In validated p67 images the initialized-data image ends exactly where the
    # build-info block begins. Do not require that relationship for other BES
    # variants, but reject a copy range that would consume the footer.
    if (
        layout.boot_info_offset is not None
        and data_lma_offset + data_size > layout.boot_info_offset
    ):
        return

    layout.msp = msp
    layout.msplim = msplim
    layout.data_lma = data_lma
    layout.data_lma_offset = data_lma_offset
    layout.data_start = data_start
    layout.data_end = data_end
    layout.bss_start = bss_start
    layout.bss_end = bss_end


def _probe(li):
    """Return parsed image metadata, or None when the format does not match."""
    size = li.size()
    if size < MIN_IMAGE_SIZE or size > MAX_REASONABLE_IMAGE_SIZE:
        return None

    header = _read_exact(li, 0, HEADER_SIZE)
    stub = _read_exact(li, ENTRY_STUB_OFFSET, len(ENTRY_STUB))
    mapped_ptr = _read_u32le(li, MAPPED_ENTRY_LITERAL_OFFSET)
    if header is None or stub is None or mapped_ptr is None:
        return None

    magic0, version, reserved, boot_info_ptr = struct.unpack("<4I", header)
    if (
        magic0 not in HEADER_MAGICS
        or version != HEADER_VERSION
        or reserved != HEADER_RESERVED
        or stub != ENTRY_STUB
        or (mapped_ptr & 1) == 0
    ):
        return None

    mapped_entry = mapped_ptr & ~1
    if mapped_entry < MAPPED_ENTRY_OFFSET:
        return None
    xip_base = mapped_entry - MAPPED_ENTRY_OFFSET

    # The execute alias plus OTA slot offset is naturally aligned on observed
    # BES images. This also prevents arbitrary Thumb data matching the loader.
    if (xip_base & 0xFFFF) != 0:
        return None
    image_end = _checked_add_u32(xip_base, size)
    if image_end is None or image_end <= xip_base:
        return None

    layout = ImageLayout(size, xip_base, mapped_entry, boot_info_ptr)
    _probe_alias_layout(li, layout, boot_info_ptr)
    _probe_ram_layout(li, layout)
    return layout


def accept_file(li, filename):
    """IDA loader format probe (IDA 9.4 API)."""
    try:
        image = _probe(li)
        if image is None:
            return 0

        if image.has_aliases:
            detail = "%s, XIP 0x%08X, cached 0x%08X, NC 0x%08X" % (
                image.boot_config.get("CHIP", "unknown chip"),
                image.xip_base,
                image.cached_base,
                image.nc_base,
            )
        else:
            detail = "XIP base 0x%08X" % image.xip_base
        return {
            "format": "%s (%s)" % (FORMAT_NAME, detail),
            "processor": PROCESSOR_NAME,
        }
    finally:
        li.seek(0)


def _map_file(li, file_offset, start, end):
    if end <= start:
        return
    if not li.file2base(file_offset, start, end, ida_loader.FILEREG_PATCHABLE):
        raise RuntimeError(
            "Unable to map file offset 0x%X at [0x%X, 0x%X)"
            % (file_offset, start, end)
        )


def _add_segment(start, end, name, sclass, permissions):
    if end <= start:
        return
    if not ida_segment.add_segm(0, start, end, name, sclass):
        raise RuntimeError(
            "Unable to create segment %s [0x%X, 0x%X)" % (name, start, end)
        )

    segment = ida_segment.getseg(start)
    if segment is None:
        raise RuntimeError("Unable to retrieve segment %s" % name)

    segment.bitness = 1  # 32-bit addressing
    segment.perm = permissions
    if not ida_segment.update_segm(segment):
        raise RuntimeError("Unable to update segment %s" % name)


def _set_name(ea, name):
    ida_name.set_name(ea, name, ida_name.SN_NOCHECK | ida_name.SN_PUBLIC)


def _set_comment(ea, text):
    ida_bytes.set_cmt(ea, text, False)


def _map_xip_alias(li, image):
    base = image.xip_base
    boot_offset = image.boot_info_offset
    data_offset = image.data_lma_offset

    # Map the complete file first, then describe evidence-backed boundaries.
    _map_file(li, 0, base, base + image.size)
    _add_segment(
        base,
        base + HEADER_SIZE,
        "BES_HEADER",
        "DATA",
        ida_segment.SEGPERM_READ,
    )

    text_end_offset = data_offset or boot_offset or image.size
    _add_segment(
        base + HEADER_SIZE,
        base + text_end_offset,
        "XIP_TEXT_RO",
        "CODE",
        ida_segment.SEGPERM_READ | ida_segment.SEGPERM_EXEC,
    )
    if data_offset is not None:
        data_lma_end = boot_offset or image.size
        _add_segment(
            base + data_offset,
            base + data_lma_end,
            "XIP_DATA_LMA",
            "DATA",
            ida_segment.SEGPERM_READ,
        )
    if boot_offset is not None:
        _add_segment(
            base + boot_offset,
            base + image.size,
            "XIP_BOOTINFO",
            "DATA",
            ida_segment.SEGPERM_READ,
        )


def _map_flash_aliases(li, image):
    if not image.has_aliases:
        return

    # The cached alias can be the execute alias itself (best1700); the XIP
    # segments then already describe those addresses.
    if image.cached_base != image.xip_base:
        _map_file(li, 0, image.cached_base, image.cached_base + image.size)
        _add_segment(
            image.cached_base,
            image.cached_base + image.size,
            "FLASH_CACHED",
            "DATA",
            ida_segment.SEGPERM_READ,
        )
    _map_file(li, 0, image.nc_base, image.nc_base + image.size)

    boot_ea = image.nc_base + image.boot_info_offset
    _add_segment(
        image.nc_base,
        boot_ea,
        "FLASH_NC",
        "DATA",
        ida_segment.SEGPERM_READ,
    )
    _add_segment(
        boot_ea,
        image.nc_base + image.size,
        "BOOT_INFO",
        "DATA",
        ida_segment.SEGPERM_READ,
    )


def _map_ram(li, image):
    if not image.has_ram_layout:
        return

    _map_file(li, image.data_lma_offset, image.data_start, image.data_end)
    _add_segment(
        image.data_start,
        image.data_end,
        "DATA",
        "DATA",
        ida_segment.SEGPERM_READ | ida_segment.SEGPERM_WRITE,
    )
    if image.bss_end > image.bss_start:
        _add_segment(
            image.bss_start,
            image.bss_end,
            "BSS",
            "BSS",
            ida_segment.SEGPERM_READ | ida_segment.SEGPERM_WRITE,
        )
    _add_segment(
        image.msplim,
        image.msp,
        "MSP_STACK",
        "BSS",
        ida_segment.SEGPERM_READ | ida_segment.SEGPERM_WRITE,
    )


def _describe_header(image):
    header_fields = (
        (0x00, "bes_header_magic", "Observed BES image magic 0xFFFFFFFF"),
        (0x04, "bes_header_version", "Observed BES image/header version 0x00050000"),
        (0x08, "bes_header_reserved", "Observed reserved zero word"),
        (
            0x0C,
            "bes_boot_info_ptr_nc",
            "Non-cached flash pointer to the trailing boot/build-info block",
        ),
    )
    for offset, name, comment in header_fields:
        ea = image.xip_base + offset
        ida_bytes.create_dword(ea, 4)
        _set_name(ea, name)
        _set_comment(ea, comment)
    _set_name(image.xip_base, "bes_image_header")


def _describe_boot_info(image):
    if not image.has_aliases:
        return

    boot_ea = image.nc_base + image.boot_info_offset
    trailer_ea = image.nc_base + image.size - BOOT_TRAILER_SIZE
    config_bytes = image.boot_config

    # The observed block starts with a newline before CHIP=best1503.
    string_start = boot_ea + 1
    string_data_end = image.nc_base + image.boot_text_end_offset
    ida_bytes.create_strlit(string_start, string_data_end, ida_nalt.STRTYPE_C)
    _set_name(boot_ea, "bes_boot_info")
    _set_comment(
        boot_ea,
        "Boot/build metadata referenced by header word 3 through FLASH_NC alias",
    )

    checksum_ea = trailer_ea
    mapped_base_ea = trailer_ea + 4
    ida_bytes.create_dword(checksum_ea, 4)
    ida_bytes.create_dword(mapped_base_ea, 4)
    _set_name(checksum_ea, "bes_boot_info_integrity")
    _set_comment(checksum_ea, "Boot-info integrity word; algorithm not yet verified")
    _set_name(mapped_base_ea, "bes_mapped_nc_image_base")
    _set_comment(
        mapped_base_ea,
        "FLASH_NC_BASE + OTA_CODE_OFFSET; used to recover boot-info file offset",
    )

    summary = ", ".join(
        "%s=%s" % (key, config_bytes[key])
        for key in ("FLASH_BASE", "FLASH_NC_BASE", "OTA_CODE_OFFSET")
        if key in config_bytes
    )
    if summary:
        _set_comment(string_start, summary)


def _describe_startup(image):
    _set_name(image.entry_stub, "_start")
    _set_comment(
        image.entry_stub,
        "BES mapping stub: loads the odd Thumb pointer at file offset 0x64 and branches to it",
    )
    _set_name(image.mapped_entry, "mapped_startup")
    _set_comment(
        image.mapped_entry,
        "Mapped Thumb startup target; branch targets use the execute/XIP alias",
    )

    if image.has_aliases and image.cached_base != image.xip_base:
        _set_name(image.cached_base, "bes_flash_cached_image")
        _set_comment(
            image.cached_base,
            "Cached flash alias: FLASH_BASE + OTA_CODE_OFFSET; most rodata references use this mapping",
        )
        _set_name(image.nc_base, "bes_flash_nc_image")
        _set_comment(
            image.nc_base,
            "Non-cached flash alias: FLASH_NC_BASE + OTA_CODE_OFFSET",
        )

    if image.has_ram_layout:
        startup_literals = (
            (MSP_LITERAL_OFFSET, "startup_msp_value", "Initial main-stack pointer"),
            (MSPLIM_LITERAL_OFFSET, "startup_msplim_value", "Initial main-stack lower bound"),
            (
                DATA_LMA_LITERAL_OFFSET,
                "startup_data_lma",
                "Cached-flash source of initialized DATA",
            ),
            (DATA_START_LITERAL_OFFSET, "startup_data_start", "DATA VMA start"),
            (DATA_END_LITERAL_OFFSET, "startup_data_end", "DATA VMA end"),
            (BSS_START_LITERAL_OFFSET, "startup_bss_start", "BSS VMA start"),
            (BSS_END_LITERAL_OFFSET, "startup_bss_end", "BSS VMA end"),
        )
        for offset, name, comment in startup_literals:
            ea = image.xip_base + offset
            ida_bytes.create_dword(ea, 4)
            _set_name(ea, name)
            _set_comment(ea, comment)

        _set_name(image.data_start, "__data_start")
        _set_name(image.data_end, "__data_end__bss_start")
        _set_comment(image.data_end, "Initialized DATA end and BSS start")
        _set_name(image.bss_end, "__bss_end")
        _set_name(image.msplim, "__msp_stack_limit")
        _set_name(image.msp, "__msp_stack_top")


def load_file(li, neflags, format):
    """Map a BES Vela AP image into an IDA 9.4 database."""
    image = _probe(li)
    if image is None:
        return 0

    ida_ida.inf_set_be(False)
    ida_ida.inf_set_app_bitness(32)
    chip = image.boot_config.get("CHIP", "")
    for name in PROCESSOR_BY_CHIP.get(chip, (PROCESSOR_NAME,)) + ("ARM",):
        if ida_idp.set_processor_type(name, ida_idp.SETPROC_LOADER):
            break
    else:
        raise RuntimeError("IDA could not select the ARM processor module")

    _map_xip_alias(li, image)
    _map_flash_aliases(li, image)
    _map_ram(li, image)

    # The AP executes this image as Thumb code. Only the execute alias gets
    # T=1 and execute permission; cached/NC aliases remain data mappings.
    t_reg = ida_idp.str2reg("T")
    if t_reg < 0:
        raise RuntimeError("ARM processor module does not expose the T register")
    ida_segregs.split_sreg_range(
        image.entry_stub,
        t_reg,
        1,
        ida_segregs.SR_user,
    )

    _describe_header(image)
    _describe_startup(image)
    _describe_boot_info(image)

    ida_entry.add_entry(image.entry_stub, image.entry_stub, "_start", 1)
    ida_entry.add_entry(
        image.mapped_entry,
        image.mapped_entry,
        "mapped_startup",
        1,
    )
    ida_ida.inf_set_start_ea(image.entry_stub)
    ida_ida.inf_set_start_ip(image.entry_stub)

    if image.has_aliases:
        print(
            "%s: mapped %d bytes at XIP 0x%08X, cached 0x%08X, NC 0x%08X"
            % (
                FORMAT_NAME,
                image.size,
                image.xip_base,
                image.cached_base,
                image.nc_base,
            )
        )
        print(
            "  OTA offset 0x%X, boot info file+0x%X -> 0x%08X"
            % (image.ota_offset, image.boot_info_offset, image.boot_info_ptr)
        )
    else:
        print(
            "%s: footer aliases were not validated; conservatively mapped %d bytes at XIP 0x%08X"
            % (FORMAT_NAME, image.size, image.xip_base)
        )

    if image.has_ram_layout:
        print(
            "  DATA file+0x%X -> [0x%08X,0x%08X), BSS [0x%08X,0x%08X), MSP [0x%08X,0x%08X)"
            % (
                image.data_lma_offset,
                image.data_start,
                image.data_end,
                image.bss_start,
                image.bss_end,
                image.msplim,
                image.msp,
            )
        )
    return 1


if __name__ == "__main__":
    pass
