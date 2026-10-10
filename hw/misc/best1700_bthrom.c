/*
 * BES best1700 BTH core boot ROM stand-in
 *
 * The sensor hub firmware the BTHC0 core runs (vela_sensor.bin) calls into
 * the core's mask ROM at 0x03000000 through veneers (ldr pc, =0x03xxxxxx,
 * at 0x0c000830 and 0x0c24c3b0) and passes a few ROM functions around as
 * callbacks. No ROM image is available, so this device provides one: every
 * entry point the firmware uses gets a stub, and the stubs either do the
 * job in Thumb code (interrupt masking, the system timer) or hand their
 * arguments to QEMU through a small hypercall window, where the function is
 * implemented in C. Everything else in the ROM is UDF, so a call to an
 * entry point not listed here faults where it can be seen.
 *
 * What each entry point does is read from the firmware's call sites:
 *  - libc: memset (1542 calls, mostly with c = 0), memcpy and memmove (the
 *    firmware wraps them in its own checks), strcmp, strlen, memchr
 *  - hal_trace_printf(attr, fmt, ...): the trace output, sent to the
 *    best1700_bth_log trace event line by line
 *  - int_lock/int_unlock and a second lock pair around BTH_CMU updates:
 *    PRIMASK save/disable/restore
 *  - hal_sys_timer_get: the 16 kHz AON system timer (AON_CMU 0x50); the
 *    millisecond and microsecond to tick conversions; the busy-wait delays
 *    return at once
 *  - the idle loop's sleep: the check that picks a sleep mode returns 0
 *    and the sleep itself is a WFI
 *  - hal_norflash_get_size: 8 MB (bes_chip_flash_init asserts that size)
 *  - hal_cmu_get_crystal_freq: 24 MHz; the sleep timer calibration
 *    (crystal cycles against slow ticks) reports the nominal ratio
 * The rest are initialisation helpers whose return values the firmware
 * ignores or treats 0 as success; they are logged (best1700_bth_rom) and
 * return 0.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "system/address-spaces.h"
#include "hw/misc/best1700_bthrom.h"
#include "trace.h"

#define ROM_SIZE            0x10000
#define ROM_DISPATCH        0xF000      /* hypercall dispatcher */
#define HC_ARGS             0x00        /* guest: pointer to r0..r3, stack */
#define HC_CALL             0x04        /* guest: function id, runs it */
#define HC_RET0             0x08
#define HC_RET1             0x0C

#define AON_SYS_TIMER       0x40080050  /* hal_sys_timer_get */
#define SYS_TIMER_HZ        16000
#define XTAL_HZ             24000000    /* the AP's hal_cmu tables */

typedef enum {
    ROM_HC,             /* hypercall */
    ROM_RET,            /* bx lr */
    ROM_INT_LOCK,       /* mrs r0, primask; cpsid i; bx lr */
    ROM_INT_UNLOCK,     /* msr primask, r0; bx lr */
    ROM_TIMER_GET,      /* r0 = AON system timer */
    ROM_WFI,            /* wfi; movs r0, #0; bx lr */
    ROM_RET0,           /* movs r0, #0; bx lr */
} RomKind;

enum {
    F_MEMSET = 1, F_MEMCHR, F_MEMCPY, F_MEMMOVE, F_STRCMP, F_STRLEN,
    F_TRACE, F_MS_TO_TICKS, F_US_TO_TICKS, F_DELAY, F_FLASH_SIZE,
    F_XTAL_HZ, F_SYSTICK_HZ, F_SLOW_CALIB,
    F_OTHER,
};

typedef struct RomEntry {
    uint32_t addr;
    RomKind kind;
    int fn;
    const char *name;
} RomEntry;

static const RomEntry rom_entries[] = {
    { 0x03000440, ROM_HC, F_MEMSET, "memset" },
    { 0x03000470, ROM_HC, F_MEMCHR, "memchr" },
    { 0x03000510, ROM_HC, F_MEMCPY, "memcpy" },
    { 0x0300061c, ROM_HC, F_MEMMOVE, "memmove" },
    { 0x03000658, ROM_HC, F_STRCMP, "strcmp" },
    { 0x03000840, ROM_HC, F_STRLEN, "strlen" },
    { 0x030025c0, ROM_HC, F_TRACE, "hal_trace_printf" },
    { 0x03002638, ROM_INT_LOCK, 0, "int_lock" },
    { 0x03002646, ROM_INT_UNLOCK, 0, "int_unlock" },
    { 0x030029e0, ROM_INT_LOCK, 0, "lock" },
    { 0x03002a08, ROM_INT_UNLOCK, 0, "unlock" },
    { 0x03002b88, ROM_TIMER_GET, 0, "hal_sys_timer_get" },
    { 0x03002d74, ROM_HC, F_MS_TO_TICKS, "ms_to_ticks" },
    { 0x03002da8, ROM_HC, F_US_TO_TICKS, "us_to_ticks" },
    { 0x03002ba0, ROM_HC, F_DELAY, "hal_sys_timer_delay" },
    { 0x03002bb4, ROM_HC, F_DELAY, "delay_us" },
    { 0x03005c88, ROM_HC, F_DELAY, "delay_us2" },
    { 0x03002a2c, ROM_RET, 0, "init_a" },
    { 0x03002a2e, ROM_RET, 0, "init_b" },
    { 0x03002a30, ROM_HC, F_OTHER, "init_c" },
    { 0x03001194, ROM_HC, F_FLASH_SIZE, "hal_norflash_get_size" },
    /* not identified yet: logged, return 0 */
    { 0x03000504, ROM_HC, F_OTHER, "rom_03000504" },
    { 0x03000d4c, ROM_HC, F_OTHER, "rom_03000d4c" },
    { 0x0300113c, ROM_HC, F_OTHER, "rom_0300113c" },
    { 0x03001158, ROM_HC, F_OTHER, "rom_03001158" },
    { 0x030011a4, ROM_HC, F_OTHER, "hal_norflash_open" },
    { 0x030016f8, ROM_HC, F_OTHER, "flash_cb_030016f8" },
    { 0x03001834, ROM_HC, F_OTHER, "flash_cb_03001834" },
    { 0x03002650, ROM_HC, F_OTHER, "rom_03002650" },
    { 0x0300265c, ROM_HC, F_OTHER, "rom_0300265c" },
    { 0x03002680, ROM_HC, F_OTHER, "rom_03002680" },
    { 0x03002a48, ROM_HC, F_OTHER, "rom_03002a48" },
    { 0x03002a94, ROM_HC, F_OTHER, "rom_03002a94" },
    { 0x03002b50, ROM_HC, F_XTAL_HZ, "hal_cmu_get_crystal_freq" },
    { 0x03002b58, ROM_HC, F_SYSTICK_HZ, "hal_sys_timer_systick_hz" },
    { 0x03002b94, ROM_HC, F_OTHER, "rom_03002b94" },
    { 0x03002d40, ROM_HC, F_OTHER, "rom_03002d40" },
    { 0x03002de4, ROM_HC, F_OTHER, "rom_03002de4" },
    { 0x030042cc, ROM_HC, F_OTHER, "rom_030042cc" },
    { 0x0300442c, ROM_HC, F_OTHER, "norflash_api_register" },
    { 0x03004820, ROM_HC, F_OTHER, "rom_03004820" },
    { 0x0300487c, ROM_HC, F_OTHER, "rom_0300487c" },
    { 0x03004888, ROM_HC, F_OTHER, "rom_03004888" },
    { 0x0300489a, ROM_HC, F_OTHER, "rom_0300489a" },
    { 0x030048c8, ROM_HC, F_OTHER, "rom_030048c8" },
    { 0x03004a8c, ROM_HC, F_OTHER, "rom_03004a8c" },
    { 0x03005b10, ROM_WFI, 0, "hal_sleep_enter" },
    { 0x03005b4c, ROM_RET0, 0, "hal_sleep_check" },
    { 0x03005c1c, ROM_HC, F_SLOW_CALIB, "slow_timer_calib" },
    { 0x03005e08, ROM_HC, F_OTHER, "cb_03005e08" },
};

/* ---- Thumb encoding ---- */

static void rom_put16(uint8_t *rom, uint32_t off, uint16_t hw)
{
    stw_le_p(rom + off, hw);
}

/* B.W (T4) from the instruction at off to target */
static void rom_put_bw(uint8_t *rom, uint32_t off, uint32_t target)
{
    int32_t imm = (int32_t)(target - (off + 4));
    uint32_t s = (imm >> 24) & 1;
    uint32_t i1 = (imm >> 23) & 1, i2 = (imm >> 22) & 1;
    uint32_t j1 = !(i1 ^ s), j2 = !(i2 ^ s);

    rom_put16(rom, off, 0xF000 | (s << 10) | ((imm >> 12) & 0x3FF));
    rom_put16(rom, off + 2, 0x9000 | (j1 << 13) | (j2 << 11) |
              ((imm >> 1) & 0x7FF));
}

/* LDR Rt, [PC, #imm] to a literal at lit (16-bit encoding) */
static void rom_put_ldr_lit(uint8_t *rom, uint32_t off, int rt, uint32_t lit)
{
    uint32_t pc = (off + 4) & ~3u;

    assert(lit >= pc && !((lit - pc) & 3) && lit - pc < 1024);
    rom_put16(rom, off, 0x4800 | (rt << 8) | ((lit - pc) >> 2));
}

static void bthrom_build(Best1700BthRomState *s, uint8_t *rom)
{
    uint32_t d = ROM_DISPATCH;
    unsigned i;

    for (i = 0; i < ROM_SIZE; i += 2) {
        rom_put16(rom, i, 0xDEFE);              /* udf #0xfe */
    }

    /* dispatcher: push {r0-r3}; r2 = window; window.args = sp;
     * window.call = r12; r0/r1 = window.ret; add sp, #16; bx lr */
    rom_put16(rom, d + 0x00, 0xB40F);
    rom_put_ldr_lit(rom, d + 0x02, 2, d + 0x14);
    rom_put16(rom, d + 0x04, 0x466B);
    rom_put16(rom, d + 0x06, 0x6013);
    rom_put16(rom, d + 0x08, 0xF8C2);
    rom_put16(rom, d + 0x0A, 0xC000 | HC_CALL);
    rom_put16(rom, d + 0x0C, 0x6890);
    rom_put16(rom, d + 0x0E, 0x68D1);
    rom_put16(rom, d + 0x10, 0xB004);
    rom_put16(rom, d + 0x12, 0x4770);
    stl_le_p(rom + d + 0x14, s->hc_base);

    for (i = 0; i < ARRAY_SIZE(rom_entries); i++) {
        const RomEntry *e = &rom_entries[i];
        uint32_t off = e->addr - s->rom_base;

        switch (e->kind) {
        case ROM_HC:
            rom_put16(rom, off, 0xF04F);        /* mov.w r12, #i */
            rom_put16(rom, off + 2, 0x0C00 | i);
            rom_put_bw(rom, off + 4, d);
            break;
        case ROM_RET:
            rom_put16(rom, off, 0x4770);
            break;
        case ROM_INT_LOCK:
            rom_put16(rom, off, 0xF3EF);        /* mrs r0, primask */
            rom_put16(rom, off + 2, 0x8010);
            rom_put16(rom, off + 4, 0xB672);    /* cpsid i */
            rom_put16(rom, off + 6, 0x4770);
            break;
        case ROM_INT_UNLOCK:
            rom_put16(rom, off, 0xF380);        /* msr primask, r0 */
            rom_put16(rom, off + 2, 0x8810);
            rom_put16(rom, off + 4, 0x4770);
            break;
        case ROM_WFI:
            rom_put16(rom, off, 0xBF30);
            rom_put16(rom, off + 2, 0x2000);
            rom_put16(rom, off + 4, 0x4770);
            break;
        case ROM_RET0:
            rom_put16(rom, off, 0x2000);
            rom_put16(rom, off + 2, 0x4770);
            break;
        case ROM_TIMER_GET:
            assert(!(off & 3));
            rom_put_ldr_lit(rom, off, 3, off + 8);
            rom_put16(rom, off + 2, 0x6818);    /* ldr r0, [r3] */
            rom_put16(rom, off + 4, 0x4770);
            rom_put16(rom, off + 6, 0xBF00);
            stl_le_p(rom + off + 8, AON_SYS_TIMER);
            break;
        }
    }
}

/* ---- guest memory ---- */

static uint32_t g_ld32(Best1700BthRomState *s, uint32_t a)
{
    return address_space_ldl_le(&s->as, a, MEMTXATTRS_UNSPECIFIED, NULL);
}

static uint8_t g_ld8(Best1700BthRomState *s, uint32_t a)
{
    return address_space_ldub(&s->as, a, MEMTXATTRS_UNSPECIFIED, NULL);
}

static void g_read(Best1700BthRomState *s, uint32_t a, void *buf, uint32_t n)
{
    address_space_read(&s->as, a, MEMTXATTRS_UNSPECIFIED, buf, n);
}

static void g_write(Best1700BthRomState *s, uint32_t a, const void *buf,
                    uint32_t n)
{
    address_space_write(&s->as, a, MEMTXATTRS_UNSPECIFIED, buf, n);
}

static uint32_t g_strlen(Best1700BthRomState *s, uint32_t a)
{
    uint32_t n = 0;

    while (n < 0x100000 && g_ld8(s, a + n)) {
        n++;
    }
    return n;
}

static char *g_strdup_n(Best1700BthRomState *s, uint32_t a, uint32_t max)
{
    uint32_t n = MIN(g_strlen(s, a), max);
    char *str = g_malloc(n + 1);

    g_read(s, a, str, n);
    str[n] = 0;
    return str;
}

/* ---- hal_trace_printf ---- */

typedef struct VaArgs {
    Best1700BthRomState *s;
    uint32_t args;      /* guest address of the argument words */
    unsigned idx;
} VaArgs;

static uint32_t va_u32(VaArgs *va)
{
    return g_ld32(va->s, va->args + 4 * va->idx++);
}

static uint64_t va_u64(VaArgs *va)
{
    uint64_t lo, hi;

    va->idx = (va->idx + 1) & ~1u;          /* AAPCS: 8-byte aligned */
    lo = va_u32(va);
    hi = va_u32(va);
    return lo | hi << 32;
}

static void bthrom_format(GString *out, Best1700BthRomState *s, uint32_t fmt,
                          VaArgs *va)
{
    g_autofree char *f = g_strdup_n(s, fmt, 1024);
    const char *p = f;

    while (*p) {
        char spec[32];
        unsigned sl = 0;
        int lng = 0;

        if (*p != '%') {
            g_string_append_c(out, *p++);
            continue;
        }
        spec[sl++] = *p++;
        while (*p && strchr("-+ #0", *p) && sl < 20) {
            spec[sl++] = *p++;
        }
        if (*p == '*') {
            sl += snprintf(spec + sl, sizeof(spec) - sl, "%d",
                           (int32_t)va_u32(va));
            p++;
        }
        while (g_ascii_isdigit(*p) && sl < 24) {
            spec[sl++] = *p++;
        }
        if (*p == '.') {
            spec[sl++] = *p++;
            if (*p == '*') {
                sl += snprintf(spec + sl, sizeof(spec) - sl, "%d",
                               (int32_t)va_u32(va));
                p++;
            }
            while (g_ascii_isdigit(*p) && sl < 28) {
                spec[sl++] = *p++;
            }
        }
        while (*p && strchr("hlzjtL", *p)) {
            if (*p == 'l') {
                lng++;
            }
            p++;
        }
        if (!*p) {
            break;
        }
        switch (*p) {
        case 'd': case 'i':
            spec[sl++] = 'l'; spec[sl++] = 'l'; spec[sl++] = 'd'; spec[sl] = 0;
            g_string_append_printf(out, spec, lng >= 2 ?
                                   (long long)(int64_t)va_u64(va) :
                                   (long long)(int32_t)va_u32(va));
            break;
        case 'u': case 'x': case 'X': case 'o':
            spec[sl++] = 'l'; spec[sl++] = 'l'; spec[sl++] = *p; spec[sl] = 0;
            g_string_append_printf(out, spec, lng >= 2 ?
                                   (unsigned long long)va_u64(va) :
                                   (unsigned long long)va_u32(va));
            break;
        case 'p':
            g_string_append_printf(out, "0x%08x", va_u32(va));
            break;
        case 'c':
            spec[sl++] = 'c'; spec[sl] = 0;
            g_string_append_printf(out, spec, (int)va_u32(va));
            break;
        case 's': {
            uint32_t a = va_u32(va);
            g_autofree char *str = a ? g_strdup_n(s, a, 512) : g_strdup("(null)");

            spec[sl++] = 's'; spec[sl] = 0;
            g_string_append_printf(out, spec, str);
            break;
        }
        case 'f': case 'g': case 'e':
            va_u64(va);
            g_string_append(out, "<float>");
            break;
        case '%':
            g_string_append_c(out, '%');
            break;
        default:
            g_string_append_c(out, *p);
            break;
        }
        p++;
    }
}

static void bthrom_trace(Best1700BthRomState *s, uint32_t args)
{
    VaArgs va = { s, args, 2 };
    char *nl;

    bthrom_format(s->line, s, g_ld32(s, args + 4), &va);
    while ((nl = strchr(s->line->str, '\n'))) {
        *nl = 0;
        trace_best1700_bth_log(s->line->str);
        g_string_erase(s->line, 0, nl - s->line->str + 1);
    }
    if (s->line->len > 512) {
        trace_best1700_bth_log(s->line->str);
        g_string_truncate(s->line, 0);
    }
}

/* ---- hypercall window ---- */

static void bthrom_call(Best1700BthRomState *s, unsigned idx)
{
    const RomEntry *e;
    uint32_t a = s->args, r0, r1, r2;
    uint32_t ret = 0;

    if (idx >= ARRAY_SIZE(rom_entries)) {
        return;
    }
    e = &rom_entries[idx];
    r0 = g_ld32(s, a);
    r1 = g_ld32(s, a + 4);
    r2 = g_ld32(s, a + 8);

    switch (e->fn) {
    case F_MEMSET: {
        uint8_t buf[256];
        uint32_t n = r2, off = 0;

        memset(buf, r1, sizeof(buf));
        while (off < n) {
            uint32_t chunk = MIN(n - off, sizeof(buf));

            g_write(s, r0 + off, buf, chunk);
            off += chunk;
        }
        ret = r0;
        break;
    }
    case F_MEMCPY:
    case F_MEMMOVE: {
        g_autofree uint8_t *buf = g_malloc(r2 ? r2 : 1);

        g_read(s, r1, buf, r2);
        g_write(s, r0, buf, r2);
        ret = r0;
        break;
    }
    case F_MEMCHR: {
        uint32_t i;

        for (i = 0; i < r2; i++) {
            if (g_ld8(s, r0 + i) == (uint8_t)r1) {
                ret = r0 + i;
                break;
            }
        }
        break;
    }
    case F_STRCMP: {
        uint32_t i;

        for (i = 0;; i++) {
            uint8_t c0 = g_ld8(s, r0 + i), c1 = g_ld8(s, r1 + i);

            if (c0 != c1 || !c0) {
                ret = (uint32_t)((int)c0 - (int)c1);
                break;
            }
        }
        break;
    }
    case F_STRLEN:
        ret = g_strlen(s, r0);
        break;
    case F_TRACE:
        bthrom_trace(s, a);
        break;
    case F_MS_TO_TICKS:
        ret = r0 * (SYS_TIMER_HZ / 1000);
        break;
    case F_US_TO_TICKS:
        ret = DIV_ROUND_UP((uint64_t)r0 * SYS_TIMER_HZ, 1000000);
        break;
    case F_DELAY:
        break;
    case F_FLASH_SIZE:
        ret = s->flash_size;
        break;
    case F_XTAL_HZ:
        ret = XTAL_HZ;
        break;
    case F_SYSTICK_HZ:
        ret = SYS_TIMER_HZ;
        break;
    case F_SLOW_CALIB: {
        /* r0 crystal/4 cycles: report the slow ticks they take */
        uint32_t ticks = muldiv64(r0, SYS_TIMER_HZ, XTAL_HZ / 4);

        g_write(s, r1, &ticks, 4);
        break;
    }
    default:
        trace_best1700_bth_rom(e->addr, e->name, r0, r1, r2,
                               g_ld32(s, a + 12));
        break;
    }
    s->ret[0] = ret;
    s->ret[1] = 0;
}

static uint64_t bthrom_hc_read(void *opaque, hwaddr offset, unsigned size)
{
    Best1700BthRomState *s = opaque;

    switch (offset) {
    case HC_RET0:
        return s->ret[0];
    case HC_RET1:
        return s->ret[1];
    default:
        return 0;
    }
}

static void bthrom_hc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    Best1700BthRomState *s = opaque;

    switch (offset) {
    case HC_ARGS:
        s->args = value;
        break;
    case HC_CALL:
        bthrom_call(s, value);
        break;
    }
}

static const MemoryRegionOps bthrom_hc_ops = {
    .read = bthrom_hc_read,
    .write = bthrom_hc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void bthrom_realize(DeviceState *dev, Error **errp)
{
    Best1700BthRomState *s = BEST1700_BTHROM(dev);

    if (!s->memory) {
        error_setg(errp, "best1700.bthrom: 'memory' link not set");
        return;
    }
    address_space_init(&s->as, s->memory, "best1700.bthrom-as");
    memory_region_init_rom(&s->rom, OBJECT(dev), "best1700.bthrom", ROM_SIZE,
                           errp);
    if (*errp) {
        return;
    }
    bthrom_build(s, memory_region_get_ram_ptr(&s->rom));
    memory_region_init_io(&s->hc, OBJECT(dev), &bthrom_hc_ops, s,
                          "best1700.bthrom-hc", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->rom);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->hc);
    s->line = g_string_new(NULL);
}

static const Property bthrom_props[] = {
    DEFINE_PROP_LINK("memory", Best1700BthRomState, memory,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_UINT32("rom-base", Best1700BthRomState, rom_base, 0x03000000),
    DEFINE_PROP_UINT32("hc-base", Best1700BthRomState, hc_base, 0x5FFFF000),
    DEFINE_PROP_UINT32("flash-size", Best1700BthRomState, flash_size,
                       0x800000),
};

static void bthrom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bthrom_realize;
    device_class_set_props(dc, bthrom_props);
}

static const TypeInfo bthrom_info = {
    .name = TYPE_BEST1700_BTHROM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Best1700BthRomState),
    .class_init = bthrom_class_init,
};

static void bthrom_register_types(void)
{
    type_register_static(&bthrom_info);
}

type_init(bthrom_register_types)
