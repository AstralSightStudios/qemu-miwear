/*
 * Guest-time PC sampling for Mirage boards.
 *
 * MIRAGE_PROFILE=FILE samples the first CPU's PC and LR every
 * MIRAGE_PROFILE_US microseconds of virtual time (default 50) and rewrites
 * FILE every 20000 samples with one "pc lr count" line (hex, hex, decimal)
 * per distinct pair, cumulative since boot: the difference of two dumps is
 * the profile of the time between them. Meant for -icount, where virtual
 * time is the guest's own instruction count (the timer fires between
 * translation blocks, so the registers are current) and the profile shows
 * where a real CPU of the modelled speed would spend its time, idle (WFI)
 * included. Without -icount the samples are taken against host time.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "hw/core/cpu.h"
#include "hw/arm/bes_board.h"

typedef struct {
    QEMUTimer *timer;
    int64_t period_ns;
    char *path;
    GHashTable *counts;         /* (lr << 32 | pc) -> count */
    uint64_t samples;
    GByteArray *reg;
} MirageProfile;

static void mirage_profile_dump(MirageProfile *p)
{
    g_autofree char *tmp = g_strdup_printf("%s.tmp", p->path);
    GHashTableIter it;
    gpointer k, v;
    FILE *f = fopen(tmp, "w");

    if (!f) {
        return;
    }
    fprintf(f, "# samples %" PRIu64 " period_ns %" PRId64 "\n", p->samples,
            p->period_ns);
    g_hash_table_iter_init(&it, p->counts);
    while (g_hash_table_iter_next(&it, &k, &v)) {
        uint64_t key = (uint64_t)GPOINTER_TO_SIZE(k);

        fprintf(f, "%08x %08x %" G_GSIZE_FORMAT "\n", (uint32_t)key,
                (uint32_t)(key >> 32), GPOINTER_TO_SIZE(v));
    }
    fclose(f);
    rename(tmp, p->path);
}

static void mirage_profile_tick(void *opaque)
{
    MirageProfile *p = opaque;
    CPUState *cs = first_cpu;

    timer_mod(p->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + p->period_ns);
    if (!cs) {
        return;
    }
    uint32_t pc = cs->cc->get_pc(cs);
    uint32_t lr = 0;
    g_byte_array_set_size(p->reg, 0);
    if (cs->cc->gdb_read_register(cs, p->reg, 14) == 4) {
        lr = ldl_le_p(p->reg->data);
    }
    gpointer key = GSIZE_TO_POINTER(((uint64_t)lr << 32) | pc);
    gsize n = GPOINTER_TO_SIZE(g_hash_table_lookup(p->counts, key));
    g_hash_table_insert(p->counts, key, GSIZE_TO_POINTER(n + 1));
    if (++p->samples % 20000 == 0) {
        mirage_profile_dump(p);
    }
}

void mirage_profile_start(void)
{
    const char *path = g_getenv("MIRAGE_PROFILE");
    const char *us = g_getenv("MIRAGE_PROFILE_US");
    MirageProfile *p;

    if (!path || !*path || sizeof(gpointer) < 8) {
        return;
    }
    p = g_new0(MirageProfile, 1);
    p->path = g_strdup(path);
    p->period_ns = (us && atoi(us) > 0 ? atoi(us) : 50) * 1000;
    p->counts = g_hash_table_new(g_direct_hash, g_direct_equal);
    p->reg = g_byte_array_new();
    p->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mirage_profile_tick, p);
    timer_mod(p->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + p->period_ns);
    info_report("mirage: sampling PCs every %" PRId64 " ns of virtual time "
                "into %s", p->period_ns, p->path);
}
