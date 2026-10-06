/*
 * BES 通用占位设备
 *
 * 用途：已确认"RAM 语义安全"（写后读回、无副作用）的外设区域的占位。
 * 所有访问打日志（-d unimp），便于侦察。
 * 严禁用它猜值喂卡点——状态位行为必须先搞清语义再写正式设备。
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_stub.h"

#define BES_STUB_NREGS 1024   /* 4KB 窗口 */

struct BesStubState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[BES_STUB_NREGS];
    char *tag;                /* 日志标签（区分实例） */
};

static uint64_t bes_stub_read(void *opaque, hwaddr offset, unsigned size)
{
    BesStubState *s = BES_STUB(opaque);
    uint32_t idx = offset >> 2;
    uint32_t v = (idx < BES_STUB_NREGS) ? s->regs[idx] : 0;

    qemu_log_mask(LOG_UNIMP, "bes.%s: read  off=0x%03x -> 0x%08x\n",
                  s->tag ? s->tag : "stub", (unsigned)offset, v);
    return v;
}

static void bes_stub_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    BesStubState *s = BES_STUB(opaque);
    uint32_t idx = offset >> 2;

    qemu_log_mask(LOG_UNIMP, "bes.%s: write off=0x%03x = 0x%08x\n",
                  s->tag ? s->tag : "stub", (unsigned)offset,
                  (unsigned)value);
    if (idx < BES_STUB_NREGS) {
        s->regs[idx] = value;
    }
}

static const MemoryRegionOps bes_stub_ops = {
    .read = bes_stub_read,
    .write = bes_stub_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void bes_stub_realize(DeviceState *dev, Error **errp)
{
    BesStubState *s = BES_STUB(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_stub_ops, s,
                          "bes.stub", BES_STUB_NREGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property bes_stub_props[] = {
    DEFINE_PROP_STRING("tag", BesStubState, tag),
};

static void bes_stub_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_stub_realize;
    device_class_set_props(dc, bes_stub_props);
}

static const TypeInfo bes_stub_info = {
    .name = TYPE_BES_STUB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesStubState),
    .class_init = bes_stub_class_init,
};

static void bes_stub_register_types(void)
{
    type_register_static(&bes_stub_info);
}

type_init(bes_stub_register_types)
