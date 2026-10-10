/*
 * GPIO keys: host keys pressing a board's push buttons
 *
 * Key n is held down while the host key with QKeyCode "qcodes"[n] is, and
 * drives output line "key" n: high while pressed, or low for the keys
 * whose bit is set in "active-low" (a button that shorts a pulled-up pin
 * to ground). Anything that feeds the QEMU input layer presses them: the
 * GUI keyboard, QMP send-key and input-send-event.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/input/gpio_keys.h"
#include "system/reset.h"

static void gpio_keys_set(GpioKeysState *s, unsigned n, bool down)
{
    bool low = s->active_low & (1u << n);

    s->pressed[n] = down;
    qemu_set_irq(s->out[n], down != low);
}

static void gpio_keys_event(DeviceState *dev, QemuConsole *src,
                            QemuInputEvent *evt)
{
    GpioKeysState *s = GPIO_KEYS(dev);
    int qcode = qemu_input_linux_to_qcode(evt->key.key);
    unsigned n;

    for (n = 0; n < s->num_keys; n++) {
        if (s->qcodes[n] == qcode && s->pressed[n] != evt->key.down) {
            gpio_keys_set(s, n, evt->key.down);
        }
    }
}

static const QemuInputHandler gpio_keys_handler = {
    .name  = "GPIO keys",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = gpio_keys_event,
};

/* the lines are wired after realize: drive the released levels on reset */
static void gpio_keys_reset(void *opaque)
{
    GpioKeysState *s = opaque;
    unsigned n;

    for (n = 0; n < s->num_keys; n++) {
        gpio_keys_set(s, n, false);
    }
}

static void gpio_keys_realize(DeviceState *dev, Error **errp)
{
    GpioKeysState *s = GPIO_KEYS(dev);

    if (s->num_keys > 32) {
        error_setg(errp, "gpio-keys: at most 32 keys");
        return;
    }
    s->pressed = g_new0(bool, s->num_keys);
    s->out = g_new0(qemu_irq, s->num_keys);
    qdev_init_gpio_out_named(dev, s->out, "key", s->num_keys);
    s->hs = qemu_input_handler_register(dev, &gpio_keys_handler);
    qemu_register_reset(gpio_keys_reset, s);
}

static const Property gpio_keys_properties[] = {
    DEFINE_PROP_ARRAY("qcodes", GpioKeysState, num_keys, qcodes,
                      qdev_prop_uint32, uint32_t),
    DEFINE_PROP_UINT32("active-low", GpioKeysState, active_low, 0),
};

static void gpio_keys_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "push buttons pressed by host keys";
    dc->realize = gpio_keys_realize;
    device_class_set_props(dc, gpio_keys_properties);
}

static const TypeInfo gpio_keys_types[] = {
    {
        .name = TYPE_GPIO_KEYS,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(GpioKeysState),
        .class_init = gpio_keys_class_init,
    },
};

DEFINE_TYPES(gpio_keys_types)
