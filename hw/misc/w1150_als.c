/*
 * W1150 ambient light / PPG front end (Band 9 Pro, n67)
 *
 * The AP firmware's w11xx driver (sub_C3913A0, "init w1150 light device")
 * reads the device on I2C5 at 0x48 and accepts the part only when the
 * identity byte equals 0xE5 and matches the driver's table (sub_C3913A0,
 * the loop over 2e e1 e5 ea e0 e6 e8). Until the register map is modelled
 * the device answers every read with that identity byte and accepts writes
 * without effect, which is all the probe needs.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"

#define TYPE_W1150_ALS "w1150.als"
#define W1150_ID        0xE5

static uint8_t w1150_recv(I2CSlave *i2c)
{
    return W1150_ID;
}

static int w1150_send(I2CSlave *i2c, uint8_t data)
{
    return 0;
}

static void w1150_class_init(ObjectClass *klass, const void *data)
{
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    sc->send = w1150_send;
    sc->recv = w1150_recv;
}

static const TypeInfo w1150_info = {
    .name          = TYPE_W1150_ALS,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(I2CSlave),
    .class_init    = w1150_class_init,
};

static void w1150_register_types(void)
{
    type_register_static(&w1150_info);
}

type_init(w1150_register_types)
