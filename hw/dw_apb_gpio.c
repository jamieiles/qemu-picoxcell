/*
 * QEMU model of the picoxcell pc30xx Fuse block.
 *
 * Copyright (c) 2011 Picochip Ltd, Jamie Iles
 *
 * This code is licenced under the GPL.
 */
#include <stdio.h>
#include "bitmap.h"
#include "sysbus.h"
#include "qdev.h"
#include "qemu-timer.h"
#include "qemu-log.h"

struct dw_apb_gpio {
    SysBusDevice busdev;
    qemu_irq gpio[8];
    uint32_t dat;
    uint32_t set;
    uint32_t dirout;
};

static uint32_t dw_apb_gpio_readl(void *opaque, target_phys_addr_t addr)
{
    struct dw_apb_gpio *gpio = opaque;

    switch (addr) {
    case 0x00:
        return gpio->set;
    case 0x04:
        return gpio->dirout;
    case 0x50:
        return gpio->dat;
    default:
        return 0;
    }
}

static void dw_apb_gpio_update(struct dw_apb_gpio *gpio)
{
    int i;

    for (i = 0; i < 8; ++i) {
        if (!(gpio->dirout & (1 << i)))
            continue;

        qemu_set_irq(gpio->gpio[i], !!(gpio->set & (1 << i)));
    }
}

static void dw_apb_gpio_writel(void *opaque, target_phys_addr_t addr,
                               uint32_t val)
{
    struct dw_apb_gpio *gpio = opaque;

    switch (addr) {
    case 0x00:
        gpio->set = val;
        dw_apb_gpio_update(gpio);
        break;

    case 0x04:
        gpio->dirout = val;
        dw_apb_gpio_update(gpio);
        break;

    default:
        break;
    }
}

static CPUReadMemoryFunc * const dw_apb_gpio_read[] = {
    NULL,
    NULL,
    dw_apb_gpio_readl
};

static CPUWriteMemoryFunc * const dw_apb_gpio_write[] = {
    NULL,
    NULL,
    dw_apb_gpio_writel
};

static void dw_apb_gpio_set(void *opaque, int line, int level)
{
    struct dw_apb_gpio *gpio = opaque;

    gpio->dat &= ~(1 << line);
    gpio->dat |= !!level << line;
}

static int dw_apb_gpio_init(SysBusDevice *dev)
{
    struct dw_apb_gpio *t = FROM_SYSBUS(struct dw_apb_gpio, dev);
    int regs;

    regs = cpu_register_io_memory(dw_apb_gpio_read, dw_apb_gpio_write, t,
				  DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x8000, regs);

    qdev_init_gpio_out(&t->busdev.qdev, t->gpio, 8);
    qdev_init_gpio_in(&t->busdev.qdev, dw_apb_gpio_set, 8);
    t->dat = 0;

    return 0;
}

static SysBusDeviceInfo dw_apb_gpio_info = {
    .init = dw_apb_gpio_init,
    .qdev.name = "dwapb_gpio",
    .qdev.size = sizeof(struct dw_apb_gpio),
    .qdev.props = (Property[]) {
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void dw_apb_gpio_register(void)
{
    sysbus_register_withprop(&dw_apb_gpio_info);
}
device_init(dw_apb_gpio_register);
