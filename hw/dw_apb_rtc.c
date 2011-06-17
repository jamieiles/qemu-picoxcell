/*
 * QEMU model of the Synopsys DesignWare timer block.
 *
 * Copyright (c) 2011 Picochip Ltd, Jamie Iles
 *
 * This code is licenced under the GPL.
 */
#include "sysbus.h"
#include "qemu-timer.h"
#include "qemu-log.h"

struct dw_apb_rtc {
    SysBusDevice busdev;
    QEMUBH *bh;
    ptimer_state *ptimer;
    uint32_t freq_hz;
};

static uint32_t dw_rtc_readl(void *opaque, target_phys_addr_t addr)
{
    struct dw_apb_rtc *t = opaque;
    uint32_t v;

    switch (addr) {
    case 0x00:
        v = ~ptimer_get_count(t->ptimer);
        break;

    default:
        v = 0;
    }

    return v;
}

static void dw_rtc_enable(struct dw_apb_rtc *t)
{
    ptimer_stop(t->ptimer);
    ptimer_set_limit(t->ptimer, 0xffffffff, 1);
    ptimer_run(t->ptimer, 0);
}

static CPUReadMemoryFunc * const dw_rtc_read[] = {
    dw_rtc_readl,
    dw_rtc_readl,
    dw_rtc_readl
};

static CPUWriteMemoryFunc * const dw_rtc_write[] = {
    NULL, NULL, NULL
};

static void dw_rtc_hit(void *opaque)
{
}

static int dw_rtc_init(SysBusDevice *dev)
{
    struct dw_apb_rtc *t = FROM_SYSBUS(struct dw_apb_rtc, dev);
    int timer_regs;

    t->bh = qemu_bh_new(dw_rtc_hit, t);
    t->ptimer = ptimer_init(t->bh);
    ptimer_set_freq(t->ptimer, t->freq_hz);
    dw_rtc_enable(t);
    timer_regs = cpu_register_io_memory(dw_rtc_read, dw_rtc_write, t,
                                        DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0xf, timer_regs);

    return 0;
}

static SysBusDeviceInfo dw_rtc_info = {
    .init = dw_rtc_init,
    .qdev.name = "dwapb_rtc",
    .qdev.size = sizeof(struct dw_apb_rtc),
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("frequency", struct dw_apb_rtc, freq_hz, 200000000),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void dw_apb_timer_register(void)
{
    sysbus_register_withprop(&dw_rtc_info);
}
device_init(dw_apb_timer_register);
