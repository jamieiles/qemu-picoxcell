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

struct dw_apb_timer {
    QEMUBH *bh;
    ptimer_state *ptimer;
    uint32_t control;
    uint32_t load_count;
    qemu_irq irq;
};

struct dw_apb_timer_block {
    SysBusDevice busdev;
    uint32_t nr_timers;
    uint32_t freq_hz;
    struct dw_apb_timer *timers;
};

static uint32_t dw_timer_readl(void *opaque, target_phys_addr_t addr)
{
    struct dw_apb_timer_block *tb = opaque;
    struct dw_apb_timer *t;
    uint32_t v;

    /* The timers are spaced at a 0x14 interval. */
    t = &tb->timers[addr / 0x14];

    addr %= 0x14;

    switch (addr) {
    case 0x04:
        v = ptimer_get_count(t->ptimer);
        break;

    case 0x08:
        v = t->control;
        break;

    case 0x0c:
        qemu_set_irq(t->irq, 0);
        v = 0;
        break;

    default:
        v = 0;
    }

    return v;
}

static void dw_timer_enable(struct dw_apb_timer *t)
{
    ptimer_stop(t->ptimer);
    ptimer_set_limit(t->ptimer, t->load_count ?: ~0, 1);
    ptimer_run(t->ptimer, 0);
}

static void dw_timer_hit(void *opaque)
{
    struct dw_apb_timer *t = opaque;

    if (!(t->control & (1 << 2)))
        qemu_set_irq(t->irq, 1);
}

static void dw_timer_writel(void *opaque, target_phys_addr_t addr,
                            uint32_t v)
{
    struct dw_apb_timer_block *tb = opaque;
    struct dw_apb_timer *t;

    /* The timers are spaced at a 0x14 interval. */
    t = &tb->timers[addr / 0x14];

    addr %= 0x14;

    switch (addr) {
    case 0x00:
        t->load_count = v;
        break;

    case 0x008:
        t->control = v;
        if (v & (1 << 0))
            dw_timer_enable(t);
        else
            ptimer_stop(t->ptimer);
        break;

    }
}

static CPUReadMemoryFunc * const dw_timer_read[] = {
    dw_timer_readl,
    dw_timer_readl,
    dw_timer_readl
};

static CPUWriteMemoryFunc * const dw_timer_write[] = {
    dw_timer_writel,
    dw_timer_writel,
    dw_timer_writel
};

static int dw_timer_init(SysBusDevice *dev)
{
    struct dw_apb_timer_block *tb = FROM_SYSBUS(struct dw_apb_timer_block,
                                                dev);
    unsigned int i;
    int timer_regs;

    tb->timers = qemu_mallocz(sizeof(*tb->timers) * tb->nr_timers);
    for (i = 0; i < tb->nr_timers; ++i) {
        struct dw_apb_timer *t = &tb->timers[i];

        t->bh = qemu_bh_new(dw_timer_hit, t);
        t->ptimer = ptimer_init(t->bh);
        ptimer_set_freq(t->ptimer, tb->freq_hz);
        sysbus_init_irq(dev, &t->irq);
    }

    timer_regs = cpu_register_io_memory(dw_timer_read, dw_timer_write, tb,
                                        DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, tb->nr_timers * 0x14, timer_regs);

    return 0;
}

static SysBusDeviceInfo dw_apb_timer_info = {
    .init = dw_timer_init,
    .qdev.name = "dwapb_timer",
    .qdev.size = sizeof(struct dw_apb_timer_block),
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("frequency", struct dw_apb_timer_block, freq_hz, 200000000),
        DEFINE_PROP_UINT32("nr-timers", struct dw_apb_timer_block, nr_timers, 2),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void dw_apb_timer_register(void)
{
    sysbus_register_withprop(&dw_apb_timer_info);
}
device_init(dw_apb_timer_register);
