#include "sysbus.h"

typedef struct {
    SysBusDevice        busdev;
    uint32_t            syscfg;
    uint32_t            irq_reg;
    uint32_t            device_id;
    uint32_t            snoop_active;
    uint32_t            snoop_enable;
    uint32_t            snoop_mask;
    uint32_t            snoop_pre_mask;
    uint32_t            snoop_post_mask;

    qemu_irq            rd_irq;
    qemu_irq            wr_irq;
} axi2cfg_state;

static const VMStateDescription vmstate_axi2cfg = {
    .name = "axi2cfg,pc3x2",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(device_id, axi2cfg_state),
        VMSTATE_END_OF_LIST(),
    }
};

static uint32_t axi2cfg_read(void *opaque, target_phys_addr_t offset)
{
    axi2cfg_state *s = (axi2cfg_state *)opaque;

    switch (offset) {
    case 0x0000:
        return s->syscfg;
    case 0x0008:
        return s->irq_reg;
    case 0x0014:
        return s->device_id;
    case 0x001c:
        return s->snoop_enable;
    case 0x0024:
        return s->snoop_mask;
    case 0x002c:
    case 0x0030:
        return s->snoop_active;

    case 0x0100 ... 0x01ff:
    case 0x0200 ... 0x02ff:
    case 0x0004:
    case 0x000c:
    case 0x0010:
    case 0x0018:
    case 0x0020:
    default:
        return 0;
    }
}

static void axi2cfg_snoop_test_write(axi2cfg_state *s, uint32_t val)
{
    s->snoop_active = (val & s->snoop_enable) & ~s->snoop_mask;

    if (s->snoop_active & 0xfff)
        qemu_irq_raise(s->rd_irq);
    if (s->snoop_active & 0x0fff0000)
        qemu_irq_raise(s->wr_irq);
}

static void axi2cfg_snoop_clear_write(axi2cfg_state *s, uint32_t val)
{
    s->snoop_active &= ~val;

    if (!(s->snoop_active & 0xfff))
        qemu_irq_lower(s->rd_irq);
    if (!(s->snoop_active & 0x0fff0000))
        qemu_irq_lower(s->wr_irq);
}

static void axi2cfg_write(void *opaque, target_phys_addr_t offset,
                          uint32_t val)
{
    axi2cfg_state *s = (axi2cfg_state *)opaque;

    switch (offset) {
    case 0x0000:
        s->syscfg = val;
        break;
    case 0x0008:
        s->irq_reg = val;
        break;
    case 0x001c:
        s->snoop_enable = val;
        break;
    case 0x0020:
        axi2cfg_snoop_clear_write(s, val);
        break;
    case 0x0024:
        s->snoop_mask = val;
        break;
    case 0x0028:
        axi2cfg_snoop_test_write(s, val);
        break;
    case 0x0004:
    case 0x0014:
    case 0x0018:
    case 0x002c:
    case 0x0030:
    case 0x0200 ... 0x02ff:
        hw_error("axi2cfg read only register offset %u\n", (unsigned)offset);

    case 0x0100 ... 0x01ff: /* Config write port not emulated. */
    case 0x000c: /* Config ports don't need any purging. */
    default:
        return;
    };
}

static CPUReadMemoryFunc * const axi2cfg_readfn[] = {
    axi2cfg_read,
    axi2cfg_read,
    axi2cfg_read
};

static CPUWriteMemoryFunc * const axi2cfg_writefn[] = {
    axi2cfg_write,
    axi2cfg_write,
    axi2cfg_write
};

static int axi2cfg_init(SysBusDevice *dev)
{
    axi2cfg_state *s = FROM_SYSBUS(axi2cfg_state, dev);
    int iomemtype;

    iomemtype = cpu_register_io_memory(axi2cfg_readfn, axi2cfg_writefn,
                                       s, DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x10000, iomemtype);

    sysbus_init_irq(dev, &s->rd_irq);
    sysbus_init_irq(dev, &s->wr_irq);

    return 0;
}

static SysBusDeviceInfo axi2cfg_info = {
    .init = axi2cfg_init,
    .qdev.name = "axi2cfg,pc3x2",
    .qdev.size = sizeof(axi2cfg_state),
    .qdev.vmsd = &vmstate_axi2cfg,
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("device_id", axi2cfg_state, device_id, 0),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void axi2cfg_register_devices(void)
{
    sysbus_register_withprop(&axi2cfg_info);
}
device_init(axi2cfg_register_devices);
