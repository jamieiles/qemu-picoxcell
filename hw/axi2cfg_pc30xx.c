#include "sysbus.h"

struct pc30xx_pll {
    bool                sensed;
    uint32_t            divf_new;
    uint32_t            divq_new;
    uint32_t            divf;
    uint32_t            divq;
};

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
    uint32_t            gpio_mux;
    uint32_t            use_pai_gpio;
    uint32_t            use_ebi_gpio;
    uint32_t            use_decode_gpio;
    uint32_t            use_misc_int_gpio;
    uint32_t            clk_gate;
    uint32_t            radio_sel;
    uint32_t            nand_enable;
    uint32_t            decode_mux;
    uint32_t            uart_usim;

    struct pc30xx_pll   arm_pll;
    struct pc30xx_pll   amba_pll;
    struct pc30xx_pll   pico_pll;
    struct pc30xx_pll   ddr_pll;

    qemu_irq            irq;
} axi2cfg_state;

static const VMStateDescription vmstate_axi2cfg = {
    .name = "axi2cfg,pc30xx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(device_id, axi2cfg_state),
        VMSTATE_END_OF_LIST(),
    }
};

static uint32_t pll_calc_freq(struct pc30xx_pll *pll)
{
    return (((uint32_t)((20000000LLU * (1LLU << 32)) /
        ((uint64_t)(1 << pll->divq) * (uint64_t)pll->divf))) / 1000000);
}

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
    case 0x34:
        return s->gpio_mux;
    case 0x0038:
        return s->use_pai_gpio;
    case 0x003c:
        return s->use_ebi_gpio;
    case 0x0040:
        return s->use_decode_gpio;
    case 0x0044:
        return s->use_misc_int_gpio;
    case 0x0048:
        return s->clk_gate;
    case 0x004c:
        return s->radio_sel;
    case 0x0050:
        if (s->arm_pll.sensed) {
            s->arm_pll.sensed = false;
            return pll_calc_freq(&s->arm_pll) | (1 << 29);
        }
        return 0;
    case 0x0054:
        if (s->amba_pll.sensed) {
            s->amba_pll.sensed = false;
            return pll_calc_freq(&s->amba_pll) | (1 << 29);
        }
        return 0;
    case 0x0058:
        if (s->ddr_pll.sensed) {
            s->ddr_pll.sensed = false;
            return pll_calc_freq(&s->ddr_pll) | (1 << 29);
        }
        return 0;
    case 0x005c:
        if (s->pico_pll.sensed) {
            s->pico_pll.sensed = false;
            return pll_calc_freq(&s->pico_pll) | (1 << 29);
        }
        return 0;
    case 0x0060:
        return s->nand_enable;
    case 0x0064:
        return s->decode_mux;
    case 0x0068:
        return 0x3 | (0x3 << 6) | (1 << 9) | (1 << 10);
    case 0x006c:
        return 0x01; /* 20MHz input xtal. */
    case 0x0070:
        return s->arm_pll.divf_new;
    case 0x0074:
        return s->arm_pll.divq_new;
    case 0x0080:
        return s->amba_pll.divf_new;
    case 0x0084:
        return s->amba_pll.divq_new;
    case 0x0090:
        return s->ddr_pll.divf_new;
    case 0x0094:
        return s->ddr_pll.divq_new;
    case 0x00a0:
        return s->pico_pll.divf_new;
    case 0x00a4:
        return s->pico_pll.divq_new;
    case 0x00c0:
        return s->uart_usim;

    case 0x0100 ... 0x01ff:
    case 0x0200 ... 0x02ff:
    case 0x0004:
    case 0x000c:
    case 0x0010:
    case 0x0018:
    case 0x0020:
    case 0x0078:
    case 0x0088:
    case 0x0098:
    case 0x00a8:
    default:
        return 0;
    }
}

static void axi2cfg_snoop_test_write(axi2cfg_state *s, uint32_t val)
{
    s->snoop_active = (val & s->snoop_enable) & ~s->snoop_mask;

    if (s->snoop_active)
        qemu_irq_raise(s->irq);
}

static void axi2cfg_snoop_clear_write(axi2cfg_state *s, uint32_t val)
{
    s->snoop_active &= ~val;

    if (s->snoop_active)
        qemu_irq_lower(s->irq);
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
    case 0x0034:
        s->gpio_mux = val & 0xffffff;
        break;
    case 0x0038:
        s->use_pai_gpio = val & 0xffff;
        break;
    case 0x003c:
        s->use_ebi_gpio = val & 0x23ff;
        break;
    case 0x0040:
        s->use_decode_gpio = val & 0xf;
        break;
    case 0x0044:
        s->use_misc_int_gpio = val & 0x1;
        break;
    case 0x0048:
        s->clk_gate = val & 0xffff;
        break;
    case 0x004c:
        s->radio_sel = val & 0x1;
        break;
    case 0x0050:
        if (val & (1 << 31))
            s->arm_pll.sensed = true;
        break;
    case 0x0054:
        if (val & (1 << 31))
            s->amba_pll.sensed = true;
        break;
    case 0x0058:
        if (val & (1 << 31))
            s->ddr_pll.sensed = true;
        break;
    case 0x005c:
        if (val & (1 << 31))
            s->pico_pll.sensed = true;
        break;
    case 0x0060:
        s->nand_enable = val & 0x1;
        break;
    case 0x0064:
        s->decode_mux = val & 0x03030303;
        break;
    case 0x0070:
        s->arm_pll.divf_new = val & 0x3fffffff;
        break;
    case 0x0074:
        s->arm_pll.divq_new = val & 0x7;
        break;
    case 0x0078:
        if (val & (1 << 31)) {
            s->arm_pll.divq = s->arm_pll.divq_new;
            s->arm_pll.divf = s->arm_pll.divf_new;
        }
        break;
    case 0x0080:
        s->amba_pll.divf_new = val & 0x3fffffff;
        break;
    case 0x0084:
        s->amba_pll.divq_new = val & 0x7;
        break;
    case 0x0088:
        if (val & (1 << 31)) {
            s->amba_pll.divq = s->amba_pll.divq_new;
            s->amba_pll.divf = s->amba_pll.divf_new;
        }
        break;
    case 0x0090:
        s->ddr_pll.divf_new = val & 0x3fffffff;
        break;
    case 0x0094:
        s->ddr_pll.divq_new = val & 0x7;
        break;
    case 0x0098:
        if (val & (1 << 31)) {
            s->ddr_pll.divq = s->ddr_pll.divq_new;
            s->ddr_pll.divf = s->ddr_pll.divf_new;
        }
        break;
    case 0x00a0:
        s->pico_pll.divf_new = val & 0x3fffffff;
        break;
    case 0x00a4:
        s->pico_pll.divq_new = val & 0x7;
        break;
    case 0x00a8:
        if (val & (1 << 31)) {
            s->pico_pll.divq = s->pico_pll.divq_new;
            s->pico_pll.divf = s->pico_pll.divf_new;
        }
        break;
    case 0x00c0:
        s->uart_usim = val & 0x1f;
        break;

    case 0x0004:
    case 0x0014:
    case 0x0018:
    case 0x002c:
    case 0x0030:
    case 0x0068:
    case 0x006c:
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

static void axi2cfg_reset(DeviceState *d)
{
    axi2cfg_state *s = FROM_SYSBUS(axi2cfg_state, sysbus_from_qdev(d));

    s->use_pai_gpio = 0xffff;
    s->gpio_mux = 0xFF;
    s->syscfg = 0x3 | (0x3 << 11); /* NAND boot, RMII. */
    s->decode_mux = 0x2 | (0x1 << 8);
    s->nand_enable = 0x1;

    s->arm_pll.divf_new = s->arm_pll.divf = 0x2222222;
    s->arm_pll.divq_new = s->arm_pll.divq = 0x2;
    s->ddr_pll.divf_new = s->ddr_pll.divf = 0x266c8c4;
    s->ddr_pll.divq_new = s->ddr_pll.divq = 0x2;
    s->pico_pll.divf_new = s->pico_pll.divf = 0x2000000;
    s->pico_pll.divq_new = s->pico_pll.divq = 0x4;
    s->amba_pll.divf_new = s->amba_pll.divf = 0x199999a;
    s->amba_pll.divq_new = s->amba_pll.divq = 0x4;
}

static int axi2cfg_init(SysBusDevice *dev)
{
    axi2cfg_state *s = FROM_SYSBUS(axi2cfg_state, dev);
    int iomemtype;

    iomemtype = cpu_register_io_memory(axi2cfg_readfn, axi2cfg_writefn,
                                       s, DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x10000, iomemtype);

    sysbus_init_irq(dev, &s->irq);

    return 0;
}

static SysBusDeviceInfo axi2cfg_info = {
    .init = axi2cfg_init,
    .qdev.name = "axi2cfg,pc30xx",
    .qdev.size = sizeof(axi2cfg_state),
    .qdev.vmsd = &vmstate_axi2cfg,
    .qdev.reset = axi2cfg_reset,
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
