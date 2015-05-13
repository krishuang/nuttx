#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_spi.h>

enum tsb_spi_state {
    TSB_SPI_STATE_INVALID,
    TSB_SPI_STATE_CLOSED,
    TSB_SPI_STATE_OPEN,
    TSB_SPI_STATE_LOCKED,
};

struct tsb_spi_info {
    struct device       *dev;
    uint32_t            reg_base;
    enum tsb_spi_state  state;

    /* supported capabilities */
    uint16_t            modes;
    uint16_t            flags;
    uint16_t            bpw;
    uint16_t            csnum;

    sem_t               exclsem;
};

static int tsb_spi_lock(struct device *dev, bool lock) {
    struct tsb_spi_info *info = dev->private;

    lldbg("%s: lock:%d\n",__func__,lock);
    if (lock) {
      /* Take the semaphore (perhaps waiting) */
        while (sem_wait(&info->exclsem) != 0) {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */
            ASSERT(errno == EINTR);
        }
        info->state = TSB_SPI_STATE_LOCKED;
    } else {
        sem_post(&info->exclsem);
        info->state = TSB_SPI_STATE_OPEN;
    }
  return OK;
}

static int tsb_spi_select(struct device *dev, int devid, bool selected) {
    struct tsb_spi_info *info = dev->private;
    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("%s: slave dev %d - selected:%d\n",__func__,devid, selected);
    return SUCCESS;
}

static int tsb_spi_setfrequency(struct device *dev, uint32_t *frequency) {
    struct tsb_spi_info *info = dev->private;
    uint32_t freq = *frequency;
    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("%s: freq:%d\n",__func__,freq);
    return SUCCESS;
}

static int tsb_spi_setmode(struct device *dev, uint16_t mode) {
    struct tsb_spi_info *info = dev->private;
    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("%s: mode:%d\n",__func__,mode);
    return SUCCESS;
}

static int tsb_spi_setbits(struct device *dev, int nbits) {
    struct tsb_spi_info *info = dev->private;
    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("%s: nbits:%d\n",__func__,nbits);
    return SUCCESS;
}

static int tsb_spi_exchange(struct device *dev,
                             struct device_spi_transfer *transfer) {
    struct tsb_spi_info *info = dev->private;
    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("%s\n",__func__);
    return SUCCESS;
}

static int tsb_spi_irq_handler(int irq, void *context)
{
    lldbg("%s\n",__func__);
    /* TODO */
    return SUCCESS;
}

static int tsb_spi_getcaps(struct device *dev, struct device_spi_caps *caps) {
    /* TODO */
    lldbg("%s\n",__func__);
    return SUCCESS;
}

static int tsb_spi_dev_open(struct device *dev)
{
    struct tsb_spi_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    lldbg("%s\n",__func__);
    flags = irqsave();

    if (info->state != TSB_SPI_STATE_CLOSED) {
        ret = -EBUSY;
        goto err_irqrestore;
    }
    info->state = TSB_SPI_STATE_OPEN;

err_irqrestore:
    irqrestore(flags);
    return ret;
}

static void tsb_spi_dev_close(struct device *dev)
{
    struct tsb_spi_info *info = dev->private;
    irqstate_t flags;

    lldbg("%s\n",__func__);
    flags = irqsave();
    info->state = TSB_SPI_STATE_CLOSED;
    irqrestore(flags);
}

static int tsb_spi_dev_probe(struct device *dev)
{
    struct tsb_spi_info *info;
    struct device_resource *r;
    irqstate_t flags;
    int ret = OK;

    lldbg("%s\n",__func__);
    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_REGS, "reg_base");
    if (!r) {
        ret = -EINVAL;
        goto err_freemem;
    }

    flags = irqsave();
    ret = irq_attach(TSB_IRQ_SPI, tsb_spi_irq_handler);
    if (ret != OK) {
        ret = -EIO;
        goto err_irqrestore;
    }
#if 0
    // only ES3 chip supported SPI Master, disable interrupt now.
    up_enable_irq(TSB_IRQ_SPI);
#endif

    info->dev = dev;
    info->reg_base = r->start;
    info->state = TSB_SPI_STATE_CLOSED;
    dev->private = info;
    irqrestore(flags);
    lldbg("%s: reg_base:0x%08x\n",__func__,info->reg_base);
    return SUCCESS;

err_irqrestore:
    irqrestore(flags);
err_freemem:
    free(info);
    return ret;
}

static void tsb_spi_dev_remove(struct device *dev)
{
    struct tsb_spi_info *info = dev->private;
    irqstate_t flags;

    lldbg("%s\n",__func__);
    flags = irqsave();
    irq_detach(TSB_IRQ_SPI);
    info->state = TSB_SPI_STATE_INVALID;
    dev->private = NULL;
    irqrestore(flags);
    free(info);
}

static struct device_spi_type_ops tsb_spi_type_ops = {
    .lock           = tsb_spi_lock,
    .select         = tsb_spi_select,
    .setfrequency   = tsb_spi_setfrequency,
    .setmode        = tsb_spi_setmode,
    .setbits        = tsb_spi_setbits,
    .exchange       = tsb_spi_exchange,
    .getcaps        = tsb_spi_getcaps,
};

static struct device_driver_ops tsb_spi_driver_ops = {
    .probe          = tsb_spi_dev_probe,
    .remove         = tsb_spi_dev_remove,
    .open           = tsb_spi_dev_open,
    .close          = tsb_spi_dev_close,
    .type_ops.spi   = &tsb_spi_type_ops,
};

struct device_driver tsb_spi_driver = {
    .type       = DEVICE_TYPE_SPI_HW,
    .name       = "tsb_spi",
    .desc       = "TSB SPI Driver",
    .ops        = &tsb_spi_driver_ops,
};
