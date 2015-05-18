/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 for more details.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Google Inc. or Linaro Ltd. nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GOOGLE INC. OR
 * LINARO LTD. BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
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
    uint32_t            bpw;
    uint16_t            csnum;

    sem_t               exclsem;
};

static int tsb_spi_lock(struct device *dev, bool lock) {
    struct tsb_spi_info *info = dev->private;
    int ret = 0;

    lldbg("%s\n", lock ? "lock":"unlock");
    if (lock) {
      /* Take the semaphore (perhaps waiting) */
        ret = sem_wait(&info->exclsem);
        if (ret != OK) {
          /* The sem_wait() call should fail only if we are awakened by
           * a signal.
           */
            return -get_errno();
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
    lldbg("slave dev %d - selected:%d\n",devid, selected);
    return SUCCESS;
}

static int tsb_spi_setfrequency(struct device *dev, uint32_t *frequency) {
    struct tsb_spi_info *info = dev->private;
    uint32_t freq = *frequency;
    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("freq:%d\n",freq);
    return SUCCESS;
}

static int tsb_spi_setmode(struct device *dev, uint16_t mode) {
    struct tsb_spi_info *info = dev->private;
    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("mode:%d\n",mode);
    return SUCCESS;
}

static int tsb_spi_setbits(struct device *dev, int nbits) {
    struct tsb_spi_info *info = dev->private;
    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("nbits:%d\n",nbits);
    return SUCCESS;
}

static int tsb_spi_exchange(struct device *dev,
                             struct device_spi_transfer *transfer) {
    struct tsb_spi_info *info = dev->private;
    int i = 0;
    uint8_t *txbuf, *rxbuf;

    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO */
    lldbg("xfer len %d %s %s\n",transfer->nwords,
                                transfer->txbuffer ? "tx" : "",
                                transfer->txbuffer ? "rx" : "");

    // for test only
    txbuf = transfer->txbuffer;
    rxbuf = transfer->rxbuffer;
    for (i=0; i<transfer->nwords; i++) {
        if (txbuf && rxbuf) {
            rxbuf[i] = ~txbuf[i];
        } else if (rxbuf) {
            rxbuf[i] = (uint8_t)i;
        }
    }
    return SUCCESS;
}

static int tsb_spi_irq_handler(int irq, void *context)
{
    /* TODO */
    return SUCCESS;
}

static int tsb_spi_getcaps(struct device *dev, struct device_spi_caps *caps) {
    /* TODO */
    caps->modes = SPI_MODE_CPHA |
                  SPI_MODE_CPOL |
                  SPI_MODE_CS_HIGH |
                  SPI_MODE_LSB_FIRST |
                  SPI_MODE_LOOP;
    caps->flags = 0;
    caps->bpw = BIT(8-1) | BIT(16-1) | BIT(32-1);
    caps->csnum = 1;
    lldbg("mode: 0x%x\n flags: 0x%x bpw: 0x%x\n csnum: %d\n",caps->modes,
                                                             caps->flags,
                                                             caps->bpw,
                                                             caps->csnum);
    return SUCCESS;
}

static int tsb_spi_dev_open(struct device *dev)
{
    struct tsb_spi_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    lldbg("\n");
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

    lldbg("\n");
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

    lldbg("\n");
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
    sem_init(&info->exclsem, 0, 1);
    irqrestore(flags);
    lldbg("reg_base: 0x%08x\n",info->reg_base);
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

    lldbg("\n");
    flags = irqsave();
    irq_detach(TSB_IRQ_SPI);
    info->state = TSB_SPI_STATE_INVALID;
    sem_destroy(&info->exclsem);
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
