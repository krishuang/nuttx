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

/**
 * SPI device state
 */
enum tsb_spi_state {
    TSB_SPI_STATE_INVALID,
    TSB_SPI_STATE_CLOSED,
    TSB_SPI_STATE_OPEN,
    TSB_SPI_STATE_LOCKED,
};


/**
 * struct tsb_spi_info - private SPI device information
 *
 * @param dev: Driver model representation of the device
 * @param reg_base: SPI device base address
 * @param state: SPI device state
 * @param modes: bit masks of supported SPI protocol mode
 * @param flags: bit masks of supported SPI protocol flags
 * @param bpw: number of bits per word supported
 * @param csnum: number of chip select pins supported
 * @param exclsem: Exclusive access for SPI bus
 */
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


/**
 * @brief Lock/unlock SPI bus for exclusive access
 *
 * On SPI buses where there are multiple devices, it will be necessary to lock
 * SPI to have exclusive access to the buses for a sequence of transfers.
 * The bus should be locked before the chip is selected. After locking the SPI
 * bus, the caller should then also call the setfrequency(), setbits() , and
 * setmode() methods to make sure that the SPI is properly configured for the
 * device. If the SPI buses is being shared, then it may have been left in an
 * incompatible state.
 *
 * @param dev pointer to structure of device data
 * @param lock true: To lock SPI bus, false: To unlock SPI bus
 * @retval 0 sussess to lock/unlock SPI bus
 * @retval -EINVAL Parameter is invalid
 */
static int tsb_spi_lock(struct device *dev, bool lock)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    lldbg("%s\n", lock ? "lock":"unlock");

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

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


/**
 * @brief Enable/disable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock(),
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @param selected true: slave selected, false: slave deselected
 * @retval 0 Success to assert/deassert chipselect pin
 * @retval -EINVAL Parameter is invalid
 * @retval -EPERM driver is not in lock state, need to call lock() first
 */
static int tsb_spi_select(struct device *dev, int devid, bool selected)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }

    /* TODO: Implement chip-select code.
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
    lldbg("slave dev %d - selected:%d\n",devid, selected);
    return SUCCESS;
}


/**
 * @brief Configure SPI clock.
 *
 * If SPI hardware doesn’t support this frequency value, this function should
 * find the nearest lower frequency in which hardware supported and then
 * configure SPI clock to this value. It will return the actual frequency
 * selected value back to the caller via parameter frequency.
 * This function should be called after lock(), if the driver is not in lock
 * state, it returns an error code to notify a problem.
 *
 * @param dev pointer to structure of device data
 * @param frequency SPI frequency requested (unit: Hz)
 * @retval 0 Success to set SPI frequency
 * @retval -EINVAL Parameters are invalid
 * @retval -EPERM driver is not in lock state, need to call lock() first
 */
static int tsb_spi_setfrequency(struct device *dev, uint32_t *frequency)
{
    struct tsb_spi_info *info = NULL;
    uint32_t freq = *frequency;

    /* check input parameters */
    if (!dev || !dev->private || !frequency) {
        return -EINVAL;
    }

    info = dev->private;

    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO: Change SPI hardware clock
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
    lldbg("freq:%d\n",freq);
    return SUCCESS;
}


/**
 * @brief Configure SPI mode.
 *
 * To configure SPI configuration such as clock polarity and phase via the mode
 * parameter. Other possible definition of SPI mode can be found in SPI mode
 * definition. If the value of mode parameter is out of SPI mode definition or
 * this mode isn’t supported by the current hardware, this function should
 * return -EOPNOTSUPP error code.
 * This function should be called after lock(), if driver is not in lock state,
 * function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param mode SPI protocol mode requested
 * @retval 0 Success to set SPI mode
 * @retval -EINVAL Parameter is invalid
 * @retval -EPERM driver is not in lock state, need to call lock() first
 * @retval -EOPNOTSUPP mode is not supported
 */
static int tsb_spi_setmode(struct device *dev, uint16_t mode)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO: change SPI mode register
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
    lldbg("mode:%d\n",mode);
    return SUCCESS;
}


/**
 * @brief Set the number of bits per word in transmission.
 *
 * This function should be called after lock(), if driver is not in lock state,
 * this function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param nbits The number of bits requested. The nbits value range is from
 *        1 to 32. The generic nbits value is 8, 16, 32, but this value still
 *        depends on hardware supported.
 * @retval 0 Success to set SPI bits per word setting
 * @retval -EINVAL Parameters are invalid
 * @retval -EPERM driver is not in lock state, need to call lock() first
 * @retval -EOPNOTSUPP The bits per word value is not supported or out of range
 */
static int tsb_spi_setbits(struct device *dev, int nbits)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }
    /* TODO: Implement setbits function
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
    lldbg("nbits:%d\n",nbits);
    return SUCCESS;
}


/**
 * @brief Exchange a block of data from SPI
 *
 * Device driver uses this function to transfer and receive data from SPI bus.
 * This function should be called after lock() , if the driver is not in lock
 * state, it returns -EPERM error code.
 * The transfer structure is consists of the read/write buffer,transfer length,
 * transfer flags and callback function.
 * SPI exchange() function supports synchronous and asynchronous transfer
 * method.
 *
 * For asynchronous SPI transfer mode, the caller sets SPI_FLAG_ASYNC_TRANSFER
 * flag and provides the callback function. This function will issue a SPI
 * transfer command and return immediately without waiting. When the
 * transaction is completed, this function will invoke the callback function
 * to notify the caller the completion.
 *
 * If the caller doesn’t provide the callback function, the exchange() function
 * will ignore callback function, but the caller still can poll the status
 * field to make sure whether the transaction is completed or not.
 * The timeout value can be defined by the caller to specify the maximum time
 * for transaction to be completed. When the timer expired and the transaction
 * is not completed yet, the transaction will be cancelled, the exchange()
 * function also invokes the callback function to notify the caller transaction
 * has been cancelled, and the timeout error code reported in status field.
 *
 * For synchronous SPI transfer mode, the caller doesn’t set
 * SPI_FLAG_ASYNC_TRANSFER flag. The exchange() function issues SPI transfer
 * command and it will be blocked on exchange() function until transaction
 * completed.
 * If the caller sets the timeout value, it will be blocked on exchange()
 * function until the timer expire. When the timer expired, it cancels the
 * current transaction and returns a timeout error code. On synchronous
 * transfer, the caller doesn’t have to set the callback function.
 *
 * The exchange() function also supports DMA transfer. If the caller want to
 * use DMA transfer, it can set SPI_FLAG_DMA_TRNSFER flag. This function will
 * use the DMA mode instead of the PIO mode to transfer, if DMA is not
 * supported by hardware, the exchange() function should fall back to PIO mode.
 *
 * The exchange() function also can support asynchronous and DMA transfer at
 * the same time. Invalid value being set in the flag field will be ignored.
 *
 * @param dev pointer to structure of device data
 * @param transfer pointer to the spi transfer request
 * @retval 0 Success to transfer data
 * @retval -EINVAL Parameters are invalid
 * @retval -EPERM driver is not in lock state, need to call lock() first
 * @retval -ETIMEDOUT synchronous SPI transfer timeout.
 * @retval -EIO SPI transfer error.
 */
static int tsb_spi_exchange(struct device *dev,
                             struct device_spi_transfer *transfer)
{
    struct tsb_spi_info *info = NULL;
    int i = 0;
    uint8_t *txbuf = NULL, *rxbuf = NULL;


    /* check input parameters */
    if (!dev || !dev->private || !transfer) {
        return -EINVAL;
    }

    /* check transfer buffer */
    if(!transfer->txbuffer && !transfer->rxbuffer) {
        return -EINVAL;
    }

    info = dev->private;

    if (info->state != TSB_SPI_STATE_LOCKED) {
        return -EBUSY;
    }

    /* TODO: Implement SPI transfer function
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
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


/**
 * @brief SPI interrupt handler
 *
 * @param irq interrupt number
 * @param context argument for interrupt handler
 * @return 0 if successful, negative error code otherise.
 */
static int tsb_spi_irq_handler(int irq, void *context)
{
    /* TODO: Implement IRQ handler code.
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
    return SUCCESS;
}


/**
 * @brief Get SPI device driver hardware capabilities information.
 *
 * This function can be called whether lock() has been called or not.
 *
 * @param dev pointer to structure of device data
 * @param caps pointer to the spi_caps structure to receive the capabilities
 *             information.
 * @retval 0 sussess to get capabilities
 * @retval -EINVAL Parameters are invalid
 */
static int tsb_spi_getcaps(struct device *dev, struct device_spi_caps *caps)
{
    /* check input parameters */
    if (!dev || !caps) {
        return -EINVAL;
    }

    /* TODO: Add query hardware capabilities code.
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
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


/**
 * @brief Open SPI device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev pointer to structure of device data
 * @retval 0 sussess to open device
 * @retval -EINVAL Parameters are invalid
 * @retval -EBUSY The driver is already opened
 */
static int tsb_spi_dev_open(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    irqstate_t flags;
    int ret = 0;

    lldbg("\n");

    /* check input parameter */
    if (!dev || !dev->private) {
        return -EINVAL;
    }
    info = dev->private;

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


/**
 * @brief Close SPI device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_close(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    irqstate_t flags;

    lldbg("\n");
    /* check input parameter */
    if (!dev || !dev->private) {
        lldbg("invalid parameter\n");
        return;
    }
    info = dev->private;

    flags = irqsave();
    info->state = TSB_SPI_STATE_CLOSED;
    irqrestore(flags);
}


/**
 * @brief Probe SPI device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private SPI device
 * information, and then setup the hardware resource and interrupt handler.
 *
 * @param dev pointer to structure of device data
 * @retval 0 Sussess
 * @retval -EINVAL Parameters are invalid
 * @retval -ENOMEM Memory allocate failed
 * @retval -EIO Failed to register IRQ and interrupt handle
 */
static int tsb_spi_dev_probe(struct device *dev)
{
    struct tsb_spi_info *info;
    struct device_resource *r;
    irqstate_t flags;
    int ret = OK;

    lldbg("\n");
    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }
    /* get register data from resource block */
    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_REGS, "reg_base");
    if (!r) {
        ret = -EINVAL;
        goto err_freemem;
    }

    flags = irqsave();
    /* register SPI IRQ number */
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


/**
 * @brief Remove SPI device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_remove(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    irqstate_t flags;

    lldbg("\n");
    /* check input parameter */
    if (!dev || !dev->private) {
        lldbg("invalid parameter\n");
        return;
    }
    info = dev->private;

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
