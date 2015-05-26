/*
 * Copyright (c) 2015 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_ARM_DEVICE_SPI_H
#define __ARCH_ARM_DEVICE_SPI_H

#include <stdint.h>
#include <stdbool.h>

#define DEVICE_TYPE_SPI_HW          "spi"

#define SPI_FLAG_ASYNC_TRANSFER     0x01        /* non-blocking transfer */
#define SPI_FLAG_DMA_TRNSFER        0x02        /* DMA transfer */

/* SPI mode definition */
#define SPI_MODE_CPHA               0x01        /* clock phase */
#define SPI_MODE_CPOL               0x02        /* clock polarity */
#define SPI_MODE_CS_HIGH            0x04        /* chipselect active high */
#define SPI_MODE_LSB_FIRST          0x08        /* per-word bits-on-wire */
#define SPI_MODE_3WIRE              0x10        /* SI/SO signals shared */
#define SPI_MODE_LOOP               0x20        /* loopback mode */
#define SPI_MODE_NO_CS              0x40        /* 1 dev/bus, no chipselect */
#define SPI_MODE_READY              0x80        /* slave pulls low to pause */

#define SPI_MODE_0                  (0 | 0)     /* (original MicroWire) */
#define SPI_MODE_1                  (0 | SPI_MODE_CPHA)
#define SPI_MODE_2                  (SPI_MODE_CPOL | 0)
#define SPI_MODE_3                  (SPI_MODE_CPOL | SPI_MODE_CPHA)

/* SPI Flag */
#define SPI_FLAG_HALF_DUPLEX        0x0001      /* can’t do full duplex */
#define SPI_FLAG_NO_RX              0x0002      /* can’t do buffer read */
#define SPI_FLAG_NO_TX              0x0004      /* can’t do buffer write */

/* error code */
#define SUCCESS                     0


/**
 * struct device_spi_transfer - SPI a read/write buffer pair
 *
 * @param txbuffer: Data to be written, or NULL
 * @param rxbuffer: Data to be read, or NULL
 * @param nwords: Size of rx and tx buffers
 * @param flags: SPI transfer mode
 * @param timeout: Timeout value (milliseconds) for SPI transfer. Timeout value
 *                 must larger then 0, timeout <=0 means infinite timeout
 * @param complete: Called to report transaction completion, only for
 *                  asynchronous SPI transfer
 * @param context: The argument to complete() function when it's called
 * @param status: Return code for asynchronous SPI transfer
 */
struct device_spi_transfer {
    void *txbuffer;
    void *rxbuffer;
    size_t nwords;
    uint16_t flags;
    int timeout;
    /* completion notify via callback */
    void (*complete)(void *context);
    void *context;
    int status;
};


/**
 * struct device_spi_caps - SPI hardware capabilities info
 *
 * @param modes: bit masks of supported SPI protocol mode
 * @param flags: bit masks of supported SPI protocol flags
 * @param bpw: number of bits per word supported
 * @param csnum: number of chip select pins supported
 */
struct device_spi_caps {
    uint16_t modes;
    uint16_t flags;
    uint32_t bpw;
    uint16_t csnum;
};


/**
 * struct device_spi_type_ops - SPI device driver operations
 *
 * @param lock: SPI lock() function pointer
 * @param unlock: SPI unlock() function pointer
 * @param select: SPI select() function pointer
 * @param setfrequency: SPI setfrequency() function pointer
 * @param setmode: SPI setmode() function pointer
 * @param setbits: SPI setbits() function pointer
 * @param exchange: SPI exchange() function pointer
 * @param getcaps: SPI getcaps() function pointer
 */
struct device_spi_type_ops {
    int (*lock)(struct device *dev);
    int (*unlock)(struct device *dev);
    int (*select)(struct device *dev, int devid, bool selected);
    int (*setfrequency)(struct device *dev, uint32_t *frequency);
    int (*setmode)(struct device *dev, uint16_t mode);
    int (*setbits)(struct device *dev, int nbits);
    int (*exchange)(struct device *dev, struct device_spi_transfer *transfer);
    int (*getcaps)(struct device *dev, struct device_spi_caps *caps);
};


/**
 * @brief SPI lock wrap function
 */
static inline int device_spi_lock(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }
    if (dev->driver->ops->type_ops.spi->lock) {
        return dev->driver->ops->type_ops.spi->lock(dev);
    }
    return -EOPNOTSUPP;
}


/**
 * @brief SPI unlock wrap function
 */
static inline int device_spi_unlock(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }
    if (dev->driver->ops->type_ops.spi->unlock) {
        return dev->driver->ops->type_ops.spi->unlock(dev);
    }
    return -EOPNOTSUPP;
}


/**
 * @brief SPI select wrap function
 */
static inline int device_spi_select(struct device *dev,
                                    int devid,
                                    bool selected)
{
    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }
    if (dev->driver->ops->type_ops.spi->select) {
        return dev->driver->ops->type_ops.spi->select(dev, devid, selected);
    }
    return -EOPNOTSUPP;
}


/**
 * @brief SPI setfrequency wrap function
 */
static inline int device_spi_setfrequency(struct device *dev,
                                          uint32_t *frequency)
{
    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }
    if (dev->driver->ops->type_ops.spi->setfrequency) {
        return dev->driver->ops->type_ops.spi->setfrequency(dev, frequency);
    }
    return -EOPNOTSUPP;
}


/**
 * @brief SPI setmode wrap function
 */
static inline int device_spi_setmode(struct device *dev, uint16_t mode)
{
    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }
    if (dev->driver->ops->type_ops.spi->setmode) {
        return dev->driver->ops->type_ops.spi->setmode(dev, mode);
    }
    return -EOPNOTSUPP;
}


/**
 * @brief SPI setbits wrap function
 */
static inline int device_spi_setbits(struct device *dev, int nbits)
{
    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }
    if (dev->driver->ops->type_ops.spi->setbits) {
        return dev->driver->ops->type_ops.spi->setbits(dev, nbits);
    }
    return -EOPNOTSUPP;
}


/**
 * @brief SPI exchange wrap function
 */
static inline int device_spi_exchange(struct device *dev,
                                      struct device_spi_transfer *transfer)
{
    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }
    if (dev->driver->ops->type_ops.spi->exchange) {
        return dev->driver->ops->type_ops.spi->exchange(dev, transfer);
    }
    return -EOPNOTSUPP;
}


/**
 * @brief SPI getcaps wrap function
 */
static inline int device_spi_getcaps(struct device *dev,
                                     struct device_spi_caps *caps)
{
    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }
    if (dev->driver->ops->type_ops.spi->getcaps) {
        return dev->driver->ops->type_ops.spi->getcaps(dev, caps);
    }
    return -EOPNOTSUPP;
}

#endif /* __ARCH_ARM_DEVICE_SPI_H */
