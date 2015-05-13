#ifndef __ARCH_ARM_DEVICE_SPI_H
#define __ARCH_ARM_DEVICE_SPI_H

#include <stdint.h>
#include <stdbool.h>

#define DEVICE_TYPE_SPI_HW          "spi"

#define SPI_FLAG_ASYNC_TRANSFER     0x01
#define SPI_FLAG_DMA_TRNSFER        0x02

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

struct device_spi_caps {
    uint16_t modes;     /* bit masks of supported SPI protocol mode */
    uint16_t flags;     /* bit masks of supported SPI protocol flags */
    uint16_t bpw;       /* number of bits per word supported */
    uint16_t csnum;     /* number of chip select pins supported */
};

struct device_spi_type_ops {
    int (*lock)(struct device *dev, bool lock);
    int (*select)(struct device *dev, int devid, bool selected);
    int (*setfrequency)(struct device *dev, uint32_t *frequency);
    int (*setmode)(struct device *dev, uint16_t mode);
    int (*setbits)(struct device *dev, int nbits);
    int (*exchange)(struct device *dev, struct device_spi_transfer *transfer);
    int (*getcaps)(struct device *dev, struct device_spi_caps *caps);
};

#endif /* __ARCH_ARM_DEVICE_SPI_H */
