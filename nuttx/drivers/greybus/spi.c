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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/device.h>
#include <nuttx/device_spi.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>

#include "spi-gb.h"

#define GB_SPI_VERSION_MAJOR 0
#define GB_SPI_VERSION_MINOR 1

static struct device *spi_dev = NULL;


/**
 * @brief Returns the major and minor Greybus SPI protocol version number
 *        supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @retval GB_OP_SUCCESS Success
 * @retval GB_OP_NO_MEMORY Failed to allocate memory for response
 */
static uint8_t gb_spi_protocol_version(struct gb_operation *operation)
{
    struct gb_spi_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_SPI_VERSION_MAJOR;
    response->minor = GB_SPI_VERSION_MINOR;
    gb_info("%s(): %d.%d\n", __func__, response->major, response->minor);
    return GB_OP_SUCCESS;
}


/**
 * @brief Returns a bit mask indicating the modes supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @retval GB_OP_SUCCESS Success
 * @retval GB_OP_INVALID Failed to get hardware capabilities
 * @retval GB_OP_NO_MEMORY Failed to allocate memory for response
 */
static uint8_t gb_spi_protocol_mode(struct gb_operation *operation)
{
    struct gb_spi_mode_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get hardware capabilities */
    ret = device_spi_getcaps(spi_dev, &caps);
    if (ret != 0) {
        return GB_OP_INVALID;
    }
    response->mode = caps.modes;
    gb_info("%s(): mode 0x%04x\n", __func__, response->mode);
    return GB_OP_SUCCESS;
}


/**
 * @brief Returns a bit mask indicating the constraints of the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @retval GB_OP_SUCCESS Success
 * @retval GB_OP_INVALID Failed to get hardware capabilities
 * @retval GB_OP_NO_MEMORY Failed to allocate memory for response
 */
static uint8_t gb_spi_protocol_flags(struct gb_operation *operation)
{
    struct gb_spi_flags_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get hardware capabilities */
    ret = device_spi_getcaps(spi_dev, &caps);
    if (ret != 0) {
        return GB_OP_INVALID;
    }
    response->flags = caps.flags;
    gb_info("%s(): flags 0x%04x\n", __func__, response->flags);
    return GB_OP_SUCCESS;
}


/**
 * @brief Returns the number of bits per word supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @retval GB_OP_SUCCESS Success
 * @retval GB_OP_INVALID Failed to get hardware capabilities
 * @retval GB_OP_NO_MEMORY Failed to allocate memory for response
 */
static uint8_t gb_spi_protocol_bpw(struct gb_operation *operation)
{
    struct gb_spi_bpw_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get hardware capabilities */
    ret = device_spi_getcaps(spi_dev, &caps);
    if (ret != 0) {
        return GB_OP_INVALID;
    }

    response->bits_per_word_mask = caps.bpw;
    gb_info("%s(): bits_per_word_mask 0x%08x\n", __func__,
                                                 response->bits_per_word_mask);
    return GB_OP_SUCCESS;
}


/**
 * @brief Returns the number of chip select pins supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @retval GB_OP_SUCCESS Success
 * @retval GB_OP_INVALID Failed to get hardware capabilities
 * @retval GB_OP_NO_MEMORY Failed to allocate memory for response
 */
static uint8_t gb_spi_protocol_num_chipselect(struct gb_operation *operation)
{
    struct gb_spi_chipselect_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get hardware capabilities */
    ret = device_spi_getcaps(spi_dev, &caps);
    if (ret != 0) {
        return GB_OP_INVALID;
    }
    response->num_chipselect = caps.csnum;
    gb_info("%s(): num_chipselect %d\n", __func__, response->num_chipselect);
    return GB_OP_SUCCESS;
}


/**
 * @brief Performs a SPI transaction as one or more SPI transfers, defined
 *        in the supplied array.
 *
 * @param operation pointer to structure of Greybus operation message
 * @retval GB_OP_SUCCESS Success
 * @retval GB_OP_INVALID Invalid SPI operation.
 * @retval GB_OP_NO_MEMORY Failed to allocate memory for response.
 */
static uint8_t gb_spi_protocol_transfer(struct gb_operation *operation)
{
    int i, op_count;
    uint32_t size = 0;
    int ret = 0;
    uint8_t *write_data;
    uint8_t *read_buf;
    uint32_t freq = 0;
    struct device_spi_transfer transfer;

    struct gb_spi_transfer_desc *desc;
    struct gb_spi_transfer_request *request;
    struct gb_spi_transfer_response *response;

    request = gb_operation_get_request_payload(operation);
    op_count = request->count;
    write_data = (uint8_t *)&request->transfers[op_count];

    gb_info("%s() enter\n", __func__);
    for (i = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        size += desc->len;
    }

    response = gb_operation_alloc_response(operation, size);
    if (!response) {
        ret = GB_OP_NO_MEMORY;
        goto err_alloc;
    }
    read_buf = response->data;

    /* lock SPI bus */
    ret = device_spi_lock(spi_dev, true);
    if (ret != 0) {
        ret = GB_OP_INVALID;
        goto err_alloc;
    }

    /* set SPI mode */
    ret = device_spi_setmode(spi_dev, request->mode);
    if (ret != 0) {
        ret = GB_OP_INVALID;
        goto err_lock;
    }

    /* parse all transfer request from AP host side */
    for (i = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        freq = desc->speed_hz;

        /* set SPI bits-per-word */
        ret = device_spi_setbits(spi_dev, desc->bits_per_word);
        if (ret != 0) {
            ret = GB_OP_INVALID;
            goto err_lock;
        }

        /* set SPI clock */
        ret = device_spi_setfrequency(spi_dev, &freq);
        if (ret != 0) {
            ret = GB_OP_INVALID;
            goto err_lock;
        }

        /* assert chip-select pin */
        ret = device_spi_select(spi_dev, request->chip_select, true);
        if (ret != 0) {
            ret = GB_OP_INVALID;
            goto err_select;
        }

        /* setup SPI transfer */
        memset(&transfer, 0, sizeof(struct device_spi_transfer));
        transfer.txbuffer = write_data;
        transfer.rxbuffer = read_buf;
        transfer.nwords = desc->len;
        transfer.flags = SPI_FLAG_DMA_TRNSFER; // synchronous & DMA transfer

        /* start SPI transfer */
        ret = device_spi_exchange(spi_dev, &transfer);
        if (ret != 0) {
            ret = GB_OP_INVALID;
            goto err_select;
        }
        /* move to next gb_spi_transfer data buffer */
        write_data += desc->len;
        read_buf += desc->len;

        if (desc->delay_usecs) {
            usleep(desc->delay_usecs);
        }

        /* if cs_change enable, change the chip-select pin signal */
        if (desc->cs_change) {
            /* force deassert chip-select pin */
            ret = device_spi_select(spi_dev, request->chip_select, false);
            if (ret != 0) {
                ret = GB_OP_INVALID;
                goto err_lock;
            }
        }
    }

    /* deassert chip-select pin */
    ret = device_spi_select(spi_dev, request->chip_select, false);
    if (ret != 0) {
        ret = GB_OP_INVALID;
        goto err_lock;
    }

    /* set SPI unclock */
    ret = device_spi_lock(spi_dev, false);
    if (ret != 0) {
        ret = GB_OP_INVALID;
        goto err_alloc;
    }

    gb_info("%s() exit\n", __func__);
    return GB_OP_SUCCESS;

err_select:
    device_spi_select(spi_dev, request->chip_select, false);
err_lock:
    device_spi_lock(spi_dev, false);
err_alloc:
    gb_info("%s() exit. fail\n", __func__);
    return ret;
}


/**
 * @brief Greybus SPI protocol initialize function
 *
 * @param cport cport number
 * @retval 0 Success
 * @retval -EIO Failed to open SPI device driver
 */
static int gb_spi_init(unsigned int cport)
{
    gb_info("%s()\n", __func__);
    if (!spi_dev) {
        spi_dev = device_open(DEVICE_TYPE_SPI_HW, 0);
        if (!spi_dev) {
            return -EIO;
        }
    }
    return 0;
}


/**
 * @brief Greybus SPI protocol deinitialize function
 *
 * @param cport cport number
 */
static void gb_spi_exit(unsigned int cport)
{
    gb_info("%s()\n", __func__);
    if (spi_dev) {
        device_close(spi_dev);
        spi_dev = NULL;
    }
}


/**
 * @brief Greybus SPI protocol operation handler
 */
static struct gb_operation_handler gb_spi_handlers[] = {
    GB_HANDLER(GB_SPI_PROTOCOL_VERSION, gb_spi_protocol_version),
    GB_HANDLER(GB_SPI_PROTOCOL_MODE, gb_spi_protocol_mode),
    GB_HANDLER(GB_SPI_PROTOCOL_FLAGS, gb_spi_protocol_flags),
    GB_HANDLER(GB_SPI_PROTOCOL_BITS_PER_WORD_MASK, gb_spi_protocol_bpw),
    GB_HANDLER(GB_SPI_PROTOCOL_NUM_CHIPSELECT, gb_spi_protocol_num_chipselect),
    GB_HANDLER(GB_SPI_PROTOCOL_TRANSFER, gb_spi_protocol_transfer),
};


static struct gb_driver gb_spi_driver = {
    .init = gb_spi_init,
    .exit = gb_spi_exit,
    .op_handlers = gb_spi_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_spi_handlers),
};


/**
 * @brief Register Greybus SPI protocol
 *
 * @param cport cport number
 */
void gb_spi_register(int cport)
{
    gb_register_driver(cport, &gb_spi_driver);
}


/**
 * @brief Set SPI device
 *
 * @param device pointer to structure of device data
 * @retval 0 Success
 * @retval -EBUSY device has been assigned.
 */
int gb_spi_set_dev(struct device *dev)
{
    if (!spi_dev)
        spi_dev = dev;
    else
        return -EBUSY;
    return 0;
}


/**
 * @brief Get SPI device
 *
 * @return a device pointer.
 */
struct device *gb_spi_get_dev(void)
{
    return spi_dev;
}
