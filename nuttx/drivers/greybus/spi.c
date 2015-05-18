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

struct device *spi_dev = NULL;

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

static uint8_t gb_spi_protocol_mode(struct gb_operation *operation)
{
    struct gb_spi_mode_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    ret = device_spi_getcaps(spi_dev,&caps);
    if (ret != 0) {
        return GB_OP_INVALID;
    }
    response->mode = caps.modes;
    gb_info("%s(): mode 0x%04x\n", __func__, response->mode);
    return GB_OP_SUCCESS;
}

static uint8_t gb_spi_protocol_flags(struct gb_operation *operation)
{
    struct gb_spi_flags_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    ret = device_spi_getcaps(spi_dev,&caps);
    if (ret != 0) {
        return GB_OP_INVALID;
    }
    response->flags = caps.flags;
    gb_info("%s(): flags 0x%04x\n", __func__, response->flags);
    return GB_OP_SUCCESS;
}

static uint8_t gb_spi_protocol_bpw(struct gb_operation *operation)
{
    struct gb_spi_bpw_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    ret = device_spi_getcaps(spi_dev,&caps);
    if (ret) {
        return GB_OP_INVALID;
    }

    response->bits_per_word_mask = caps.bpw;
    gb_info("%s(): bits_per_word_mask 0x%08x\n", __func__,
                                                 response->bits_per_word_mask);
    return GB_OP_SUCCESS;
}

static uint8_t gb_spi_protocol_num_chipselect(struct gb_operation *operation)
{
    struct gb_spi_chipselect_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }
    ret = device_spi_getcaps(spi_dev,&caps);
    if (ret) {
        return GB_OP_INVALID;
    }
    response->num_chipselect = caps.csnum;
    gb_info("%s(): num_chipselect %d\n", __func__, response->num_chipselect);
    return GB_OP_SUCCESS;
}

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

    request = (struct gb_spi_transfer_request *)
                  gb_operation_get_request_payload(operation);
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

    // lock SPI bus
    ret = device_spi_lock(spi_dev, true);
    if (ret) {
        ret = GB_OP_INVALID;
        goto err_alloc;
    }

    ret = device_spi_select(spi_dev, request->chip_select, true);
    if (ret) {
        ret = GB_OP_INVALID;
        goto err_lock;
    }

    ret = device_spi_setmode(spi_dev, request->mode);
    if (ret) {
        ret = GB_OP_INVALID;
        goto err_select;
    }

    for (i = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        freq = desc->speed_hz;

        ret = device_spi_setbits(spi_dev, desc->bits_per_word);
        if (ret) {
            ret = GB_OP_INVALID;
            goto err_select;
        }

        ret = device_spi_setfrequency(spi_dev, &freq);
        if (ret) {
            ret = GB_OP_INVALID;
            goto err_select;
        }
        // setup SPI transfer
        memset(&transfer, 0, sizeof(struct device_spi_transfer));
        transfer.txbuffer = write_data;
        transfer.rxbuffer = read_buf;
        transfer.nwords = desc->len;
        transfer.flags = SPI_FLAG_DMA_TRNSFER; // synchronous & DMA transfer

        ret = device_spi_exchange(spi_dev, &transfer);
        if (ret) {
            ret = GB_OP_INVALID;
            goto err_select;
        }
        write_data += desc->len;
        read_buf += desc->len;

        if (desc->cs_change) {
            ret = device_spi_select(spi_dev, request->chip_select, false);
            if (ret) {
                ret = GB_OP_INVALID;
                goto err_lock;
            }
        }
        usleep(desc->delay_usecs);
    }

    ret = device_spi_select(spi_dev, request->chip_select, false);
    if (ret) {
        ret = GB_OP_INVALID;
        goto err_lock;
    }

    ret = device_spi_lock(spi_dev,false);
    if (ret) {
        ret = GB_OP_INVALID;
        goto err_alloc;
    }

    gb_info("%s() exit\n", __func__);
    return GB_OP_SUCCESS;

err_select:
    device_spi_select(spi_dev, request->chip_select, false);
err_lock:
    device_spi_lock(spi_dev,false);
err_alloc:
    gb_info("%s() exit. fail\n", __func__);
    return ret;
}

static int gb_spi_init(unsigned int cport)
{
    gb_info("%s()\n", __func__);
    if (!spi_dev) {
        spi_dev = device_open(DEVICE_TYPE_SPI_HW, 0);
        if (!spi_dev) {
            return GB_OP_INVALID;
        }
    }
    return 0;
}

static void gb_spi_exit(unsigned int cport)
{
    gb_info("%s()\n", __func__);
    if (spi_dev) {
        device_close(spi_dev);
        spi_dev = NULL;
    }
}

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

void gb_spi_register(int cport)
{
    gb_register_driver(cport, &gb_spi_driver);
}

int gb_spi_set_dev(struct device *dev)
{
    if (!spi_dev)
        spi_dev = dev;
    else
        return -EBUSY;
    return 0;
}

struct device *gb_spi_get_dev(void)
{
    return spi_dev;
}
