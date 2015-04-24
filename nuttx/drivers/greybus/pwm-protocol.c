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
#include <string.h>
#include <stdlib.h>

#include <nuttx/device_pwm.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/list.h>
#include <apps/greybus-utils/utils.h>

#include "pwm-gb.h"

#define GB_PWM_VERSIONS_TBL 1

static struct pwm_generators *gb_get_pwm_handler(uint8_t which);
/*
 * The version should fill as little-endian
 * {major, minor}
 */

struct version_table pwm_version_tbl[GB_PWM_VERSIONS_TBL] = {
    {0,1}
};

/* This link listis to store PWMx instance that returned by device driver which
 * be called when "Activate" protocol arrived.
 */

struct list_head pwm_list = {
    .prev = &pwm_list,
    .next = &pwm_list,
};

static uint8_t gb_pwm_protocol_version(struct gb_operation *operation) {
    struct gb_pwm_version_response *response;
    struct gb_pwm_version_request *request;
    uint8_t find_major = 0;
    uint8_t find_minor = 0;
    int i;

    request = gb_operation_get_request_payload(operation);

    response = gb_operation_alloc_response(operation, sizeof(*response));

    if (!response)
        return GB_OP_NO_MEMORY;

    for (i = 0; i < GB_PWM_VERSIONS_TBL; i++) {
        if (request->offer_major > pwm_version_tbl[i].major) {
            /* go here means major still less than request, so find
             * next version in table.
             */
                find_major = pwm_version_tbl[i].major;
                find_minor = pwm_version_tbl[i].minor;
                continue;
        } else if (request->offer_major == pwm_version_tbl[i].major) {
            if(request->offer_minor == pwm_version_tbl[i].minor) {
                /* go here means both major and minor are matched */
                find_major = pwm_version_tbl[i].major;
                find_minor = pwm_version_tbl[i].minor;
                goto return_version;
            } else if (request->offer_minor < pwm_version_tbl[i].minor) {
                /* go here means major is matched but minor is big than
                 * request, so protocol interface need to return
                 * previouly lower version that it supported to caller
                 */
                goto return_version;
            }
            /* go here means major match but minor still less
             * than request, so find next version in table
             */
            find_major = pwm_version_tbl[i].major;
            find_minor = pwm_version_tbl[i].minor;
            continue;
        } else if (request->offer_major < pwm_version_tbl[i].major) {
            /* go here means major already big than request major,
             * so the protocol interface need to retun previously
             * lower version that it supported to caller
             */
            goto return_version;
        }
    }

return_version:
    response->major = find_major;
    response->minor = find_minor;
    return GB_OP_SUCCESS;
}

static uint8_t gb_pwm_protocol_count(struct gb_operation *operation) {
    struct gb_pwm_count_response *response;
    uint16_t count = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    count = pwm_count();

    if (count == 0 || count > 256)
        return GB_OP_INVALID;

    response->count = (uint8_t)count - 1;

    return GB_OP_SUCCESS;
}

static uint8_t gb_pwm_protocol_activate(struct gb_operation *operation)
{
    struct gb_pwm_activate_request *request;
    struct pwm_generators *dev_handler = NULL;

    request = gb_operation_get_request_payload(operation);
    if (request->which >= pwm_count())
        return GB_OP_INVALID;

    dev_handler = malloc(sizeof(struct pwm_generators));
    if (!dev_handler)
        return GB_OP_NO_MEMORY;

    dev_handler->dev = pwm_initialize(request->which);

    if (!dev_handler->dev) {
        gb_info("%s(): activate error in ops return", __func__);
        free(dev_handler);
        return GB_OP_INVALID;
    }

    /* Store pwm number for deactivate */
    dev_handler->which = request->which;

    list_add(&pwm_list, &dev_handler->list);

    return GB_OP_SUCCESS;
}


static uint8_t gb_pwm_protocol_deactivate(struct gb_operation *operation) {
    struct gb_pwm_activate_request *request;
    struct pwm_generators *dev_handler = NULL;

    request = gb_operation_get_request_payload(operation);
    if (request->which >= pwm_count())
        return GB_OP_INVALID;

    dev_handler = gb_get_pwm_handler(request->which);

    if (!dev_handler) {
        gb_info("%s() pwm%d handler not found!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    if (!dev_handler->dev->ops->deactivate(dev_handler->dev)) {
        gb_info("%s() deactivate pwm%d fail!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    list_del(&dev_handler->list);

    free(dev_handler);

    return GB_OP_SUCCESS;
}

static uint8_t gb_pwm_protocol_config(struct gb_operation *operation) {
    struct gb_pwm_config_request *request;
    struct pwm_generators *dev_handler = NULL;

    request = gb_operation_get_request_payload(operation);
    if (request->which >= pwm_count())
        return GB_OP_INVALID;

    dev_handler = gb_get_pwm_handler(request->which);

    if (!dev_handler) {
        gb_info("%s() pwm%d handler not found!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    if (!dev_handler->dev->ops->config(dev_handler->dev, request->duty_cycle,
                                       request->period)) {
        gb_info("%s() config pwm%d fail!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_pwm_protocol_setpolarity(struct gb_operation *operation)
{
    struct gb_pwm_polarity_request *request;
    struct pwm_generators *dev_handler = NULL;

    request = gb_operation_get_request_payload(operation);
    if (request->which >= pwm_count())
        return GB_OP_INVALID;

    dev_handler = gb_get_pwm_handler(request->which);

    if (!dev_handler) {
        gb_info("%s() pwm%d handler not found!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    if (!dev_handler->dev->ops->setpolarity(dev_handler->dev,
                                            request->polarity)) {
        gb_info("%s() set polarity pwm%d fail!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_pwm_protocol_enable(struct gb_operation *operation)
{
    struct gb_pwm_enable_request *request;
    struct pwm_generators *dev_handler = NULL;

    request = gb_operation_get_request_payload(operation);
    if (request->which >= pwm_count())
        return GB_OP_INVALID;

    dev_handler = gb_get_pwm_handler(request->which);

    if (!dev_handler) {
        gb_info("%s() pwm%d handler not found!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    if (!dev_handler->dev->ops->enable(dev_handler->dev)) {
        gb_info("%s() enable pwm%d fail!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_pwm_protocol_disable(struct gb_operation *operation)
{
    struct gb_pwm_disable_request *request;
    struct pwm_generators *dev_handler = NULL;

    request = gb_operation_get_request_payload(operation);
    if (request->which >= pwm_count())
        return GB_OP_INVALID;

    dev_handler = gb_get_pwm_handler(request->which);

    if (!dev_handler) {
        gb_info("%s() pwm%d handler not found!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    if (!dev_handler->dev->ops->disable(dev_handler->dev)) {
        gb_info("%s() disable pwm%d fail!!\n", __func__, request->which);
        return GB_OP_INVALID;
    }

    return GB_OP_SUCCESS;
}

static struct pwm_generators *gb_get_pwm_handler(uint8_t which)
{
    struct pwm_generators *dev_handler = NULL;
    struct list_head *iter;

    list_foreach(&pwm_list, iter) {
        dev_handler = list_entry(iter, struct pwm_generators, list);
        if (dev_handler->which == which) {
            gb_info("%s() found pwm0 handler 0x%x!!\n", __func__,dev_handler);
            gb_info("%s() found pwm0 ops 0x%x!!\n", __func__,dev_handler->dev);
            return dev_handler;
        }
    }
    return NULL;
}

static int gb_pwm_init(unsigned int cport)
{
#if 0 /* PWM Count test code */

    uint16_t count = 0;

    gb_info("%s()\n", __func__);

    count = pwm_count();

    if(count == 0 || count > 256)
        gb_info("%s(): get pwm error, count = %d\n", __func__, count);
    else
        gb_info("%s(): get pwm count = %d\n", __func__, count - 1);
#endif

#if 0 /* Activate test code */

    struct pwm_generators *dev_handler = NULL;
    struct list_head *iter;
    int i;

    gb_info("%s()Test Activate--------------------\n", __func__);

    for (i = 0; i < 2; i++) {
        dev_handler = malloc(sizeof(struct pwm_generators));
        if (!dev_handler) {
            gb_info("%s(): activate error in memnoy alloc", __func__);
            return GB_OP_NO_MEMORY;
        }
        gb_info("%s(): pwm%d handler pointer = 0x%x\n", __func__, i, dev_handler);

        dev_handler->dev = pwm_initialize(i);
        if(!dev_handler->dev) {
            gb_info("%s(): activate error return", __func__);
            free(dev_handler);
            return GB_OP_INVALID;
        }
        else
            gb_info("%s(): activate success get pwm%d ops = 0x%x\n",
                    __func__, i, dev_handler->dev);

        /* Store pwm number for deactivate */
        dev_handler->which = i;

        list_add(&pwm_list, &dev_handler->list);
    }

    list_foreach(&pwm_list, iter) {
        dev_handler = list_entry(iter, struct pwm_generators, list);
        gb_info("%s() pwm%d of handler 0x%x!!\n", __func__, dev_handler->which, dev_handler);
    }
#endif

#if 0 /* Deactivate test code */

    gb_info("%s() Test Deactivate-------------------\n", __func__);

    dev_handler = gb_get_pwm_handler(0);

    if(!dev_handler)
        gb_info("%s() pwm0 handler not found!!\n", __func__);
    else  {
        if(dev_handler->dev->ops->deactivate(dev_handler->dev)) {
            gb_info("%s() deactivate pwm%d fail!!\n", __func__, 0);
            return GB_OP_INVALID;
        }
        list_del(&dev_handler->list);
        free(dev_handler);
    }

    list_foreach(&pwm_list, iter) {
        dev_handler = list_entry(iter, struct pwm_generators, list);
        if (dev_handler->which == 0) {
            gb_info("%s() pwm0 found matched handler!!\n", __func__);
            break;
        }
    }

    list_foreach(&pwm_list, iter) {
        dev_handler = list_entry(iter, struct pwm_generators, list);
        if (dev_handler->which == 1) {
            gb_info("%s() pwm1 found matched handler!!\n", __func__);
            break;
        }
    }
#endif
    return 0;
}

static struct gb_operation_handler gb_pwm_handlers[] = {
    GB_HANDLER(GB_PWM_PROTOCOL_VERSION, gb_pwm_protocol_version),
    GB_HANDLER(GB_PWM_PROTOCOL_COUNT, gb_pwm_protocol_count),
    GB_HANDLER(GB_PWM_PROTOCOL_ACTIVATE, gb_pwm_protocol_activate),
    GB_HANDLER(GB_PWM_PROTOCOL_DEACTIVATE, gb_pwm_protocol_deactivate),
    GB_HANDLER(GB_PWM_PROTOCOL_CONFIG, gb_pwm_protocol_config),
    GB_HANDLER(GB_PWM_PROTOCOL_POLARITY, gb_pwm_protocol_setpolarity),
    GB_HANDLER(GB_PWM_PROTOCOL_ENABLE, gb_pwm_protocol_enable),
    GB_HANDLER(GB_PWM_PROTOCOL_DISABLE, gb_pwm_protocol_disable),
};

static struct gb_driver gb_pwm_driver = {
    .init = gb_pwm_init,
    .op_handlers = gb_pwm_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_pwm_handlers),
};

void gb_pwm_register(int cport)
{
    gb_register_driver(cport, &gb_pwm_driver);
}
