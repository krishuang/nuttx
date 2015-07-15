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
#include <nuttx/device_hid.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>

#include <arch/byteorder.h>

#include "hid-gb.h"

#define GB_HID_VERSION_MAJOR 0
#define GB_HID_VERSION_MINOR 1

static struct device *hid_dev = NULL;

/**
 * @brief Returns the major and minor Greybus HID protocol version number
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_hid_protocol_version(struct gb_operation *operation)
{
    struct gb_hid_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_HID_VERSION_MAJOR;
    response->minor = GB_HID_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Returns HID Descriptor, that specifies details of the HID device.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_hid_get_descriptor(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/**
 * @brief Returns a HID Report Descriptor
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_hid_get_report_descriptor(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/**
 * @brief Power-on the HID device.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_hid_power_on(struct gb_operation *operation)
{
    /* turn on hid device */
    return GB_OP_SUCCESS;
}

/**
 * @brief Power-off the HID device.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_hid_power_off(struct gb_operation *operation)
{
    /* turn off hid device */
    return GB_OP_SUCCESS;
}

/**
 * @brief Gets input or feature report from device to host synchronously.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_hid_get_report(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/**
 * @brief Sets output or feature report from host to device synchronously.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_hid_set_report(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/**
 * @brief Greybus HID protocol initialize function
 *
 * @param cport CPort number
 * @return 0 on success, negative errno on error
 */
static int gb_hid_init(unsigned int cport)
{
    if (!hid_dev) {
        hid_dev = device_open(DEVICE_TYPE_HID_HW, 0);
        if (!hid_dev) {
            return -EIO;
        }
    }
    return 0;
}

/**
 * @brief Greybus HID protocol deinitialize function
 *
 * @param cport CPort number
 */
static void gb_hid_exit(unsigned int cport)
{
    if (hid_dev) {
        device_close(hid_dev);
        hid_dev = NULL;
    }
}

/**
 * @brief Greybus HID protocol operation handler
 */
static struct gb_operation_handler gb_hid_handlers[] = {
    GB_HANDLER(GB_HID_TYPE_PROTOCOL_VERSION, gb_hid_protocol_version),
    GB_HANDLER(GB_HID_TYPE_GET_DESC, gb_hid_get_descriptor),
    GB_HANDLER(GB_HID_TYPE_GET_REPORT_DESC, gb_hid_get_report_descriptor),
    GB_HANDLER(GB_HID_TYPE_PWR_ON, gb_hid_power_on),
    GB_HANDLER(GB_HID_TYPE_PWR_OFF, gb_hid_power_off),
    GB_HANDLER(GB_HID_TYPE_GET_REPORT, gb_hid_get_report),
    GB_HANDLER(GB_HID_TYPE_SET_REPORT, gb_hid_set_report),
};

static struct gb_driver gb_hid_driver = {
    .init = gb_hid_init,
    .exit = gb_hid_exit,
    .op_handlers = gb_hid_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_hid_handlers),
};

/**
 * @brief Register Greybus HID protocol
 *
 * @param cport CPort number
 */
void gb_hid_register(int cport)
{
    gb_register_driver(cport, &gb_hid_driver);
}
