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

//#define WORKS_ON_DRIVER 1

struct gb_hid_info {
    /** assigned CPort number */
    uint16_t        cport;

    /** opened device driver handler */
    struct device   *dev;

    /** device descriptor for this device */
    struct gb_hid_desc_response hid_desc;

    /** device type for this device */
    char            *dev_type;

    /** Id for device in device table */
    uint16_t        dev_id;
};

static struct gb_hid_info *hid_info = NULL;

char ReportDescriptor[130] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0,                          // END_COLLECTION
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x02,                    //     REPORT_ID (2)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0x09, 0x01,                    //     USAGE (iName)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
    0xb1, 0x03,                    //     FEATURE (Cnst,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

struct gb_hid_desc_response fake_desc = {
    0x0a,
    0x82,
    0x101,
    0x0416,
    0x192F,
    0x00
};

char fake_keyboard_data [9] = {
    0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00
};

char fake_mouse_data [4] = {
    0x01, 0x00, 0x00, 0x00
};

char fake_dev_feature_data [2] = {
    0x02, 0x00
};

char fake_raw_feature_data [256] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x02, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x03, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x04, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x05, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x06, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x07, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x08, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x09, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x0a, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x0b, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x0c, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x0d, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x0e, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x0f, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x10, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x11, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x12, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x13, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x14, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x15, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x16, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x17, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x18, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x19, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x009,
    0x00, 0x1a, 0x02, 0x03, 0x04, 0x05
};

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
    struct gb_hid_desc_response *response;
    //int ret = 0;

    lldbg("%s()+\n", __func__);

    if (!hid_info || !hid_info->dev) {
        return GB_OP_UNKNOWN_ERROR;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

#ifdef WORKS_ON_DRIVER
    ret = device_hid_get_descriptor(hid_info->dev, response);
    if (ret) {
        gb_info("%s(): %x error in ops\n", __func__, ret);
        return GB_OP_MALFUNCTION;
    }
#else
    // Just retun fake data for verify whithout peripheral drievr
    response->length = fake_desc.length;
    response->report_desc_length = fake_desc.report_desc_length;
    response->hid_version = fake_desc.hid_version;
    response->product_id = fake_desc.product_id;
    response->vendor_id = fake_desc.vendor_id;
    response->country_code = fake_desc.country_code;
#endif

    memcpy(&hid_info->hid_desc, response, response->length);

    lldbg("%s()-\n", __func__);
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
    char *response;
    //int ret =0;

    lldbg("%s()+\n", __func__);

    response = gb_operation_alloc_response(operation,
                                           hid_info->hid_desc.report_desc_length
                                           );
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

#ifdef WORKS_ON_DRIVER
    ret = device_hid_get_report_descriptor(hid_info->dev, response);
    if (ret) {
        gb_info("%s(): %x error in ops\n", __func__, ret);
        return GB_OP_MALFUNCTION;
    }
#else
    // Just retun fake data for verify whithout peripheral drievr
    memcpy(response, &ReportDescriptor, sizeof(ReportDescriptor));
#endif
    lldbg("%s()-\n", __func__);
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
    struct gb_hid_get_report_request *request;
    char *response;

#ifdef WORKS_ON_DRIVER
    int ret = 0;
    uint16_t report_len;
#endif

    lldbg("%s()+\n", __func__);

    request = gb_operation_get_request_payload(operation);
    lldbg("%s():report_type is %d\n", __func__, request->report_type);
    lldbg("%s():report_id is %d\n", __func__, request->report_id);

#ifdef WORKS_ON_DRIVER
    ret = device_hid_get_report_length(hid_info->dev, request->report_type,
                                       request->report_id);
    if (ret <= 0) {
        return GB_OP_MALFUNCTION;
    }

    report_len = ret;

    /**
     * If the report ID is not '0', the report data will include extern one
     * byte date for ID.
     */
    if (request->report_id > 0) {
        report_len += 1;
    }

    response = gb_operation_alloc_response(operation, report_len);
    if (!response) {
            return GB_OP_NO_MEMORY;
    }

    ret = device_hid_get_report(hid_info->dev, request->report_type,
                                request->report_id, response, report_len);
    if (ret) {
        lldbg("%s(): %x error in ops\n", __func__, ret);
        return GB_OP_MALFUNCTION;
    }
#else
    // Just retun fake data for verify whithout peripheral drievr
    if (request->report_id == 1 && request->report_type == 0) {
        // keyboard input report
        response = gb_operation_alloc_response(operation,
                                               sizeof(fake_keyboard_data));
        if (!response) {
            return GB_OP_NO_MEMORY;
        }
        memcpy(response, &fake_keyboard_data, sizeof(fake_keyboard_data));
    } else if (request->report_id == 2 && request->report_type == 0) {
        // mouse input report
        response = gb_operation_alloc_response(operation,
                                               sizeof(fake_mouse_data));
        if (!response) {
            return GB_OP_NO_MEMORY;
        }
        memcpy(response, &fake_mouse_data, sizeof(fake_mouse_data));
    } else if (request->report_id == 2 && request->report_type == 2) {
        // mouse input report
        response = gb_operation_alloc_response(operation,
                                               sizeof(fake_dev_feature_data));
        if (!response) {
            return GB_OP_NO_MEMORY;
        }
        memcpy(response, &fake_dev_feature_data, sizeof(fake_dev_feature_data));
    } else if (request->report_id == 9) {
        // feature input report
        response = gb_operation_alloc_response(operation,
                                               sizeof(fake_raw_feature_data));
        if (!response) {
            return GB_OP_NO_MEMORY;
        }
        memcpy(response, &fake_raw_feature_data, sizeof(fake_raw_feature_data));
    }
    else
        return GB_OP_MALFUNCTION;
#endif //WORKS_ON_DRIVER
    lldbg("%s()-\n", __func__);
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
    struct gb_hid_set_report_request *request;

#ifdef WORKS_ON_DRIVER
    int ret = 0;
    uint16_t report_len;
#endif

    lldbg("%s()+\n", __func__);

    request = gb_operation_get_request_payload(operation);
    lldbg("%s():report_type is %d\n", __func__, request->report_type);
    lldbg("%s():report_id is %d\n", __func__, request->report_id);

#ifdef WORKS_ON_DRIVER
    ret = device_hid_get_report_length(hid_info->dev, request->report_type,
                                       request->report_id);
    if (ret <= 0) {
        return GB_OP_MALFUNCTION;
    }

    report_len = ret;

    ret = device_hid_set_report(hid_info->dev, request->report_type,
                                request->report_id, request->report,
                                report_len);
    if (ret) {
        return GB_OP_MALFUNCTION;
    }
#endif
    lldbg("%s()-\n", __func__);
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
    hid_info = zalloc(sizeof(*hid_info));
    if (!hid_info) {
        return -ENOMEM;
    }

    hid_info->cport = cport;
    hid_info->dev_type = DEVICE_TYPE_HID_HW;
    hid_info->dev_id = 0;

    hid_info->dev = device_open(hid_info->dev_type, hid_info->dev_id);
    if (!hid_info->dev) {
        free(hid_info);
        gb_info("%s(): failed to open HID device!\n", __func__);
        return -EIO;
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
    if (hid_info->dev) {
        device_close(hid_info->dev);
    }

    if (hid_info) {
        free(hid_info);
        hid_info = NULL;
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
