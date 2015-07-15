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

#ifndef __INCLUDE_NUTTX_DEVICE_HID_H
#define __INCLUDE_NUTTX_DEVICE_HID_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_HID_HW          "hid"

/**
 * HID Deivce Descriptor
 */
struct hid_descriptor {
    uint8_t bLength;
    uint16_t wReportDescLength;
    uint16_t bcdHID;
    uint16_t wProductID;
    uint16_t wVendorID;
    uint16_t wVersionID;
    uint8_t bCountryCode;
} __packed;

/**
 * HID report descriptor
 */
struct hid_report_descriptor {
    uint8_t report[0];
};

/**
 * HID device driver operations
 */
struct device_hid_type_ops {
    int (*power_on)(struct device *dev);
    int (*power_off)(struct device *dev);
    int (*get_descriptor)(struct device *dev, struct hid_descriptor *desc);
    int (*get_report_descriptor)(struct device *dev,
                                 struct hid_report_descriptor *desc);
    int (*get_report)(struct device *dev, uint8_t report_type,
                      uint8_t report_id, uint8_t *data, uint32_t len);
    int (*set_report)(struct device *dev, uint8_t report_type,
                      uint8_t report_id, uint8_t *data, uint32_t len);
    int (*register_callback)(struct device *dev,
                             int (*callback)(void *context));
    int (*unregister_callback)(struct device *dev);
};

#endif /* __INCLUDE_NUTTX_DEVICE_HID_H */
