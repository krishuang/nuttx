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
#include <nuttx/greybus/types.h>

#define DEVICE_TYPE_HID_HW          "hid"

/* HID Report type */
#define HID_INPUT_REPORT            0 /* Input Report */
#define HID_OUTPUT_REPORT           1 /* Output Report */
#define HID_FEATURE_REPORT          2 /* Feature Report */

/**
 * HID Deivce Descriptor
 */
struct hid_descriptor {
    uint8_t length;
    uint16_t report_desc_length;
    uint16_t hid_version;
    uint16_t product_id;
    uint16_t vendor_id;
    uint8_t country_code;
} __packed;

/**
 * HID report descriptor
 */
struct hid_report_descriptor {
    uint8_t desc[0];
};

/**
 * HID event callback function
 */
typedef int (*hid_event_callback)(struct device *dev, uint8_t report_type,
                                  uint8_t *report, uint16_t len);

/**
 * HID device driver operations
 */
struct device_hid_type_ops {
    int (*power_on)(struct device *dev);
    int (*power_off)(struct device *dev);
    int (*get_descriptor)(struct device *dev, struct hid_descriptor *desc);
    int (*get_report_descriptor)(struct device *dev,
                                 struct hid_report_descriptor *desc);
    int (*get_report_length)(struct device *dev, uint8_t report_type,
                             uint8_t report_id);
    int (*get_maximum_report_length)(struct device *dev, uint8_t report_type);
    int (*get_report)(struct device *dev, uint8_t report_type,
                      uint8_t report_id, uint8_t *data, uint16_t len);
    int (*set_report)(struct device *dev, uint8_t report_type,
                      uint8_t report_id, uint8_t *data, uint16_t len);
    int (*register_callback)(struct device *dev, hid_event_callback callback);
    int (*unregister_callback)(struct device *dev);
};

static inline int device_hid_power_on(struct device *dev)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->power_on) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->power_on(dev);
}

static inline int device_hid_power_off(struct device *dev)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->power_off) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->power_off(dev);
}

static inline int device_hid_get_descriptor(struct device *dev,
                                            struct hid_descriptor *desc)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->get_descriptor) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->get_descriptor(dev, desc);
}

static inline int device_hid_get_report_length(struct device *dev,
                                               uint8_t report_type,
                                               uint8_t report_id)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->get_report_length) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->get_report_length(dev, report_type,
                                                             report_id);
}

static inline int device_hid_get_max_report_length(struct device *dev,
                                                   uint8_t report_type)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->get_maximum_report_length) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->get_maximum_report_length(dev,
                                                             HID_INPUT_REPORT);
}

static inline int device_hid_get_report_descriptor(struct device *dev,
                                                   struct hid_report_descriptor
                                                   *desc)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->get_report_descriptor) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->get_report_descriptor(dev, desc);
}

static inline int device_hid_get_report(struct device *dev,
                                        uint8_t report_type,
                                        uint8_t report_id,
                                        uint8_t *data, uint32_t len)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->get_report) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->get_report(dev, report_type,
                                                      report_id, data, len);
}

static inline int device_hid_set_report(struct device *dev,
                                        uint8_t report_type,
                                        uint8_t report_id,
                                        uint8_t *data, uint32_t len)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->set_report) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->set_report(dev, report_type,
                                                      report_id, data, len);
}

static inline int device_hid_register_callback(struct device *dev,
                                               hid_event_callback callback)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->register_callback) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->register_callback(dev, callback);
}

static inline int device_hid_unregister_callback(struct device *dev)
{
    DEBUGASSERT(dev && dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hid);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (!dev->driver->ops->type_ops.hid->unregister_callback) {
        return -ENOSYS;
    }

    return dev->driver->ops->type_ops.hid->unregister_callback(dev);
}
#endif /* __INCLUDE_NUTTX_DEVICE_HID_H */
