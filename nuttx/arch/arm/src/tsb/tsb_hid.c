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

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_hid.h>
#include <nuttx/gpio.h>

#include <arch/tsb/chip.h>
#include <tsb_scm.h>

/**
 * Private HID device information
 */
struct tsb_hid_info {
    /** Driver model representation of the device */
    struct device *dev;

    /** hid input event callback function */
    int (*callback)(void *context);
    /** Exclusive access for operation */
    sem_t lock;
};

static int tsb_hid_power_on(struct device *dev)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("HID device powered on\n");
    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_power_off(struct device *dev)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("HID device powered off\n");
    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_get_desc(struct device *dev, struct hid_descriptor *desc)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !desc) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("get HID descriptor\n");
    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_get_report_desc(struct device *dev,
                                   struct hid_report_descriptor *desc)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !desc) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("get HID report descriptor\n");
    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_get_report(struct device *dev, uint8_t report_type,
                              uint8_t report_id, uint8_t *data, uint32_t len)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !data) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("get HID report\n");
    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_set_report(struct device *dev, uint8_t report_type,
                              uint8_t report_id, uint8_t *data, uint32_t len)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !data) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("set HID report\n");
    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_register_callback(struct device *dev,
                                     int (*callback)(void *context))
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !callback) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);

    info->callback = callback;
    lldbg("register hid input report callback function\n");

    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_unregister_callback(struct device *dev)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);

    info->callback = NULL;
    lldbg("unregister callback function\n");

    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Open HID device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_dev_open(struct device *dev)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameter */
    if (!dev || !dev->private) {
        return -EINVAL;
    }
    info = dev->private;

    sem_wait(&info->lock);
    lldbg("open hid device.\n");
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Close HID device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_hid_dev_close(struct device *dev)
{
    /* check input parameter */
    if (!dev || !dev->private) {
        return;
    }
    lldbg("close hid device.\n");
}

/**
 * @brief Probe HID device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private SPI device
 * information, and then setup the hardware resource and interrupt handler.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_dev_probe(struct device *dev)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    dev->private = info;

    sem_init(&info->lock, 0, 1);
    lldbg("probe hid device.\n");
    return ret;
}

/**
 * @brief Remove HID device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_hid_dev_remove(struct device *dev)
{
    struct tsb_hid_info *info = NULL;

    /* check input parameter */
    if (!dev || !dev->private) {
        return;
    }
    info = dev->private;

    sem_destroy(&info->lock);

    dev->private = NULL;
    free(info);
    lldbg("remove hid device.\n");
}

static struct device_hid_type_ops tsb_hid_type_ops = {
    .power_on = tsb_hid_power_on,
    .power_off = tsb_hid_power_off,
    .get_descriptor = tsb_hid_get_desc,
    .get_report_descriptor = tsb_hid_get_report_desc,
    .get_report = tsb_hid_get_report,
    .set_report = tsb_hid_set_report,
    .register_callback = tsb_hid_register_callback,
    .unregister_callback = tsb_hid_unregister_callback,
};

static struct device_driver_ops tsb_hid_driver_ops = {
    .probe          = tsb_hid_dev_probe,
    .remove         = tsb_hid_dev_remove,
    .open           = tsb_hid_dev_open,
    .close          = tsb_hid_dev_close,
    .type_ops.hid   = &tsb_hid_type_ops,
};

struct device_driver tsb_hid_driver = {
    .type       = DEVICE_TYPE_HID_HW,
    .name       = "tsb_hid",
    .desc       = "TSB HID Driver",
    .ops        = &tsb_hid_driver_ops,
};
