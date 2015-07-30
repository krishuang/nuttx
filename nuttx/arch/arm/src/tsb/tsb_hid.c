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

struct hid_descriptor fake_desc = {
    0x0a,
    0x82,
    0x101,
    0x0416,
    0x192F,
    0x00
};

uint8_t fake_keyboard_data [9] = {
    0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00
};

uint8_t fake_mouse_data [4] = {
    0x02, 0x00, 0x00, 0x00
};

uint8_t fake_dev_feature_data [2] = {
    0x02, 0x00
};

uint8_t fake_raw_feature_data [256] = {
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
 * Private HID device information
 */
struct tsb_hid_info {
    /** Driver model representation of the device */
    struct device *dev;

    /** hid input event callback function */
    int (*callback)(struct device *dev, uint8_t report_type, uint8_t *report,
                    uint16_t len);
    /** Exclusive access for operation */
    sem_t lock;

    /** semaphore for notifying data received */
    sem_t               test_start;

    pthread_t           test_thread;
};

static struct device *saved_dev;

static void* tsb_hid_test_thread(void *data)
{
    struct tsb_hid_info *info = saved_dev->private;

    while(1) {
        sem_wait(&info->test_start);
        lldbg("\n");

        usleep(100000);

        if (info->callback) {
            info->callback(info->dev, HID_INPUT_REPORT, fake_keyboard_data,
                           sizeof(fake_keyboard_data));
        }
    }

    return OK;
}

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
    lldbg("\n");
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
    lldbg("\n");
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
    lldbg("\n");
    memcpy(desc, &fake_desc, sizeof(fake_desc));
    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_get_report_desc(struct device *dev, uint8_t *desc)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !desc) {
        return -EINVAL;
    }

    info = dev->private;
    lldbg("() +\n");
    sem_wait(&info->lock);
    memcpy(desc, &ReportDescriptor, sizeof(ReportDescriptor));
    sem_post(&info->lock);
    lldbg("() -\n");
    return ret;
}

static int tsb_hid_get_report_len(struct device *dev, uint8_t report_type,
                                  uint8_t report_id)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

    lldbg("\n");

    sem_wait(&info->lock);
    if (report_type == HID_INPUT_REPORT && report_id == 1) {
        ret = sizeof(fake_keyboard_data);
    } else if (report_type == HID_INPUT_REPORT && report_id == 2) {
        ret = sizeof(fake_mouse_data);
    } else if (report_type == HID_FEATURE_REPORT && report_id == 2) {
        ret = sizeof(fake_dev_feature_data);
    } else if (report_type == HID_FEATURE_REPORT && report_id == 9) {
        ret = sizeof(fake_raw_feature_data);
    } else if (report_type == HID_OUTPUT_REPORT && report_id == 1) {
        ret = 2;
    } else
        ret = -EINVAL;
    sem_post(&info->lock);
    return ret;
}

static int tsb_get_max_report_len(struct device *dev, uint8_t report_type)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    ret = 256;
    sem_post(&info->lock);

    return ret;
}

static int tsb_hid_get_report(struct device *dev, uint8_t report_type,
                              uint8_t report_id, uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !data) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("\n");
    if (report_type == HID_INPUT_REPORT && report_id == 1) {
        memcpy(data, &fake_keyboard_data, len);
    } else if (report_type == HID_INPUT_REPORT && report_id == 2) {
        memcpy(data, &fake_mouse_data, len);
    } else if (report_type == HID_FEATURE_REPORT && report_id == 2) {
        memcpy(data, &fake_dev_feature_data, len);
    } else if (report_type == HID_FEATURE_REPORT && report_id == 9) {
        memcpy(data, &fake_raw_feature_data, len);
    } else
        ret = -EINVAL;

    sem_post(&info->lock);
    return ret;
}

static int tsb_hid_set_report(struct device *dev, uint8_t report_type,
                              uint8_t report_id, uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !data) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("\n");
    sem_post(&info->lock);
    sem_post(&info->test_start);
    return ret;
}

static int tsb_hid_register_callback(struct device *dev,
                                     hid_event_callback callback)
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
    lldbg("\n");

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
    lldbg("\n");

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
    saved_dev = dev;
    dev->private = info;

    sem_init(&info->lock, 0, 1);
    sem_init(&info->test_start, 0, 0);
    pthread_create(&info->test_thread, NULL, tsb_hid_test_thread, NULL);
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
    .get_report_length = tsb_hid_get_report_len,
    .get_maximum_report_length = tsb_get_max_report_len,
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
