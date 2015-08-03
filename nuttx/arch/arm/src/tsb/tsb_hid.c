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
#include <pthread.h>

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
 * HID Report Size Structure. Type define for a report item size
 * information structure, to retain the size of a device's reports by ID.
 */
struct hid_size_info {
    uint8_t id;
    uint16_t size[3];
};

/**
 * mouse report information
 */
struct hid_mouse_info {
    uint8_t rid; /**< report id */
    uint8_t button; /**< bit[0-2] : button0 - button2 */
    uint8_t xy[3]; /**< X an Y axis : bit[0-11]: X , bit[12-23]: Y */
    uint8_t wheel; /**< Wheel */
} __packed;

/**
 * wait queue
 */
struct hid_waitq {
    int abort;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
};

/**
 * Private HID device information
 */
struct tsb_hid_info {
    /** Driver model representation of the device */
    struct device *dev;
    struct hid_mouse_info mouse;

    struct hid_descriptor *hdesc;
    uint8_t *rdesc;
    struct hid_size_info *sinfo;
    int num_ids;
    /** multiple report structure support or not */
    int multisupp;
    /** HID device power state, 0:off 1:on */
    int power_state;

    /** hid input event callback function */
    hid_event_callback event_callback;
    /** Exclusive access for operation */
    sem_t lock;

    pthread_t hid_thread;
    struct hid_waitq wq;
};

/**
 * Mouse HID Device Descriptor
 */
struct hid_descriptor hid_dev_desc = {
    0x0A,   /* device descriptor length */
    0x40,   /* report descriptor length */
    0x0111, /* HID version (1.11 compliant) */
    0x4D81, /* product id */
    0x0461, /* vendor id */
    0x00,   /* country code */
};

/**
 * Mouse HID report descriptor
 */
uint8_t hid_report_desc[64] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x01,                    //     INPUT (Cnst,Ary,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x16, 0x01, 0xf8,              //     LOGICAL_MINIMUM (-2047)
    0x26, 0xff, 0x07,              //     LOGICAL_MAXIMUM (2047)
    0x75, 0x0c,                    //     REPORT_SIZE (12)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xc0,                          //     END_COLLECTION
    0xc0                           // END_COLLECTION
};

struct hid_size_info hid_sizeinfo[1] =
{
    { 0, { 5, 0, 0 } },
};

/**
 * @brief HID mouse thread function
 *
 * @param context pointer to structure of device data
 */
void tsb_hid_thread_func(void *context)
{
    struct device *dev = context;
    struct tsb_hid_info *info = NULL;
    struct hid_mouse_info *mouse = NULL;
    uint8_t *data = NULL;
    int len = 0, x = 0, y = 0, step = 2;
    int count = 0, testcase = 0;

    lldbg("start thread...\n");
    /* check input parameters */
    if (!dev || !dev->private) {
        return;
    }

    info = dev->private;

    pthread_mutex_lock(&info->wq.mutex);

    mouse = &info->mouse;
    data = (uint8_t*)mouse;
    len = sizeof(struct hid_mouse_info);

    while(!info->wq.abort) {
        if (!info->power_state) {
            // wait for power on
            lldbg("wait for power on...\n");
            pthread_cond_wait(&info->wq.cond,&info->wq.mutex);
            usleep(1000000);
            lldbg("start to send test data.\n");
        }
#if 0
        /* test case */
        switch (testcase) {
            case 0:
                if (count > 300) {
                    testcase = 1;
                    count = 0;
                }
                x = step;
                y = 0;
            break;
            case 1:
                if (count > 300) {
                    testcase = 2;
                    count = 0;
                }
                x = 0;
                y = step;
            break;
            case 2:
                if (count > 300) {
                    testcase = 3;
                    count = 0;
                }
                x = -1 * step;
                y = 0;
            break;
            case 3:
                if (count > 300) {
                    testcase = 0;
                    count = 0;
                }
                x = 0;
                y = -1 * step;
            break;
        }
        count++;
        /* end of test case */
#else
        if (count > 10) {
            info->wq.abort = 1;
            break;
        }
        x = step;
        y = step;
        count++;
#endif
        mouse->rid = 0; /* for mouse test case, we don't have report id */
        mouse->xy[0] = x & 0xFF;
        mouse->xy[1] = ((x >> 8) & 0x0F) | ((y << 4) & 0xF0);
        mouse->xy[2] = (y >> 4) & 0xFF;
        mouse->button = 0;
        mouse->wheel = 0;

        lldbg("[INPUT] x %d y %d\n", x, y);
        if (info->event_callback) {
            if (info->multisupp) {
                info->event_callback(dev,HID_INPUT_REPORT, data, len);
            } else {
                info->event_callback(dev,HID_INPUT_REPORT, data + 1, len - 1);
            }
        }
        usleep(1);
    }
    pthread_mutex_unlock(&info->wq.mutex);
    lldbg("end thread...\n");
}

/**
 * @brief Power-on the HID device.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
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
    if (!info->power_state) {
        info->power_state = 1;
        pthread_cond_signal(&info->wq.cond);
        lldbg("HID device powered on\n");
    } else {
        ret = -EBUSY;
    }
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Power-off the HID device.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
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
    if (info->power_state) {
        info->power_state = 0;
        lldbg("HID device powered off\n");
    } else {
        ret = -EIO;
    }
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get HID Descriptor
 *
 * @param dev pointer to structure of device data
 * @param desc pointer to structure of HID device descriptor
 * @return 0 on success, negative errno on error
 */
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
    memcpy(desc, info->hdesc, sizeof(struct hid_descriptor));
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get HID Report Descriptor
 *
 * @param dev pointer to structure of device data
 * @param desc pointer to HID report descriptor
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_get_report_desc(struct device *dev, uint8_t *desc)
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
    memcpy(desc, info->rdesc, info->hdesc->report_desc_length);
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get HID report length
 *
 * @param dev pointer to structure of device data
 * @param report_type HID report type
 * @param report_id HID report id
 * @return the report size on success, negative errno on error
 */
static int tsb_hid_get_report_length(struct device *dev, uint8_t report_type,
                                     uint8_t report_id)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0, i;

    /* check input parameters */
    if (!dev || !dev->private || report_type > HID_FEATURE_REPORT) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    for (i = 0; i < info->num_ids; i++) {
        if (info->sinfo[i].id == report_id) {
            ret = info->sinfo[i].size[report_type];
            break;
        }
    }
    lldbg("HID type %u id %u len %d\n", report_type, report_id, ret);
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get HID maximum report size in all Report ID for each Report type
 *
 * @param dev pointer to structure of device data
 * @param report_type HID report type
 * @return the report size on success, negative errno on error
 */
static int tsb_hid_get_maximum_report_length(struct device *dev,
                                             uint8_t report_type)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || report_type > HID_FEATURE_REPORT) {
        return -EINVAL;
    }

    info = dev->private;

    sem_wait(&info->lock);
    lldbg("set HID report\n");
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get HID Input / Feature report data
 *
 * @param dev pointer to structure of device data
 * @param report_type HID report type
 * @param report_id HID report id
 * @param data pointer of input buffer size
 * @param len max input buffer size
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_get_report(struct device *dev, uint8_t report_type,
                              uint8_t report_id, uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !data) {
        return -EINVAL;
    }

    /* only support input and feature report */
    if (report_type != HID_INPUT_REPORT &&
        report_type != HID_FEATURE_REPORT) {
        return -EINVAL;
    }
    info = dev->private;

    sem_wait(&info->lock);
    lldbg("get HID report\n");
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Set HID Output / Feature report data
 *
 * @param dev pointer to structure of device data
 * @param report_type HID report type
 * @param report_id HID report id
 * @param data pointer of output buffer size
 * @param len max output buffer size
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_set_report(struct device *dev, uint8_t report_type,
                              uint8_t report_id, uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !dev->private || !data) {
        return -EINVAL;
    }

    /* only support output and feature report */
    if (report_type != HID_OUTPUT_REPORT &&
        report_type != HID_FEATURE_REPORT) {
        return -EINVAL;
    }
    info = dev->private;

    sem_wait(&info->lock);
    lldbg("set HID report\n");
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Register HID Report notify event
 *
 * @param dev pointer to structure of device data
 * @param callback callback function for notify event
 * @return 0 on success, negative errno on error
 */
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

    info->event_callback = callback;
    lldbg("register hid input report callback function\n");

    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Remove HID Report notify event
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
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

    info->event_callback = NULL;
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
    int ret = 0, i = 0;

    /* check input parameter */
    if (!dev || !dev->private) {
        return -EINVAL;
    }
    info = dev->private;

    sem_wait(&info->lock);
    lldbg("open hid device.\n");

    info->hdesc = &hid_dev_desc;
    info->rdesc = hid_report_desc;
    info->sinfo = hid_sizeinfo;
    info->num_ids = ARRAY_SIZE(hid_sizeinfo);
    info->event_callback = NULL;
    info->power_state = 0; // power off

    /* check has multiple report id support or not */
    for (i=0; i <= info->num_ids; i++) {
        if (info->sinfo[i].id != 0) {
            info->multisupp = 1;
            break;
        }
    }
    /* set mouse default value */
    info->mouse.rid = 0;
    info->mouse.button = 0;
    info->mouse.xy[0] = 0;
    info->mouse.xy[1] = 0;
    info->mouse.xy[2] = 0;
    info->mouse.wheel = 0;

    /* initialize waitqueue */
    info->wq.abort = 0;
    pthread_mutex_init(&info->wq.mutex, NULL);
    pthread_cond_init(&info->wq.cond, NULL);

    if (pthread_create(&info->hid_thread, NULL, (void*)tsb_hid_thread_func,
                       (void*)dev) != 0) {
        ret = -EIO;
    }
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
    struct tsb_hid_info *info = NULL;
    /* check input parameter */
    if (!dev || !dev->private) {
        return;
    }

    info = dev->private;
    sem_wait(&info->lock);
    lldbg("close hid device.\n");

    if (info->hid_thread != (pthread_t)0) {
        info->wq.abort = 0;
        pthread_join(info->hid_thread, NULL);
    }

    pthread_cond_destroy(&info->wq.cond);
    pthread_mutex_destroy(&info->wq.mutex);

    info->hdesc = NULL;
    info->rdesc = NULL;
    info->sinfo = NULL;
    info->num_ids = 0;
    info->event_callback = NULL;

    sem_post(&info->lock);
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
    .get_report_length = tsb_hid_get_report_length,
    .get_maximum_report_length = tsb_hid_get_maximum_report_length,
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
