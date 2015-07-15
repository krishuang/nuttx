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

static int tsb_hid_get_report_length(struct device *dev, uint8_t report_type,
                                     uint8_t report_id);

#define HID_DEVICE_FLAG_PROBE       BIT(0)
#define HID_DEVICE_FLAG_OPEN        BIT(1)
#define HID_DEVICE_FLAG_POWERON     BIT(2)

/**
 * HID Report Size Structure. Type define for a report item size
 * information structure, to retain the size of a device's reports by ID.
 */
struct hid_size_info {
    /** Report ID */
    uint8_t id;
    /**
     * HID Report length array
     *
     * size[0] : Input Report length
     * size[1] : Output Report length
     * size[2] : Feature Report length
     */
    uint16_t size[3];
};

/**
 * wait queue
 */
struct hid_waitq {
    /** thread exit flag */
    int abort;
    /** condition object for wait event */
    pthread_cond_t cond;
    /** mutex object for wait event */
    pthread_mutex_t mutex;
};

/**
 * Private HID device information
 */
struct tsb_hid_info {
    /** Driver model representation of the device */
    struct device *dev;

    /** HID device descriptor */
    struct hid_descriptor *hdesc;
    /** HID report descriptor */
    uint8_t *rdesc;

    /** number of HID Report structure */
    int num_ids;
    /** report length of each HID Reports */
    struct hid_size_info *sinfo;

    /** multiple report structure support or not */
    int multisupp;
    /** HID device state*/
    int state;

    /** hid input event callback function */
    hid_event_callback event_callback;
    /** Exclusive access for operation */
    sem_t lock;
    /** Exclusive access for data */
    sem_t data;

    /** thread handle */
    pthread_t hid_thread;
    /** wait queue for power on/off event*/
    struct hid_waitq wq;

    /** HID mouse test data index value */
    int data_idx;
    /** HID mouse test data size */
    int data_num;
    /** HID mouse test data report rate (unit: microsecond) */
    int rate;
};

#define VENDORID                0x18D1
#define PRODUCTID               0x1234

#define HID_REPORT_DESC_LEN     52

/**
 * Mouse HID Device Descriptor
 */
struct hid_descriptor hid_dev_desc = {
    0x0A,
    HID_REPORT_DESC_LEN,
    0x0111, /* HID v1.11 compliant */
    PRODUCTID,
    VENDORID,
    0x00, /* no country code */
};

// Input report - 4 bytes
//
// Byte |  D7    D6      D5      D4      D3      D2      D1      D0
// -----+---------------------------------------------------------------------
//  0   |  0     0       0     Extra    Side   Middle   Right   Left (Buttons)
//  1   |                             X (REL)
//  2   |                             Y (REL)
//  3   |                       Vertical Wheel (REL)
//
// Output report - n/a
//
// Feature report - n/a
//

/**
 * Mouse HID report descriptor
 */
uint8_t hid_report_desc[HID_REPORT_DESC_LEN] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x05,                    //     USAGE_MAXIMUM (Button 5)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x95, 0x05,                    //     REPORT_COUNT (5)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x75, 0x03,                    //     REPORT_SIZE (3)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x01,                    //     INPUT (Cnst,Ary,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xC0,                          //   END_COLLECTION
    0xC0                           // END_COLLECTION
};

/**
 * mouse report data
 */
struct hid_mouse_data {
    /** mouse button, bit[0-4] : button0 - button4 */
    uint8_t button;
    /** mouse x axis, range(-127:127) */
    uint8_t x;
    /** mouse y axis, range(-127:127) */
    uint8_t y;
    /** Wheel, range(-127:127) */
    uint8_t wheel;
} __packed;

/**
 * report length of each HID Reports in HID Mouse Report Descriptor
 */
struct hid_size_info hid_sizeinfo[] =
{
    { 0, { 4, 0, 0 } }, /* parsed from HID Report Descriptor manually */
};

/**
 * HID Mouse test data
 */
struct hid_mouse_data testdata[24] =
{
    {0x01, 0x00, 0x00, 0x00},   /* Button0 on */
    {0x00, 0x00, 0x00, 0x00},   /* Button0 off */
    {0x00, 0x02, 0x00, 0x00},   /* X + 2 */
    {0x00, 0x02, 0x00, 0x00},   /* X + 2 */
    {0x00, 0x02, 0x00, 0x00},   /* X + 2 */
    {0x00, 0x02, 0x00, 0x00},   /* X + 2 */
    {0x00, 0x00, 0x00, 0x01},   /* Wheel + 1 */
    {0x00, 0x00, 0x00, 0x00},   /* Wheel + 0 */
    {0x00, 0x00, 0x02, 0x00},   /* Y + 2 */
    {0x00, 0x00, 0x02, 0x00},   /* Y + 2 */
    {0x00, 0x00, 0x02, 0x00},   /* Y + 2 */
    {0x00, 0x00, 0x02, 0x00},   /* Y + 2 */
    {0x00, 0x00, 0x00, 0xFF},   /* Wheel - 1 */
    {0x00, 0x00, 0x00, 0x00},   /* Wheel - 0 */
    {0x00, 0xFE, 0x00, 0x00},   /* X - 2 */
    {0x00, 0xFE, 0x00, 0x00},   /* X - 2 */
    {0x00, 0xFE, 0x00, 0x00},   /* X - 2 */
    {0x00, 0xFE, 0x00, 0x00},   /* X - 2 */
    {0x02, 0x00, 0x00, 0x00},   /* Button1 on */
    {0x00, 0x00, 0x00, 0x00},   /* Button1 off */
    {0x00, 0x00, 0xFE, 0x00},   /* Y - 2 */
    {0x00, 0x00, 0xFE, 0x00},   /* Y - 2 */
    {0x00, 0x00, 0xFE, 0x00},   /* Y - 2 */
    {0x00, 0x00, 0xFE, 0x00}    /* Y - 2 */
};

/**
 * @brief Get HID mouse hardware data
 *
 * Because driver doesn't connect to real mouse hardware, so just reports some
 * fake mouse data to upper layer.
 *
 * @param info pointer to structure of private HID data
 * @param data output data buffer
 * @return 0 on success, negative errno on error
 */
static int get_mouse_data(struct tsb_hid_info *info, struct hid_mouse_data *mouse)
{
    sem_wait(&info->data);
    /* get mouse report data from testdata array*/
    memcpy(mouse, &testdata[info->data_idx], sizeof(struct hid_mouse_data));
    info->data_idx = (info->data_idx + 1) % info->data_num;
    sem_post(&info->data);
    return 0;
}

/**
 * @brief Get HID Input report data
 *
 * @param dev pointer to structure of device data
 * @param report_id HID report id
 * @param data pointer of input buffer size
 * @param len max input buffer size
 * @return 0 on success, negative errno on error
 */
static int get_input_report(struct device *dev, uint8_t report_id,
                            uint8_t *data, uint16_t len)
{
    int ret = 0, rptlen = 0;

    rptlen = tsb_hid_get_report_length(dev, HID_INPUT_REPORT, report_id);

    if (rptlen) {
        if (!report_id) {
            if (len < sizeof(struct hid_mouse_data)) {
                return -EINVAL;
            }
            ret = get_mouse_data(dev->private, (struct hid_mouse_data*)data);
        } else {
            /* For current test case, we don't support multiple Report ID
             * structure, so just returns error code */
            ret = -EIO;
        }
    } else {
        /* Can't find Input Report structure in report descriptor. */
        ret = -EIO;
    }
    return ret;
}

/**
 * @brief Get HID Feature report data
 *
 * @param dev pointer to structure of device data
 * @param report_id HID report id
 * @param data pointer of input buffer size
 * @param len max input buffer size
 * @return 0 on success, negative errno on error
 */
static int get_feature_report(struct device *dev, uint8_t report_id,
                            uint8_t *data, uint16_t len)
{
    int ret = 0, rptlen = 0;

    rptlen = tsb_hid_get_report_length(dev, HID_FEATURE_REPORT, report_id);

    if (rptlen) {
        /* For current test case, HID Report Descriptor doesn't contain a
         * Feature Report structure, so I just put a dummy infrastructure
         * in here and return -EIO error code. */
         ret = -EIO;
    } else {
        /* Can't find Feature Report structure in report descriptor. */
        ret = -EIO;
    }
    return ret;
}

/**
 * @brief Set HID Output report data
 *
 * @param dev pointer to structure of device data
 * @param report_id HID report id
 * @param data pointer of output buffer size
 * @param len max output buffer size
 * @return 0 on success, negative errno on error
 */
static int set_output_report(struct device *dev, uint8_t report_id,
                            uint8_t *data, uint16_t len)
{
    int ret = 0, rptlen = 0;

    rptlen = tsb_hid_get_report_length(dev, HID_OUTPUT_REPORT, report_id);

    if (rptlen) {
        /* For current test case, HID Report Descriptor doesn't contain a
         * Output Report structure, so I just put a dummy infrastructure
         * in here and return -EIO error code. */
         ret = -EIO;
    } else {
        /* Can't find Output Report structure in report descriptor. */
        ret = -EIO;
    }
    return ret;
}

/**
 * @brief Set HID Feature report data
 *
 * @param dev pointer to structure of device data
 * @param report_id HID report id
 * @param data pointer of output buffer size
 * @param len max output buffer size
 * @return 0 on success, negative errno on error
 */
static int set_feature_report(struct device *dev, uint8_t report_id,
                            uint8_t *data, uint16_t len)
{
    int ret = 0, rptlen = 0;

    rptlen = tsb_hid_get_report_length(dev, HID_FEATURE_REPORT, report_id);

    if (rptlen) {
        /* For current test case, HID Report Descriptor doesn't contain a
         * Feature Report structure, so I just put a dummy infrastructure
         * in here and return -EIO error code. */
         ret = -EIO;
    } else {
        /* Can't find Feature Report structure in report descriptor. */
        ret = -EIO;
    }
    return ret;
}

/**
 * @brief HID mouse thread function
 *
 * @param context pointer to structure of device data
 */
void tsb_hid_thread_func(void *context)
{
    struct device *dev = context;
    struct tsb_hid_info *info = NULL;
    struct hid_mouse_data mouse;

    /* check input parameters */
    if (!dev || !dev->private) {
        return;
    }

    info = dev->private;

    pthread_mutex_lock(&info->wq.mutex);

    while (!info->wq.abort) {
        if (!(info->state & HID_DEVICE_FLAG_POWERON)) {
            /* wait for power on */
            pthread_cond_wait(&info->wq.cond, &info->wq.mutex);
            if (info->wq.abort) {
                /* exit tsb_hid_thread_func loop */
                break;
            }
        }

        /* get input report data and send to upper layer */
        get_input_report(dev, 0, (uint8_t*)&mouse,
                         sizeof(struct hid_mouse_data));

        if (info->event_callback) {
            /* for current test case, only supports single report structure */
            if (!info->multisupp) {
                info->event_callback(dev, HID_INPUT_REPORT, (uint8_t*)&mouse,
                                     sizeof(struct hid_mouse_data));
            }
        }
        usleep(info->rate);
    }
    pthread_mutex_unlock(&info->wq.mutex);
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
    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_poweron;
    }
    if (!(info->state & HID_DEVICE_FLAG_POWERON)) {
        info->state |= HID_DEVICE_FLAG_POWERON;
        /* notify thread function, HID device has been powered-on. */
        pthread_cond_signal(&info->wq.cond);
    } else {
        ret = -EBUSY;
    }
err_poweron:
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
    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_poweroff;
    }
    if (info->state & HID_DEVICE_FLAG_POWERON) {
        /* set power-on state to power off state */
        info->state &= ~HID_DEVICE_FLAG_POWERON;
    } else {
        ret = -EIO;
    }
err_poweroff:
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

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }
    /* get HID device descriptor */
    memcpy(desc, info->hdesc, sizeof(struct hid_descriptor));
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

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }

    /* get HID report descriptor */
    memcpy(desc, info->rdesc, info->hdesc->report_desc_length);
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
    if (!dev || !dev->private || (report_type > HID_FEATURE_REPORT)) {
        return -EINVAL;
    }
    info = dev->private;

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }

    /* lookup the hid_size_info table to find the report size */
    for (i = 0; i < info->num_ids; i++) {
        if (info->sinfo[i].id == report_id) {
            ret = info->sinfo[i].size[report_type];
            break;
        }
    }
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
    int i = 0, maxlen = 0, id = 0;

    /* check input parameters */
    if (!dev || !dev->private || (report_type > HID_FEATURE_REPORT)) {
        return -EINVAL;
    }
    info = dev->private;

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }

    /* lookup the hid_size_info table to find the max report size
     * in specific Report type  */

    for (i = 0; i < info->num_ids; i++) {
        if (info->sinfo[i].size[report_type] > maxlen) {
            id = info->sinfo[i].id;
            maxlen = info->sinfo[i].size[report_type];
        }
    }
    /* If the Report ID isn't zero, add a extra 1-byte space to save
     * the Report ID.*/
    if (id != 0) {
        maxlen++;
    }
    return maxlen;
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
    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_getreport;
    }

    switch (report_type) {
        case HID_INPUT_REPORT:
            ret = get_input_report(dev, report_id, data, len);
        break;
        case HID_FEATURE_REPORT:
            ret = get_feature_report(dev, report_id, data, len);
        break;
    }
err_getreport:
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
    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_setreport;
    }

    switch (report_type) {
        case HID_OUTPUT_REPORT:
            ret = set_output_report(dev, report_id, data, len);
        break;
        case HID_FEATURE_REPORT:
            ret = set_feature_report(dev, report_id, data, len);
        break;
    }
err_setreport:
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

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }
    info->event_callback = callback;
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

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }
    info->event_callback = NULL;
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

    if (!(info->state & HID_DEVICE_FLAG_PROBE)) {
        ret = -EIO;
        goto err_open;
    }

    if (info->state & HID_DEVICE_FLAG_OPEN) {
        /* device has been opened, return error */
        ret = -EBUSY;
        goto err_open;
    }

    info->hdesc = &hid_dev_desc;
    info->rdesc = hid_report_desc;
    info->sinfo = hid_sizeinfo;
    info->num_ids = ARRAY_SIZE(hid_sizeinfo);
    /* check whether or not to support multiple report structure */
    if (info->num_ids > 1) {
        info->multisupp = 1;
    }
    info->event_callback = NULL;
    info->state = HID_DEVICE_FLAG_OPEN;

    /* set mouse default value */
    info->data_idx = 0;
    info->data_num = ARRAY_SIZE(testdata);
    info->rate = 1000; /* 1ms */
    /* initialize waitqueue */
    info->wq.abort = 0;
    pthread_mutex_init(&info->wq.mutex, NULL);
    pthread_cond_init(&info->wq.cond, NULL);

    /* create thread to receive HID mouse data */
    if (pthread_create(&info->hid_thread, NULL, (void*)tsb_hid_thread_func,
                       (void*)dev) != 0) {
        ret = -EIO;
    }
err_open:
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

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        goto err_close;
    }

    if (info->state & HID_DEVICE_FLAG_POWERON) {
        tsb_hid_power_off(dev);
    }
    if (info->hid_thread != (pthread_t)0) {
        info->wq.abort = 0;
        pthread_cond_signal(&info->wq.cond);
        /* wait for thread completed */
        pthread_join(info->hid_thread, NULL);
    }

    pthread_cond_destroy(&info->wq.cond);
    pthread_mutex_destroy(&info->wq.mutex);

    info->hdesc = NULL;
    info->rdesc = NULL;
    info->sinfo = NULL;
    info->num_ids = 0;
    info->data_idx = 0;
    info->data_num = 0;
    info->rate = 0;
    info->event_callback = NULL;

err_close:
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
    info->state = HID_DEVICE_FLAG_PROBE;
    dev->private = info;

    sem_init(&info->lock, 0, 1);
    sem_init(&info->data, 0, 1);
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

    if (info->state & HID_DEVICE_FLAG_OPEN) {
        tsb_hid_dev_close(dev);
    }
    info->state = 0;
    sem_destroy(&info->data);
    sem_destroy(&info->lock);

    dev->private = NULL;
    free(info);
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
