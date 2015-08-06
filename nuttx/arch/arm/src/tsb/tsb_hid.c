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
#include <nuttx/device.h>
#include <nuttx/device_hid.h>
#include <nuttx/gpio.h>
#include <nuttx/clock.h>

#include <arch/tsb/chip.h>
#include <tsb_scm.h>
#include "clock/clock.h"
static int tsb_hid_get_report_length(struct device *dev, uint8_t report_type,
                                     uint8_t report_id);

#define HID_DEVICE_FLAG_PROBE       BIT(0)
#define HID_DEVICE_FLAG_OPEN        BIT(1)
#define HID_DEVICE_FLAG_POWERON     BIT(2)

static struct device *hid_dev = NULL;

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

    /** store latest valid keyboard interrupt time */
    struct timespec last_activetime;
    /** store latest valid keyboard state */
    uint8_t last_keystate;
    /**
     * GPIO debounce time count
     *
     * Since cuurent SysTick resolution is 10 milliseconds, so the
     * debounce_count unit is 10 milliseconds.
     */
    int debounce_count;
    /**
     * default modifier key
     *
     * For 1-key HID device, device need to report modifier + keycode data.
     * bit[0-4] : Num Lock, Caps Lock, Scroll Lock, Compose, KANA
     */
    uint8_t modifier;
    /**
     * default keycode value
     *
     * For 1-key HID device, device need to report modifier + keycode data.
     * keycode: 0 ~ 101
     */
    uint8_t keycode;
};

#define VENDORID                0x18D1
#define PRODUCTID               0x1234

#define GPIO_KBD                0
#define DEFAULT_DEBOUNCE_NUM    25      /* 250ms (1 SysTick = 10ms) */
#define DEFAULT_MODIFIER        0
#define DEFAULT_KEYCODE         4       /* KEY_A */

#define HID_REPORT_DESC_LEN     35

/**
 * Keyboard HID Device Descriptor
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
// Byte |  D7    D6    D5    D4     D3        D2        D1      D0
// -----+-------------------------------------------------------------------
//  0   |  0     0     0   KANA  Compose  ScrollLock CapsLock NumLock
//  1   |                         Keycode
//
// Output report - n/a
//
// Feature report - n/a
//

/**
 * Keyboard HID report descriptor
 */
uint8_t hid_report_desc[HID_REPORT_DESC_LEN] = {
    0x05, 0x01,                    /* USAGE_PAGE (Generic Desktop) */
    0x09, 0x06,                    /* USAGE (Keyboard) */
    0xa1, 0x01,                    /* COLLECTION (Application) */
    0x05, 0x07,                    /*   USAGE_PAGE (Keyboard) */
    0x19, 0xe0,                    /*   USAGE_MINIMUM (Keyboard LeftControl) */
    0x29, 0xe7,                    /*   USAGE_MAXIMUM (Keyboard Right GUI) */
    0x15, 0x00,                    /*   LOGICAL_MINIMUM (0) */
    0x25, 0x01,                    /*   LOGICAL_MAXIMUM (1) */
    0x75, 0x01,                    /*   REPORT_SIZE (1) */
    0x95, 0x08,                    /*   REPORT_COUNT (8) */
    0x81, 0x02,                    /*   INPUT (Data,Var,Abs) */
    0x95, 0x01,                    /*   REPORT_COUNT (1) */
    0x75, 0x08,                    /*   REPORT_SIZE (8) */
    0x25, 0x65,                    /*   LOGICAL_MAXIMUM (101) */
    0x19, 0x00,                    /*   USAGE_MINIMUM (Reserved (no event)) */
    0x29, 0x65,                    /*   USAGE_MAXIMUM (Keyboard Application) */
    0x81, 0x00,                    /*   INPUT (Data,Ary,Abs) */
    0xc0                           /* END_COLLECTION */
};

/**
 * Keyboard report data
 */
struct hid_kbd_data {
    /** modifier key: bit[0-4]: NumLock, CapsLock, ScrollLock, Compose, KANA */
    uint8_t modifier;
    /** keycode, 0 ~ 101 key value  */
    uint8_t keycode;
} __packed;

/**
 * report length of each HID Reports in HID Report Descriptor
 */
struct hid_size_info hid_sizeinfo[] =
{
    { 0, { 2, 0, 0 } }, /* parsed from HID Report Descriptor manually */
};

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
    struct tsb_hid_info *info = NULL;
    struct hid_kbd_data *kbd;
    int ret = 0, rptlen = 0;

    info = dev->private;

    rptlen = tsb_hid_get_report_length(dev, HID_INPUT_REPORT, report_id);

    if (rptlen) {
        if (!report_id) {
            if (len < sizeof(struct hid_kbd_data)) {
                return -EINVAL;
            }
            /* get keyboard data and return to upper layer */
            kbd = (struct hid_kbd_data *)data;
            kbd->modifier = info->modifier;
            kbd->keycode = gpio_get_value(GPIO_KBD) ? info->keycode : 0;
        } else {
            /* For current case, we don't support multiple Report ID structure,
             * so just returns error code */
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
        /* For current HID case, HID Report Descriptor doesn't contain a
         * Feature Report structure, so I just return -EIO error code. */
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
         * Output Report structure, so I just return -EIO error code. */
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
         * Feature Report structure, so I just return -EIO error code. */
        ret = -EIO;
    } else {
        /* Can't find Feature Report structure in report descriptor. */
        ret = -EIO;
    }
    return ret;
}

/**
 * @brief HID device interrupt routing
 *
 * @param context pointer to structure of device data
 */
int tsb_hid_irq_event(int irq, FAR void *context)
{
    struct device *dev = hid_dev;
    struct tsb_hid_info *info = NULL;
    struct hid_kbd_data kbd;
    uint8_t value = 0;
    int ticks = 0, elapsed = 0;

    if (!dev || !dev->private) {
        return ERROR;
    }

    info = hid_dev->private;

    clock_time2ticks(&info->last_activetime, &ticks);
    elapsed = clock_systimer() - ticks;

    if (elapsed >= info->debounce_count) {
        gpio_mask_irq(irq);
        value = gpio_get_value(GPIO_KBD);

        /* check whether the key state change or not */
        if (info->last_keystate != value) {
            info->last_keystate = value;
            clock_ticks2time(clock_systimer(), &info->last_activetime);

            /* get keyboard data and send to upper layer */
            kbd.modifier = info->modifier;
            kbd.keycode = value ? info->keycode : 0;

            if (info->event_callback) {
                info->event_callback(dev, HID_INPUT_REPORT, (uint8_t*)&kbd,
                                     sizeof(struct hid_kbd_data));
            }
        }
        gpio_unmask_irq(irq);
    }
    return OK;
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
        /* enable interrupt */
        gpio_unmask_irq(GPIO_KBD);
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
        /* changed power-on state */
        info->state &= ~HID_DEVICE_FLAG_POWERON;
        /* disable interrupt */
        gpio_mask_irq(GPIO_KBD);
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
    return 0;
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
    return 0;
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
    if (id) {
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
        default:
            /* only support input and feature report */
            ret = -EINVAL;
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
        default:
            /* only support output and feature report */
            ret = -EINVAL;
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
    return 0;
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
    return 0;
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
    info->state |= HID_DEVICE_FLAG_OPEN;

    /* set device default value */
    info->last_activetime.tv_sec = 0;
    info->last_activetime.tv_nsec = 0;
    info->last_keystate = 0xFF;
    info->debounce_count = DEFAULT_DEBOUNCE_NUM;
    info->modifier = DEFAULT_MODIFIER;
    info->keycode = DEFAULT_KEYCODE;

    /* initialize GPIO pin */
    if (GPIO_KBD >= gpio_line_count()) {
        info->state &= ~HID_DEVICE_FLAG_OPEN;
        ret = -EIO;
        goto err_open;
    }
    gpio_activate(GPIO_KBD);
    gpio_direction_in(GPIO_KBD);
    gpio_mask_irq(GPIO_KBD);
    set_gpio_triggering(GPIO_KBD, IRQ_TYPE_EDGE_BOTH);
    gpio_irqattach(GPIO_KBD, tsb_hid_irq_event);

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

    /* uninitialize GPIO pin */
    gpio_unmask_irq(GPIO_KBD);
    gpio_deactivate(GPIO_KBD);

    if (info->state & HID_DEVICE_FLAG_OPEN) {
        /* clear open state */
        info->state &= ~HID_DEVICE_FLAG_OPEN;
    }
    info->hdesc = NULL;
    info->rdesc = NULL;
    info->sinfo = NULL;
    info->num_ids = 0;

    info->last_activetime.tv_sec = 0;
    info->last_activetime.tv_nsec = 0;
    info->last_keystate = 0xFF;
    info->debounce_count = 0;

    info->event_callback = NULL;

err_close:
    sem_post(&info->lock);
}

/**
 * @brief Probe HID device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private HID device
 * information, and then setup the hardware resource and interrupt handler.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_dev_probe(struct device *dev)
{
    struct tsb_hid_info *info = NULL;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    hid_dev = dev;
    info->dev = dev;
    info->state = HID_DEVICE_FLAG_PROBE;
    dev->private = info;

    sem_init(&info->lock, 0, 1);
    return 0;
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
    sem_destroy(&info->lock);

    dev->private = NULL;
    hid_dev = NULL;
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
