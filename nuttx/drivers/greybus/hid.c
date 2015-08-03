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
#include <queue.h>

#include <nuttx/device.h>
#include <nuttx/device_hid.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>

#include <arch/byteorder.h>

#include "hid-gb.h"

#define GB_HID_VERSION_MAJOR 0
#define GB_HID_VERSION_MINOR 1

/* Reserved operations for rx data buffer. */
#define MAX_RX_OPERATION        5
#define MAX_RX_BUF_SIZE         32

/**
 * The buffer in operation structure.
 */
struct op_node {
    /** queue entry */
    sq_entry_t entry;

    /** pointer to operation */
    struct gb_operation *operation;

    /** pointer to buffer of request in operation */
    uint8_t  *buffer;
};

struct gb_hid_info {
    /** assigned CPort number */
    uint16_t cport;

    /** opened device driver handler */
    struct device *dev;

    /** device descriptor for this device */
    struct gb_hid_desc_response hid_desc;

    /** device type for this device */
    char *dev_type;

    /** Id for device in device table */
    uint16_t dev_id;

    /** available operation queue */
    sq_queue_t free_queue;

    /** received data operation queue */
    sq_queue_t data_queue;

    /** operation node in receiving */
    struct op_node *rx_node;

    /** buffer size in operation */
    int rx_buf_size;

    /** amount of operations */
    int entries;

    /** semaphore for notifying data received */
    sem_t rx_sem;

    /** receiving data process threed */
    pthread_t rx_thread;

    /** inform the thread should be terminated */
    int thread_stop;
};

static struct gb_hid_info *hid_info = NULL;

/**
 * @brief Put the node to the back of the queue.
 *
 * @param queue The target queue to put.
 * @param node The pointer to node.
 * @return None.
 */
static void put_node_back(sq_queue_t *queue, struct op_node *node)
{
    irqstate_t flags = irqsave();

    sq_addlast(&node->entry, queue);

    irqrestore(flags);
}

/**
 * @brief Get a node from the queue.
 *
 * @param queue The target queue.
 * @return A pointer to the node or NULL for no node to get.
 */
static struct op_node *get_node_from(sq_queue_t *queue)
{
    struct op_node *node = NULL;
    irqstate_t flags = irqsave();

    if (sq_empty(queue)) {
        irqrestore(flags);
        return NULL;
    }

    node = (struct op_node *)sq_remfirst(queue);
    irqrestore(flags);

    return node;
}

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
    struct hid_descriptor hid_desc;
    int ret = 0;

    if (!hid_info || !hid_info->dev) {
        return GB_OP_UNKNOWN_ERROR;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    ret = device_hid_get_descriptor(hid_info->dev, &hid_desc);
    if (ret) {
        gb_info("%s(): %x error in ops\n", __func__, ret);
        return GB_OP_MALFUNCTION;
    }

    response->length = hid_desc.length;
    response->report_desc_length = cpu_to_le16(hid_desc.report_desc_length);
    response->hid_version = cpu_to_le16(hid_desc.hid_version);
    response->product_id = cpu_to_le16(hid_desc.product_id);
    response->vendor_id = cpu_to_le16(hid_desc.vendor_id);
    response->country_code = hid_desc.country_code;

    memcpy(&hid_info->hid_desc, &hid_desc,
           sizeof(struct gb_hid_desc_response));

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
    uint8_t *response;
    int ret = 0;

    if (!hid_info || !hid_info->dev) {
        return GB_OP_UNKNOWN_ERROR;
    }

    response = gb_operation_alloc_response(operation,
                                        hid_info->hid_desc.report_desc_length);
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    ret = device_hid_get_report_descriptor(hid_info->dev, response);
    if (ret) {
        lldbg("%s(): %x error in ops\n", __func__, ret);
        return GB_OP_MALFUNCTION;
    }

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
    int ret = 0;

    if (!hid_info || !hid_info->dev) {
        return GB_OP_UNKNOWN_ERROR;
    }

    ret = device_hid_power_on(hid_info->dev);
    if (ret) {
        gb_info("%s(): %x error in ops\n", __func__, ret);
        return GB_OP_MALFUNCTION;
    }

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
    int ret = 0;

    if (!hid_info || !hid_info->dev) {
        return GB_OP_UNKNOWN_ERROR;
    }

    ret = device_hid_power_off(hid_info->dev);
    if (ret) {
        gb_info("%s(): %x error in ops\n", __func__, ret);
        return GB_OP_MALFUNCTION;
    }

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
    uint8_t *response;
    uint16_t report_len;
    int ret = 0;

    request = gb_operation_get_request_payload(operation);

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
    if (request->report_id > 0 && request->report_id != 9) {
        report_len += 1;
    }

    response = gb_operation_alloc_response(operation, report_len);
    if (!response) {
            return GB_OP_NO_MEMORY;
    }

    ret = device_hid_get_report(hid_info->dev, request->report_type,
                                request->report_id, (uint8_t *)response,
                                report_len);
    if (ret) {
        lldbg("%s(): %x error in ops\n", __func__, ret);
        return GB_OP_MALFUNCTION;
    }

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
    uint16_t report_len;
    int ret = 0;

    request = gb_operation_get_request_payload(operation);

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

    return GB_OP_SUCCESS;
}

/**
 * @brief Callback for data receiving
 *
 * The callback function provided to device driver for being notified when
 * driver received a data stream.
 * It put the current operation to received queue and gets another operation to
 * continue receiving. Then notifies rx thread to process.
 *
 * @param buffer Data buffer.
 * @param length Received data length.
 * @param error Error code when driver receiving.
 * @return None.
 */
static int hid_event_callback_routine(struct device *dev, uint8_t report_type,
                                      uint8_t *report, uint16_t len)
{
    struct op_node *node;

    memcpy(hid_info->rx_node->buffer, report, len);
    put_node_back(&hid_info->data_queue, hid_info->rx_node);

    node = get_node_from(&hid_info->free_queue);
    if (!node) {
        return -ENOMEM;
    }

    hid_info->rx_node = node;

    sem_post(&hid_info->rx_sem);

    return 0;
}

/**
 * @brief Data receiving process thread
 *
 * This function is the thread for processing data receiving tasks. When
 * it wake up, it checks the receiving queue for processing the come in data.
 * If protocol is running out of operation, once it gets a free operation,
 * it passes to driver for continuing the receiving.
 *
 * @param data The regular thread data.
 * @return None.
 */
static void *hid_rx_thread(void *data)
{
    struct op_node *node = NULL;
    int ret;

    while (1) {
        sem_wait(&hid_info->rx_sem);

        if (hid_info->thread_stop) {
            break;
        }

        node = get_node_from(&hid_info->data_queue);
        if (node) {
            ret = gb_operation_send_request(node->operation, NULL, false);
            if (ret) {
                gb_debug("%s: IRQ Event operation failed (%x)!\n", __FUNC__,
                         ret);
            }
            put_node_back(&hid_info->free_queue, node);
        }
    }

    return NULL;
}

/**
 * @brief Free operations
 *
 * This funciton destroy operations and node memory.
 *
 * @param queue Target queue.
 * @return None.
 */
static void hid_free_op(sq_queue_t *queue)
{
    struct op_node *node = NULL;

    node = get_node_from(queue);
    while (node) {
        gb_operation_destroy(node->operation);
        free(node);
        node = get_node_from(queue);
    }
}

/**
 * @brief Allocate operations for receiver buffers
 *
 * This function is allocating operation and use them as receiving buffers.
 *
 * @param max_nodes Maximum nodes.
 * @param buf_size Buffer size in operation.
 * @param queue Target queue.
 * @return 0 for success, -errno for failures.
 */
static int hid_alloc_op(int max_nodes, int buf_size, sq_queue_t *queue)
{
    struct gb_operation *operation = NULL;
    struct gb_hid_input_report_request *request = NULL;
    struct op_node *node = NULL;
    int i = 0;

    for (i = 0; i < max_nodes; i++) {
        operation = gb_operation_create(hid_info->cport,
                                        GB_HID_TYPE_IRQ_EVENT, buf_size);
        if (!operation) {
            goto err_free_op;
        }

        node = malloc(sizeof(struct op_node));
        if (!node) {
            gb_operation_destroy(operation);
            goto err_free_op;
        }
        node->operation = operation;

        request = gb_operation_get_request_payload(operation);
        node->buffer = request->report;
        put_node_back(queue, node);
    }

    return 0;

err_free_op:
    hid_free_op(queue);

    return -ENOMEM;
}


/**
 * @brief Receiving data process initialization
 *
 * This function allocates OS resource to support the data receiving
 * function. It allocates two types of operations for undetermined length of
 * data. The semaphore works as message queue and all tasks are done in the
 * thread.
 *
 * @param None.
 * @return 0 for success, -errno for failures.
 */
static int hid_receiver_cb_init(void)
{
    int ret;

    sq_init(&hid_info->free_queue);
    sq_init(&hid_info->data_queue);

    hid_info->entries = MAX_RX_OPERATION;

    ret = hid_alloc_op(hid_info->entries, hid_info->rx_buf_size,
                       &hid_info->free_queue);
    if (ret) {
        return ret;
    }

    ret = sem_init(&hid_info->rx_sem, 0, 0);
    if (ret) {
        goto err_free_data_op;
    }

    ret = pthread_create(&hid_info->rx_thread, NULL, hid_rx_thread, hid_info);
    if (ret) {
        goto err_destroy_rx_sem;
    }

    return 0;

err_destroy_rx_sem:
    sem_destroy(&hid_info->rx_sem);
err_free_data_op:
    hid_free_op(&hid_info->free_queue);

    return -ret;
}

/**
 * @brief Releases resources for receiver thread.
 *
 * Terminates the thread for receiver and releases the system resouces and
 * operations allocated by hid_receiver_cb_init().
 *
 * @param None.
 * @return None.
 */
static void hid_receiver_cb_deinit(void)
{
    if (hid_info->rx_thread != (pthread_t)0) {
        hid_info->thread_stop = 1;
        sem_post(&hid_info->rx_sem);
        pthread_join(hid_info->rx_thread, NULL);
    }

    sem_destroy(&hid_info->rx_sem);

    hid_free_op(&hid_info->data_queue);
    hid_free_op(&hid_info->free_queue);
}


/**
 * @brief Greybus HID protocol initialize function
 *
 * @param cport CPort number
 * @return 0 on success, negative errno on error
 */
static int gb_hid_init(unsigned int cport)
{
    int ret;

    hid_info = zalloc(sizeof(*hid_info));
    if (!hid_info) {
        return -ENOMEM;
    }

    hid_info->cport = cport;
    hid_info->dev_type = DEVICE_TYPE_HID_HW;
    hid_info->dev_id = 0;

    hid_info->dev = device_open(hid_info->dev_type, hid_info->dev_id);
    if (!hid_info->dev) {
        gb_info("%s(): failed to open HID device!\n", __func__);
        ret = -EIO;
        goto err_hid_init;
    }

    ret = device_hid_get_max_report_length(hid_info->dev, GB_HID_INPUT_REPORT);
    if (ret < 0) {
        goto err_hid_init;
    }

    hid_info->rx_buf_size = ret;

    ret = hid_receiver_cb_init();
    if (ret) {
        goto err_hid_receiver_cb_deinit;
    }

    /* Get first node pointer */
    hid_info->rx_node = get_node_from(&hid_info->free_queue);

    ret = device_hid_register_callback(hid_info->dev,
                                       hid_event_callback_routine);
    if (ret) {
        goto err_device_close;
    }

    return 0;

err_device_close:
    device_close(hid_info->dev);
err_hid_receiver_cb_deinit:
    hid_receiver_cb_deinit();
err_hid_init:
    free(hid_info);
    return ret;
}

/**
 * @brief Greybus HID protocol deinitialize function
 *
 * @param cport CPort number
 */
static void gb_hid_exit(unsigned int cport)
{

    device_hid_unregister_callback(hid_info->dev);


    if (hid_info->dev) {
        device_close(hid_info->dev);
    }

    hid_receiver_cb_deinit();

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
