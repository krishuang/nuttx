/**
 * Copyright (c) 2014-2015 Google Inc.
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

#include <nuttx/config.h>
#include <arch/tsb/unipro.h>

#include <stdio.h>
#include <string.h>

#define NUM_CPORTS   (4)
#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif


static int greybus_rx_handler(unsigned int cportid, void *data, size_t len);

static unsigned int greybus_msg[] = {
    0x00facade,
    0xbaddcafe,
    0x1badb002,
    0x1badf00d
};

static int dsi_attrs[] = {
    0x4025, 16,
    0x4023, 16,
    0x4022, 16,
    0x4021, 16,
    0x4020, 16,
    0x8032, 0x0,
    0x007f, 0x0,
    0x1568, 0x0,
    0x1569, 0x0,
    0x156a, 0x0,
    0x1560, 0x0, 
    0x1583, 0x0,
    0x1584, 0x0,
    0x1580, 0x0,
    0x15B0, 0x0,
    0x1571, 0x0,
};

static struct unipro_driver greybus_driver = {
    .name = "greybus",
    .rx_handler = greybus_rx_handler,
};


/*
 * Called in IRQ context
 */
static int greybus_rx_handler(unsigned int cportid, void *data, size_t len) {
    unsigned int *payload = (unsigned int*)data;

    /* pass it off to greybus core? */
    // gb_message_handler()
    return 0;
}

static int greybus_register(void) {
    int i = 0;
    for (i = 0; i < NUM_CPORTS; i++) {
        unipro_driver_register(&greybus_driver, i);
    }
    return 0;
}

static int attr_read(int argc, char **argv) {
    unsigned int attr;
    unsigned int val;
    int peer = 0;
    unsigned int rc;
    unsigned int selector = 0;

    if (argc < 2) {
        printf("usage: <attr> <1 for peer, 0 for local> <selector(optional)>\n");
        exit(1);
    }

    attr = strtoul(argv[0], NULL, 16);
    peer = strtoul(argv[1], NULL, 10);

    if (argc == 3) {
        selector = strtoul(argv[2], NULL, 10);
    }

    unipro_attr_read(attr, &val, selector, peer, &rc);

    printf("attr: %x peer: %u selector: %u val: %x resultcode: %x\n", attr, peer, selector, val, rc);

    return 0;
}

int unipro_main(int argc, char **argv) {
    char *op;
    int rc;
    int i;
    unsigned int val;

    if (argc < 2) {
        return -1;
    }

    op = argv[1];

    if (strcmp(op, "r") == 0) {
        printf("Attribute read:\n");
        return attr_read(argc - 2, &argv[2]);
    } else if (strcmp(op, "dsi") == 0) {
        for (i =0; i < ARRAY_SIZE(dsi_attrs); i += 2) {
            unipro_attr_read(dsi_attrs[i], &val, dsi_attrs[i+1], 0, &rc);
            printf("attr: %x, selector: %d, val: %x resultcode: %x\n", dsi_attrs[i], dsi_attrs[i+1], val, rc);
  
        } 
    } else if (strcmp(op, "init") == 0) {
        printf("Initializing unipro.\n");
        unipro_init();
        rc = greybus_register();
        if (rc) {
            printf("Failed to register driver. rc: %d\n", rc);
            exit(1);
        }
    } else if ((strcmp(op, "send") == 0) && argc == 3) {
        rc = unipro_send(strtoul(argv[2], NULL, 10), greybus_msg, sizeof greybus_msg);
        if (rc) {
            printf("Failed to send data. rc: %d\n", rc);
            exit(1);
        }
    } else if (strcmp(op, "info") == 0) {
        unipro_info();
    }

    return 0;
}
