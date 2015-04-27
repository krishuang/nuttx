/*
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

#include <string.h>
#include <nuttx/usb/apb_es1.h>
#include <apps/greybus-utils/utils.h>

#include "apbridge_backend.h"

static int gbsim_usb_to_unipro(unsigned int cportid, void *payload, size_t size)
{
    greybus_rx_handler(cportid, payload, size);
    return size;
}

static int gbsim_usb_to_svc(void *payload, size_t size)
{
    return svc_handle(payload, size);
}

static int gbsim_svc_to_usb(struct apbridge_dev_s *dev,
                            void *buf, size_t length)
{
    return svc_to_usb(dev, buf, length);
}

static int gbsim_unipro_to_usb(struct apbridge_dev_s *dev,
                            unsigned int cportid, void *buf, size_t len)
{
    return unipro_to_usb(dev, buf, len);
}

static void init(void)
{
}

static int listen(unsigned int cport)
{
    return 0;
}

struct gb_transport_backend gb_unipro_backend = {
    .init = init,
    .listen = listen,
    .send = recv_from_unipro,
};

static void manifest_enable(unsigned char *manifest_file, int manifest_number)
{
    char iid[IID_LENGTH];

    snprintf(iid, IID_LENGTH, "IID-%d", manifest_number + 1);
    enable_manifest(iid, NULL);
}

void apbridge_backend_register(struct apbridge_backend *apbridge_backend)
{
    apbridge_backend->usb_to_unipro = gbsim_usb_to_unipro;
    apbridge_backend->unipro_to_usb = gbsim_unipro_to_usb;
    apbridge_backend->usb_to_svc = gbsim_usb_to_svc;
    apbridge_backend->svc_to_usb = gbsim_svc_to_usb;
    gb_init(&gb_unipro_backend);
}

void apbridge_backend_init(void)
{
    foreach_manifest(manifest_enable);
    enable_cports();
}
