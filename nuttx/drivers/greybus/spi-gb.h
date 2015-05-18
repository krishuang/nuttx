/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 for more details.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2014 - 2015 Google Inc. All rights reserved.
 * Copyright(c) 2014 - 2015 Linaro Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Google Inc. or Linaro Ltd. nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GOOGLE INC. OR
 * LINARO LTD. BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _GREYBUS_SPI_H_
#define _GREYBUS_SPI_H_

#include <nuttx/greybus/types.h>

/* SPI Protocol Operation Types */
#define GB_SPI_PROTOCOL_VERSION             0x01
#define GB_SPI_PROTOCOL_MODE                0x02
#define GB_SPI_PROTOCOL_FLAGS               0x03
#define GB_SPI_PROTOCOL_BITS_PER_WORD_MASK  0x04
#define GB_SPI_PROTOCOL_NUM_CHIPSELECT      0x05
#define GB_SPI_PROTOCOL_TRANSFER            0x06

/* SPI Protocol Mode Bit Masks */
#define GB_SPI_MODE_CPHA        0x01    /* clock phase */
#define GB_SPI_MODE_CPOL        0x02    /* clock polarity */
#define GB_SPI_MODE_CS_HIGH     0x04    /* chipselect active high */
#define GB_SPI_MODE_LSB_FIRST   0x08    /* per-word bits-on-wire */
#define GB_SPI_MODE_3WIRE       0x10    /* SI/SO signals shared */
#define GB_SPI_MODE_LOOP        0x20    /* loopback mode */
#define GB_SPI_MODE_NO_CS       0x40    /* one dev/bus, no chipselect */
#define GB_SPI_MODE_READY       0x80    /* slave pulls low to pause */

/* SPI Protocol Flags */
#define GB_SPI_FLAG_HALF_DUPLEX 0x01    /* can't do full duplex */
#define GB_SPI_FLAG_NO_RX       0x02    /* can't do buffer read */
#define GB_SPI_FLAG_NO_TX       0x04    /* can't do buffer write */

/* version request has no payload */
struct gb_spi_proto_version_response {
    __u8    major;
    __u8    minor;
};

/* mode request has no payload */
struct gb_spi_mode_response {
    __le16  mode;
};

/* flags request has no payload */
struct gb_spi_flags_response {
    __le16  flags;
};

/* bits-per-word request has no payload */
struct gb_spi_bpw_response {
    __le32  bits_per_word_mask;
};

/* num-chipselects request has no payload */
struct gb_spi_chipselect_response {
    __le16  num_chipselect;
};

struct gb_spi_transfer_desc {
    __le32  speed_hz;
    __le32  len;
    __le16  delay_usecs;
    __u8    cs_change;
    __u8    bits_per_word;
};

struct gb_spi_transfer_request {
    __u8    chip_select;
    __u8    mode;
    __le16  count;
    struct gb_spi_transfer_desc  transfers[0];
};

struct gb_spi_transfer_response {
    __u8    data[0];
};

#endif /* _GREYBUS_SPI_H_ */

