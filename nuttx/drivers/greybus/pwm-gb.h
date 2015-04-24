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

#ifndef _GREYBUS_PWM_H_
#define _GREYBUS_PWM_H_

#include <nuttx/greybus/types.h>

#define GB_PWM_PROTOCOL_VERSION         0x01
#define GB_PWM_PROTOCOL_COUNT           0x02
#define GB_PWM_PROTOCOL_ACTIVATE        0x03
#define GB_PWM_PROTOCOL_DEACTIVATE      0x04
#define GB_PWM_PROTOCOL_CONFIG          0x05
#define GB_PWM_PROTOCOL_POLARITY        0x06
#define GB_PWM_PROTOCOL_ENABLE          0x07
#define GB_PWM_PROTOCOL_DISABLE         0x08

struct gb_pwm_version_request {
    __u8    offer_major;
    __u8    offer_minor;
};

struct gb_pwm_version_response {
	__u8	major;
	__u8	minor;
};

/* count request has no payload */
struct gb_pwm_count_response {
	__u8	count;
};

/* activate response has no payload */
struct gb_pwm_activate_request {
	__u8	which;
};

/* deactivate response has no payload */
struct gb_pwm_dectivate_request {
	__u8	which;
};

/* config response has no payload */
struct gb_pwm_config_request {
	__u8	which;
    __le32  duty_cycle;
    __le32  period;
};

/* config response has no payload */
struct gb_pwm_polarity_request {
	__u8	which;
    __u8    polarity;
};


/* enable response has no payload */
struct gb_pwm_enable_request {
	__u8	which;
};

/* disable response has no payload */
struct gb_pwm_disable_request {
	__u8	which;
};

#endif /* _GREYBUS_PWM_H_ */

