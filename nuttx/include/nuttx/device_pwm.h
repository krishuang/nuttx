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

#ifndef __INCLUDE_DEVICE_PWM_H
#define __INCLUDE_DEVICE_PWM_H

/* For the purposes of this driver, a PWM device is any device that generates
 * periodic output pulses s of controlled frequency and pulse width.  Such a
 * device might be used, for example, to perform pulse-width modulated output or
 * frequency/pulse-count modulated output (such as might be needed to control
 * a stepper motor).
 *
 * The PWM driver is split into two parts:
 *
 * 1) An "upper half" that provides the comman PWM interface
 *    to greybus PWM protocol level code, and
 * 2) A "lower half", device driver that implements the low-level
 *    timer controls to implement the PWM functionality.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/list.h>

#include <fixedmath.h>

#include <nuttx/greybus/types.h>

#ifdef CONFIG_PWM

/* This structure is a set a callback functions used to call from the
 * Greybus PWM protocol level into device driver that supports the
 * low-level timer outputs.
 */
struct pwm_dev;
struct pwm_ops
{
  /* This method is called when "deactivate" protocol operation. The
   * device driver should deinitialize the device so that it is not
   * assign for use, and free any resources, turn off power/clock of PWM
   * controller if no more PWMx assigned for use.
   */

  int (*deactivate)(struct pwm_dev *dev);

  /* This method is called when "enable" protocol operation. The
   * device driver should set HW register by "period" and "duty_cycle"
   * to start pulsed output
   */

  int (*enable)(struct pwm_dev *dev);

  /* This method is called when "disable" protocol operation. The
   * device driver should stop specific PWMx pulse output immedeiately.
   */

  int (*disable)(struct pwm_dev *dev);

  /* This method is called when "config" protocol operation. The
   * device driver should configure HW register by "period" and
   * "duty_cycle" as requirement.
   */

  int (*config)(struct pwm_dev *dev, uint32_t duty_cycle,
                uint32_t period);

  /* This method is called when "Set Polarity" protocol operation. The
   * device driver should change PWMx pulse output to require polarity.
   */

  int (*setpolarity)(struct pwm_dev *dev, uint8_t polarity);
};


struct pwm_dev
{
    const struct pwm_ops *ops;  /* The field of this structure is ops pointer
                                 * for callback.
                                 */
};

struct pwm_generators
{
    struct pwm_dev *dev;
    struct list_head list;
    uint8_t which;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

 struct version_table {
     uint8_t major;
     uint8_t minor;
};


#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* This method is called when "activate" protocol operation. The device
 * driver should assign supported callbacks APIs into pwm_dev of
 * dev_pwm_ops and return a pwm_dev for specific PWMx to protocol
 * interface, the protocol driver will use returned pwm_dev to manage
 * PWMx status and other PWM protocol operation.
 * Meanwhile, configure and initialize the power/clock of PWM controller
 *  if it has not be operated, and to configure the specific PWMx so
 * that it is ready for use.
 *
 * Importantly, the specific PWMx should not output pulses until the
 * “enable” API is called
*/

EXTERN struct pwm_dev *pwm_initialize(uint8_t pwmx);

/* This API is called when the "PWM count" protocol operation coming.
 * The device driver should to report supported PWM device number on
 * controller.
 * By Greybus specification v0.11, the device driver must report at
 * least 1 device and maxima up to 256 to present supported PWM device
 * on controller.
 */

EXTERN uint16_t pwm_count(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* CONFIG_PWM */
#endif /* __INCLUDE_DEVICE_PWM_H */
