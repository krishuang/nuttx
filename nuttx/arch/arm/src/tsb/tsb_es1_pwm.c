/*
 * Copyright (c) 2015 Google Inc.
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
#include <assert.h>

#include <nuttx/device_pwm.h>
#include <nuttx/config.h>
#include <apps/greybus-utils/utils.h>

#include <sys/types.h>

#include "up_arch.h"

#include "chip.h"
#include "tsb_pwm.h"
#include "tsb_scm.h"

/* This structure represents the state of one PWM */
struct tsb_es1_pwm{
    const struct pwm_ops *ops;  /* PWM operations */
    uint32_t base;              /* The base address of the pwm */
    uint32_t pincfg;            /* Output pin configuration */
    uint32_t pclk;              /* The frequency of the pwm clock
                                 * (after divider)
                                 */
};

static void tsb_pwm_write(struct tsb_es1_pwm *dev, uint32_t addr,
                          uint32_t v) {
    putreg32(v, dev->base + addr);
}

static uint32_t tsb_pwm_read(struct tsb_es1_pwm *dev, uint32_t addr)
{
    return getreg32(dev->base + addr);
}

static int pwm_deactivate(struct pwm_dev *dev)
{
    gb_info("%s()\n", __func__);
    //tsb_clk_disable(TSB_CLK_PWMODP);
    //tsb_clk_disable(TSB_CLK_PWMODS);

    return OK;
}

static int pwm_config(struct pwm_dev *dev, uint32_t duty_cycle,
                      uint32_t period) {

    return OK;
}

static int pwm_enable(struct pwm_dev *dev)
{
    return OK;
}

static int pwm_disable(struct pwm_dev *dev)
{
    return OK;
}

static int pwm_setpolarity(struct pwm_dev *dev, uint8_t polarity)
{
    return OK;
}

static const struct pwm_ops g_pwmops = {
    .deactivate = pwm_deactivate,
    .enable = pwm_enable,
    .disable = pwm_disable,
    .config = pwm_config,
    .setpolarity = pwm_setpolarity,
};

static struct tsb_es1_pwm g_pwm0dev = {
    .ops = &g_pwmops,
    .base = TSB_PWM0,
    .pclk = TSB_PWM_CLK,
};

static struct tsb_es1_pwm g_pwm1dev = {
    .ops = &g_pwmops,
    .base = TSB_PWM1,
    .pclk = TSB_PWM_CLK,
};

uint16_t pwm_count(void)
{
    return TSB_PWM_NUMBER;
}

struct pwm_dev *pwm_initialize(uint8_t pwmx)
{
    struct tsb_es1_pwm *chiper;

    gb_info("%s()\n", __func__);

    if (pwmx == 0) {
        chiper = &g_pwm0dev;
        gb_info("%s() pwm0 pointer is 0x%x\n", __func__, chiper);
    }
    else if (pwmx == 1) {
        chiper = &g_pwm1dev;
        gb_info("%s() pwm1 pointer is 0x%x\n", __func__, chiper);
    }
    else
        return NULL;
    return (struct pwm_dev *)chiper;
}
