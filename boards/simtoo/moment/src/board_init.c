/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_init.c
 *
 * Board specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>

#include <systemlib/px4_macros.h>

//#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>
//#include <drivers/drv_pwm_output.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS

__END_DECLS

//__EXPORT void board_on_reset(int status)
//{
//	/* configure the GPIO pins to outputs and keep them low */
////	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
////		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
////	}
//
//	/**
//	 * On resets invoked from system (not boot) insure we establish a low
//	 * output state (discharge the pins) on PWM pins before they become inputs.
//	 */
//
//	if (status >= 0) {
//		up_mdelay(400);
//	}
//}


__EXPORT void stm32_boardinitialize(void)
{
//	board_on_reset(-1);

	stm32_configgpio(GPIO_MINE);

	board_spi_init_hardware();


// Remark: Doesn't seem to be necessary
//	stm32_configgpio(GPIO_I2C2_SCL);
//	stm32_configgpio(GPIO_I2C2_SDA);

// Remark: Doesn't seem to be necessary
//	stm32_usbinitialize();
} // __EXPORT void stm32_boardinitialize(void)



__EXPORT int board_app_initialize(uintptr_t arg)
{
//	for (int i=0; i<3; i++) {
//		stm32_gpiowrite(GPIO_MINE, 1);
//		up_mdelay(1000);
//		stm32_gpiowrite(GPIO_MINE, 0);
//		up_mdelay(1000);
//	}

	px4_platform_init();

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "DMA alloc FAILED\n");
	}


	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);


	if (board_hardfault_init(2, true) != 0) {
		stm32_gpiowrite(GPIO_MINE, 1);
	}


	int ret = board_spi_init_interface();

	if (ret != OK) {
		stm32_gpiowrite(GPIO_MINE, 1);
		return ret;
	}

	px4_platform_configure();

	return OK;
} // __EXPORT int board_app_initialize(uintptr_t arg)











































