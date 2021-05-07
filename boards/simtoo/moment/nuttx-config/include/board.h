/************************************************************************************
 * nuttx-config/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __CONFIG_MOMENT_INCLUDE_BOARD_H
#define __CONFIG_MOMENT_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/
#define NO_TUNES

/* Clocking *************************************************************************/
/* The SimToo Moment uses a 24MHz crystal connected to the HSE.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 24000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 24           (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 24MHz
 * LSE - not installed
 */

#define STM32_BOARD_XTAL        24000000ul

#define PLLM	 	 24
#define PLLN		336
#define PLLP		  2
#define PLLQ		  7
#define AHB_PRE		  1
#define APB1_PRE	  4
#define APB2_PRE	  2

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
//#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (24,000,000 / 24) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 336,000,000 / 7
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(PLLM)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(PLLN)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP(PLLP)
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(PLLQ)

#define STM32_PLL_VCO			(STM32_HSE_FREQUENCY / PLLM) * PLLN

#define STM32_SYSCLK_FREQUENCY  STM32_PLL_VCO / PLLP

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY / AHB_PRE
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY / APB1_PRE)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2 * STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY / APB2_PRE)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2 * STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8-11 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_APB2_TIM1_CLKIN
#define BOARD_TIM2_FREQUENCY    STM32_APB1_TIM2_CLKIN
#define BOARD_TIM3_FREQUENCY    STM32_APB1_TIM3_CLKIN
#define BOARD_TIM4_FREQUENCY    STM32_APB1_TIM4_CLKIN
#define BOARD_TIM5_FREQUENCY    STM32_APB1_TIM5_CLKIN
#define BOARD_TIM6_FREQUENCY    STM32_APB1_TIM6_CLKIN
#define BOARD_TIM7_FREQUENCY    STM32_APB1_TIM7_CLKIN
#define BOARD_TIM8_FREQUENCY    STM32_APB2_TIM8_CLKIN
#define BOARD_TIM9_FREQUENCY    STM32_APB2_TIM9_CLKIN
#define BOARD_TIM10_FREQUENCY   STM32_APB2_TIM10_CLKIN
#define BOARD_TIM11_FREQUENCY   STM32_APB2_TIM11_CLKIN
#define BOARD_TIM12_FREQUENCY   STM32_APB1_TIM12_CLKIN
#define BOARD_TIM13_FREQUENCY   STM32_APB1_TIM13_CLKIN
#define BOARD_TIM14_FREQUENCY   STM32_APB1_TIM14_CLKIN

/* Alternate function pin selections ************************************************/

/*
 * I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */
// GPS(?)
// Smart Battery(?)
// Microchip 24xx256 Flash Memory
// A983 aka HMC5983 / HMC5883 Magnetic Compass
#define GPIO_I2C2_SCL		GPIO_I2C2_SCL_1		// PB10
#define GPIO_I2C2_SDA		GPIO_I2C2_SDA_1		// PB11
#define GPIO_I2C2_SCL_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN10)
#define GPIO_I2C2_SDA_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)


/*
 * SPI
 *
 * E_MISO, E_MOSI, E_SCK exposed on headers
 */
// MPU
#define GPIO_SPI1_MISO	(GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz)			// PA6
#define GPIO_SPI1_MOSI	(GPIO_SPI1_MOSI_1 | GPIO_SPEED_50MHz)			// PA7
#define GPIO_SPI1_SCK	(GPIO_SPI1_SCK_1 | GPIO_SPEED_50MHz)			// PA5
//#define GPIO_SPI1_MISO	(GPIO_SPI1_MISO_1)								// PA6
//#define GPIO_SPI1_MOSI	(GPIO_SPI1_MOSI_1)								// PA7
//#define GPIO_SPI1_SCK	(GPIO_SPI1_SCK_1)								// PA5
#define GPIO_SPI1_NSS	GPIO_SPI1_NSS_2									// PA4

// Baro
#define GPIO_SPI2_MISO	(GPIO_SPI2_MISO_1 | GPIO_SPEED_50MHz)			// PB14
#define GPIO_SPI2_MOSI	(GPIO_SPI2_MOSI_1 | GPIO_SPEED_50MHz)			// PB15
#define GPIO_SPI2_SCK	(GPIO_SPI2_SCK_2 | GPIO_SPEED_50MHz)			// PB13
//#define GPIO_SPI2_MISO	(GPIO_SPI2_MISO_1)								// PB14
//#define GPIO_SPI2_MOSI	(GPIO_SPI2_MOSI_1)								// PB15
//#define GPIO_SPI2_SCK	(GPIO_SPI2_SCK_2)								// PB13
#define GPIO_SPI2_NSS	GPIO_SPI2_NSS_1									// PB12


/*
 * UARTs.
 *
 */
// ttyS0 Optical Flow 115200
#define GPIO_USART2_RX	GPIO_USART2_RX_1	// PA3
#define GPIO_USART2_TX	GPIO_USART2_TX_1	// PA2

// ttyS1 WiFi AP 115200
// redundant / no alternate pin config available
//#define GPIO_UART5_RX	GPIO_UART5_RX		// PD2
//#define GPIO_UART5_TX	GPIO_UART5_TX		// PC12

// ttyS2 GPS 38400
#define GPIO_USART6_RX	GPIO_USART6_RX_1	// PC7
#define GPIO_USART6_TX	GPIO_USART6_TX_1	// PC6

#endif  /* __CONFIG_MOMENT_INCLUDE_BOARD_H */

