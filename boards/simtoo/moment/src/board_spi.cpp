/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

//#include "board_spi.h"

//static constexpr bool unused = validateSPIConfig(px4_spi_buses);

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;


// SPI_BUS_MAX_BUS_ITEMS going back to platforms/nuttx/NuttX//nuttx/arch/arm/include/stm32/chip.h STM32_NSPI
constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_MPU6500, SPI::CS{GPIO::PortA, GPIO::Pin4}, SPI::DRDY{GPIO::PortC, GPIO::Pin13}),
	}),

	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(DRV_BARO_DEVTYPE_FBM, SPI::CS{GPIO::PortB, GPIO::Pin12}),
	}),
}; // constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS]



void board_spi_init_hardware(void)
{
	stm32_spiinitialize();
} // board_spi_init_hardware



int board_spi_init_interface(void)
{
	spi1 = stm32_spibus_initialize(1);

	if (!spi1) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 1\n");
		return -ENODEV;
	}

	// Default SPI1 to 10MHz
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	up_udelay(20);


	spi2 = stm32_spibus_initialize(2);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 2\n");
		return -ENODEV;
	}

	// Default SPI2 to 1MHz
	SPI_SETFREQUENCY(spi2, 10000000);
	SPI_SETBITS(spi2, 8);
	// setting mode0 here seems to be overridden, needs to be stated in device_main again!
	SPI_SETMODE(spi2, SPIDEV_MODE0);

	return OK;
} // int board_spi_init_interface(void)



static constexpr bool unused = validateSPIConfig(px4_spi_buses);









































