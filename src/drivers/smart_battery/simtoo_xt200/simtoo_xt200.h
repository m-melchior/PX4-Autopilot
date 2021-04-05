/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file simtoo_smartbattery.h
 *
 * SimToo XT200 smart battery registers.
 *
 */

#pragma once

#include <lib/drivers/device/Device.hpp>

#define DEVICE_ADDRESS							0x0B;

#define REGISTER_ADDRESS_VOLTAGE_1				0x01
#define REGISTER_ADDRESS_VOLTAGE_2				0x02
#define REGISTER_ADDRESS_STATUS					0x03
#define REGISTER_ADDRESS_TEMPERATURE			0x04
#define REGISTER_ADDRESS_VOLTAGE_CELL_1			0x0C
#define REGISTER_ADDRESS_VOLTAGE_CELL_2			0x0D
#define REGISTER_ADDRESS_MAX_CHARGE_VOLTAGE		0x10
#define REGISTER_ADDRESS_RATED_CAPACITY			0x11
#define REGISTER_ADDRESS_STATE_OF_CHARGE		0x12
#define REGISTER_ADDRESS_REMAINING_CAPACITY		0x13
#define REGISTER_ADDRESS_CHIP_MANUFACTURER		0x21
#define REGISTER_ADDRESS_PRODUCT_NAME			0x23
#define REGISTER_ADDRESS_CELL_TYPE				0x24

//extern device::Device *SimToo_SmartBattery_I2C_interface(int bus, int bus_frequency);






































