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
 * @file SimToo_SmartBattery_I2C
 *
 * I2C interface for SimToo SmartBattery
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/device/i2c.h>

#include "simtoo_xt200.h"

class SimToo_SmartBattery_I2C : public device::I2C
{
public:
	SimToo_SmartBattery_I2C(int bus, int bus_frequency);
	virtual ~SimToo_SmartBattery_I2C() = default;

	virtual int	read(unsigned address, void *data, unsigned count);

	int get_voltage_1(uint16_t *value_out);
	int get_voltage_2(uint16_t *value_out);
	int get_status(uint16_t *value_out)
	int get_temperature(uint16_t *value_out)
	int get_voltage_cell_1(uint16_t *value_out)
	int get_voltage_cell_2(uint16_t *value_out)
	int get_max_charge_voltage(uint16_t *value_out)
	int get_rated_capacity(uint16_t *value_out)
	int get_state_of_charge(uint16_t *value_out)
	int get_remaining_capacity(uint16_t *value_out)
};

int SimToo_SmartBattery_I2C::read_uint16(uint8_t register_address_in, void *data_out, uint8_t byte_count_in)
{
	uint8_t _cmd = register_address_in;
	return transfer(&_cmd, 1, (uint8_t *)data_out, byte_count_in);
}



int SimToo_SmartBattery_I2C::get_voltage_1(uint16_t *value_out)
{
	uint8_t _register_address = REGISTER_ADDRESS_VOLTAGE_1;
	return read_uint16(&_register_address, 1, (uint8_t *)value_out, 2);
}



int SimToo_SmartBattery_I2C::get_voltage_2(uint16_t *value_out)
{
	uint8_t _register_address = REGISTER_ADDRESS_VOLTAGE_2;
	return read_uint16(&_register_address, 1, (uint8_t *)value_out, 2);
}









































