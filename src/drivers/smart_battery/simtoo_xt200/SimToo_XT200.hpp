/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file SimToo_XT200.hpp
 *
 * Driver for the SimToo XT200 Smart Battery connected via I2C.
 * For more info about the members of battery_status, check /msg/battery_status.msg
 *
 */

#pragma once

#include "simtoo_xt200.h"

//#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
//#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
//#include <px4_platform_common/time.h>
#include <px4_platform_common/param.h>
//#include <px4_platform_common/px4_config.h>
#include <uORB/topics/battery_status.h>


// at the moment only one instance of the simtoo batterie is working, as they share the same device address
#define SimToo_XT200_MAX_INSTANCES				1
#define SimToo_XT200_MEASUREMENT_INTERVAL_HZ	1000000 / 2
#define CUSTOM_METHOD_MANUFACTURER				0
#define NUM_OF_CELLS							2

#define DATA_SETS_COUNT 						4 // number of data sets to update each cycle

class SimToo_XT200 : public device::I2C, public I2CSPIDriver<SimToo_XT200> {
public:
	// count up instance IDs for multiple batteries
	static int8_t _current_instance_id;

	SimToo_XT200(I2CSPIBusOption bus_option, int bus, uint32_t device, int bus_frequency, int address);
	~SimToo_XT200();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
						     int runtime_instance);

	int init();

	void start();
	void suspend();
	void resume();

	void RunImpl();

	static void print_usage();
	void print_status() override;
//	void custom_method(const BusCLIArguments &cli) override;

private:
	int _instance_id;
	battery_status_s _battery_status;

	// the data requested from the battery is organized in a struct and array for iteration
	struct data_set {
			float *battery_status_entry_ptr;
			uint16_t register_address;
			float factor;
	};

	data_set data_sets[DATA_SETS_COUNT] = {
			{&_battery_status.voltage_v, REGISTER_ADDRESS_VOLTAGE_1, 1/1000},
			{&_battery_status.voltage_cell_v[0], REGISTER_ADDRESS_VOLTAGE_CELL_1, (1 / 1000.0f)},
			{&_battery_status.voltage_cell_v[1], REGISTER_ADDRESS_VOLTAGE_CELL_2, (1 / 1000.0f)},
			{&_battery_status.remaining, REGISTER_ADDRESS_STATE_OF_CHARGE, (1 / 100.0f)},
	};

	int8_t init_values();

	int8_t update_data_set(uint8_t data_set_index_in);

	// CubeIDE is showing a syntax error here, but it is the same as in other modules and compiles alright
	perf_counter_t _bad_transfers_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};

	/** @param _batt_topic uORB battery topic. */
	orb_advert_t _batt_topic{nullptr};;

	/** @param _crit_thr Critical battery threshold param. */
	float _crit_thr{0.f};

	/** @param _emergency_thr Emergency battery threshold param. */
	float _emergency_thr{0.f};

	/** @param _low_thr Low battery threshold param. */
	float _low_thr{0.f};

}; // class SimToo_XT200 : public I2CSPIDriver<SimToo_XT200> {







































