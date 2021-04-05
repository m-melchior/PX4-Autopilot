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

#include "SimToo_XT200.hpp"

#undef PX4_DEBUG
#define PX4_DEBUG PX4_INFO

int8_t SimToo_XT200::_current_instance_id = 1;

// **************************************************************************************************************
SimToo_XT200::SimToo_XT200(I2CSPIBusOption bus_option, int bus, uint32_t device, int bus_frequency, int address) :
	I2C(DRV_BAT_DEVTYPE_SIMTOO, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus) {

} // SimToo_XT200::SimToo_XT200



// **************************************************************************************************************
SimToo_XT200::~SimToo_XT200() {
	if (_batt_topic != nullptr) {
		orb_unadvertise(_batt_topic);
	}

	perf_free(_bad_transfers_perf);

	int battsource = 0;
	param_set(param_find("BAT_SOURCE"), &battsource);
} // SimToo_XT200::~SimToo_XT200() {



// **************************************************************************************************************
I2CSPIDriverBase *SimToo_XT200::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				       int runtime_instance)
{
	SimToo_XT200 *instance = new SimToo_XT200(iterator.configuredBusOption(), iterator.bus(), iterator.devid(),
			cli.bus_frequency, cli.i2c_address);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->start();

	return instance;
} // I2CSPIDriverBase *SimToo_XT200::instantiate



// **************************************************************************************************************
int SimToo_XT200::init() {
	int8_t _result = PX4_OK;

	_result = I2C::init();
	if (_result != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", _result);
		return _result;
	} // if (_result != PX4_OK) {

	_result = init_values();
	if (_result != PX4_OK) {
		return _result;
	} // if (_result != PX4_OK) {

	// todo: deal with greater than max cases
	if (_current_instance_id < SimToo_XT200_MAX_INSTANCES) {
		_instance_id = _current_instance_id;
		_current_instance_id++;
	} // if (_current_instance_id < SimToo_XT200_MAX_INSTANCES) {

	return _result;
} // int SimToo_XT200::init() {



// **************************************************************************************************************
void SimToo_XT200::RunImpl() {
//	int8_t _result = PX4_OK;

	_battery_status.timestamp = hrt_absolute_time();

	for (uint8_t _data_set_index = 0; _data_set_index < DATA_SETS_COUNT; _data_set_index++) {
		update_data_set(_data_set_index);
	} // for (uint8_t _data_set_index = 0; _data_set_index < DATA_SETS_COUNT; _data_set_index++) {


	if (_battery_status.remaining > _low_thr) {
		_battery_status.warning = battery_status_s::BATTERY_WARNING_NONE;

	} else if (_battery_status.remaining > _crit_thr) {
		_battery_status.warning = battery_status_s::BATTERY_WARNING_LOW;

	} else if (_battery_status.remaining > _emergency_thr) {
		_battery_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else {
		_battery_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
	}

	_battery_status.interface_error = perf_event_count(_bad_transfers_perf);

	orb_publish_auto(ORB_ID(battery_status), &_batt_topic, &_battery_status, &_instance_id);

	ScheduleDelayed(SimToo_XT200_MEASUREMENT_INTERVAL_HZ);
} // void SimToo_XT200::RunImpl() {



// **************************************************************************************************************
void SimToo_XT200::print_usage() {
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SimToo Smart Battery module.

### Examples

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("simtoo_smart_battery", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x0B);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
} // void SimToo_XT200::print_usage() {



// **************************************************************************************************************
void SimToo_XT200::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_bad_transfers_perf);

	printf("interval:  %u us\n", SimToo_XT200_MEASUREMENT_INTERVAL_HZ);
}



//// **************************************************************************************************************
//void SimToo_XT200::custom_method(const BusCLIArguments &cli) {
//	switch(cli.custom1) {
//		case CUSTOM_METHOD_MANUFACTURER: {
//			PX4_INFO("The manufacturer name: %s", _manufacturer_name);
//			PX4_INFO("The manufacturer date: %d", _manufacture_date);
//			PX4_INFO("The serial number: %d", _serial_number);
//		}
//		break;
//	} // switch(cli.custom1) {
//} // void SimToo_XT200::custom_method(const BusCLIArguments &cli) {



// **************************************************************************************************************
void SimToo_XT200::start()
{
//	ScheduleOnInterval(SimToo_XT200_MEASUREMENT_INTERVAL_HZ);
	ScheduleClear();
	ScheduleNow();
}



// **************************************************************************************************************
void SimToo_XT200::suspend()
{
	ScheduleClear();
}



// **************************************************************************************************************
void SimToo_XT200::resume()
{
	//	ScheduleOnInterval(SimToo_XT200_MEASUREMENT_INTERVAL_HZ);
		ScheduleNow();
}




// **************************************************************************************************************
int8_t SimToo_XT200::init_values() {
	int32_t _cell_count = 2;

	_battery_status = {};

	param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
	param_get(param_find("BAT_LOW_THR"), &_low_thr);
//	param_get(param_find("BAT1_N_CELLS"), &_cell_count);

	_battery_status.id = _instance_id;
	_battery_status.cell_count = _cell_count;
	_battery_status.connected = true;

	return PX4_OK;
} // int8_t SimToo_XT200::init_values() {



// **************************************************************************************************************
int8_t SimToo_XT200::update_data_set(uint8_t data_set_index_in) {
	int8_t _result = PX4_OK;

	uint8_t _register_address = 0;
	uint8_t _data_rx[2] = {0};
	uint16_t _value = 0;


	_register_address = data_sets[data_set_index_in].register_address;

	_result = transfer(&_register_address, 1, (uint8_t*)&_data_rx, 2);

	if (_result == PX4_OK) {
		_value = (_data_rx[1] << 8) + _data_rx[0];
		*(data_sets[data_set_index_in].battery_status_entry_ptr) = _value * data_sets[data_set_index_in].factor;

	} else { // if (_result == PX4_OK) {
		*(data_sets[data_set_index_in].battery_status_entry_ptr) = 0;
		perf_count(_bad_transfers_perf);

	} // } else { // if (_result == PX4_OK) {

	return _result;
} // int8_t SimToo_XT200::get_data_set(uint8_t data_set_index_in) {



//extern "C" __EXPORT int simtoo_xt200_main(int argc, char *argv[]) {
extern "C" int simtoo_xt200_main(int argc, char *argv[]) {
	using ThisDriver = SimToo_XT200;

	BusCLIArguments _cli{true, false};
	_cli.default_i2c_frequency = 100000;
	_cli.i2c_address = DEVICE_ADDRESS;

	const char *verb = _cli.parseDefaultArguments(argc, argv);
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, _cli, DRV_BAT_DEVTYPE_SIMTOO);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(_cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "man_info")) {
		_cli.custom1 = CUSTOM_METHOD_MANUFACTURER;
		return ThisDriver::module_custom_method(_cli, iterator, false);
	}


	ThisDriver::print_usage();
	return -1;
} // extern "C" __EXPORT int SimToo_XT200_main(int argc, char *argv[])










































