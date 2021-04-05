/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file FBM.cpp
 *
 * Driver for the FBM barometric pressure sensor connected via SPI or I2C
 *
 * Refer to: https://github.com/formosa-measurement-technology-inc
 */

#include "FBM.h"

FBM::FBM(I2CSPIBusOption bus_option, int bus, IFBM *interface) :
	I2CSPIDriver(	MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()),
					bus_option, bus,
					interface->get_device_address()),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
} // FBM::FBM(I2CSPIBusOption bus_option, int bus, IFBM *interface)



FBM::~FBM() {
	/* free perf counters */
	perf_free (_sample_perf);
	perf_free (_measure_perf);
	perf_free (_comms_errors);

	delete _interface;
} // FBM::~FBM()



int FBM::init() {
	bool _result = false;

	_result = soft_reset();
	if(_result != true) {
		PX4_WARN("FBM failed to soft reset");
		return -EIO;
	} // if(_result != true) {

	_result = check_chip_id();
	if(_result != true) {
		PX4_WARN("FBM failed to soft reset");
		return -EIO;
	} // if(_result != true) {

	_cal = _interface->get_calibration(FPM_CALIB_DATA_ADDR);

	if (!_cal) {
		PX4_WARN("FBM failed to get baro cal init");
		return -EIO;
	} // if (!_cal) {


	set_sensor_settings();

	start();

	return OK;
} // int FBM::init()



void FBM::print_status() {
	I2CSPIDriverBase::print_status();
	perf_print_counter (_sample_perf);
	perf_print_counter (_measure_perf);
	perf_print_counter (_comms_errors);
	printf("measurement interval:  %u us \n", _measure_interval);
} // void FBM::print_status()



void FBM::start() {
	_collect_phase = false;

	// wait a bit longer for the first measurement, as otherwise the first readout might fail
	ScheduleOnInterval(_measure_interval, _measure_interval * 3);
} // void FBM::start() {



void FBM::RunImpl() {
	if (_collect_phase) {
		collect();
	}

	measure();
} // void FBM::RunImpl()



/*!
 * @brief This API checks for the Chip ID
 *
 */
bool FBM::soft_reset() {
	bool _result = false;

	// todo: check behavior and requirement for clearing the register again
	_result = _interface->set_reg(1, FBM_SOFTRESET_REG);

	return _result;
} // bool soft_reset() {



/*!
 * @brief This API performs the soft reset of the sensor.
 *
 */
bool FBM::check_chip_id() {
	bool _result = false;
	int _chip_id = 0;

	_chip_id = _interface->get_reg(FPM_CHIP_ID_ADDR)

	if (_chip_id == FPM_CHIP_ID) {
		_result = true;
	} else {
		PX4_WARN("Unexpected Chip ID: 0x%02x instead of 0x%02x", _chip_id, FPM_CHIP_ID);
		_result = false;
	}

	return _result;
} // bool check_chip_id() {







































