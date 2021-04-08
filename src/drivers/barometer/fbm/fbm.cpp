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
 * @file FBM.cpp
 *
 * Driver for the FBM barometric pressure sensor connected via SPI or I2C
 *
 * Refer to: https://github.com/formosa-measurement-technology-inc
 */

#include "fbm.h"

FBM::FBM(I2CSPIBusOption bus_option, int bus, IFBM *interface) :
	I2CSPIDriver(	MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()),
					bus_option, bus,
					interface->get_device_address()),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_errors_comms(perf_alloc(PC_ELAPSED, MODULE_NAME": comms errors")),
	_perf_process(perf_alloc(PC_ELAPSED, MODULE_NAME": process"))
{
} // FBM::FBM(I2CSPIBusOption bus_option, int bus, IFBM *interface)



// ************************************************************************
FBM::~FBM() {
	/* free perf counters */
	perf_free (_errors_comms);
	perf_free (_perf_process);

	delete _interface;
} // FBM::~FBM()



// ************************************************************************
int8_t FBM::init() {
	int8_t _result = PX4_OK;

	_result = _interface->init();
	if(_result != PX4_OK) {
		PX4_WARN("FBM init interface failed");
		return -EIO;
	} // if(_result != PX4_OK) {

	_result = _interface->get_calibration_data(&_calibration_data);
	if(_result != PX4_OK) {
		PX4_WARN("FBM get_calibration_data failed");
		return -EIO;
	} // if(_result != PX4_OK) {

	_set_oversampling_rate(OVERSAMPLING_RATE_DEFAULT);

	_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_FBM);

	_start();

	return _result;
} // int FBM::init()



// ************************************************************************
void FBM::print_status() {
	I2CSPIDriverBase::print_status();

	perf_print_counter (_errors_comms);
	perf_print_counter (_perf_process);

//	printf("measurement interval:  %u us \n", _measure_interval);
} // void FBM::print_status()



// ************************************************************************
void FBM::_start() {
	ScheduleClear();
	_state = FBM_STATE_START;
	ScheduleNow();
} // void FBM::start() {



// ************************************************************************
void FBM::RunImpl() {
	int8_t _result = PX4_OK;
	uint8_t _buffer_rx[3] = {0};

	switch(_state) {
		case FBM_STATE_IDLE:
			break;

		case FBM_STATE_START:
			_result = _interface->write(FBM_TAKE_MEAS_REG, 1, &_cmd_start_temperature);
			if (_result != PX4_OK) {
				perf_count(_errors_comms);
				ScheduleDelayed(FBM_RECOVER_FROM_ERROR_usTIME);
				break;
			}

			_state = FBM_STATE_CONVERTING_TEMPERATURE;
			ScheduleDelayed(_cnvTime_temperature);
			break;

		case FBM_STATE_CONVERTING_TEMPERATURE:
			_result = _interface->read(FBM_READ_MEAS_REG_U, 3, _buffer_rx);
			if (_result != PX4_OK) {
				perf_count(_errors_comms);
				_state = FBM_STATE_START;
				ScheduleDelayed(FBM_RECOVER_FROM_ERROR_usTIME);
				break;
			}

			_raw_temperature = ((uint32_t)_buffer_rx[2] << 16) + ((uint32_t)_buffer_rx[1] << 8) + _buffer_rx[0];

			_result = _interface->write(FBM_TAKE_MEAS_REG, 1, &_cmd_start_pressure);
			if (_result != PX4_OK) {
				perf_count(_errors_comms);
				_state = FBM_STATE_START;
				ScheduleDelayed(FBM_RECOVER_FROM_ERROR_usTIME);
				break;
			}

			_state = FBM_STATE_CONVERTING_PRESSURE;
			ScheduleDelayed(_cnvTime_pressure);
			break;

		case FBM_STATE_CONVERTING_PRESSURE:
			_result = _interface->read(FBM_READ_MEAS_REG_U, 3, _buffer_rx);
			if (_result != PX4_OK) {
				perf_count(_errors_comms);
				_state = FBM_STATE_START;
				ScheduleDelayed(FBM_RECOVER_FROM_ERROR_usTIME);
				break;
			}

			_raw_pressure = ((uint32_t)_buffer_rx[2] << 16) + ((uint32_t)_buffer_rx[1] << 8) + _buffer_rx[0];

			_process();

			const hrt_abstime _timestamp = hrt_absolute_time();

			_px4_baro.set_error_count(perf_event_count(_errors_comms));
			_px4_baro.set_temperature(_real_temperature);
			_px4_baro.update(_timestamp, _real_pressure / 100.0f);

			_state = FBM_STATE_START;

			ScheduleDelayed(0);
			break;
	} // switch(_state) {
} // void FBM::RunImpl()



// ********************************************************************************
void FBM::_process() {
	perf_begin(_perf_process);

	int32_t X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32;
	int32_t PP1, PP2, PP3, PP4, CF;
	int32_t RT, RP, UT, UP, DT, DT2;

	UT = _raw_temperature;
	DT = ((UT - 8388608) >> 4) + (_calibration_data.C0 << 4);
	X01 = (_calibration_data.C1 + 4459) * DT >> 1;
	X02 = ((((_calibration_data.C2 - 256) * DT) >> 14) * DT) >> 4;
	X03 = (((((_calibration_data.C3 * DT) >> 18) * DT) >> 18) * DT);
	RT = ((2500 << 15) - X01 - X02 - X03) >> 15;

	DT2 = (X01 + X02 + X03) >> 12;
	X11 = ((_calibration_data.C5 - 4443) * DT2);
	X12 = (((_calibration_data.C6 * DT2) >> 16) * DT2) >> 2;
	X13 = ((X11 + X12) >> 10) + ((_calibration_data.C4 + 120586) << 4);

	X21 = ((_calibration_data.C8 + 7180) * DT2) >> 10;
	X22 = (((_calibration_data.C9 * DT2) >> 17) * DT2) >> 12;
	X23 = abs(X22 - X21);
	X24 = (X23 >> 11) * (_calibration_data.C7 + 166426);
	X25 = ((X23 & 0x7FF) * (_calibration_data.C7 + 166426)) >> 11;
	X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + _calibration_data.C7 + 166426) : (((X24 + X25) >> 11) + _calibration_data.C7 + 166426);

	UP = _raw_pressure;
	PP1 = ((UP - 8388608) - X13) >> 3;
	PP2 = (X26 >> 11) * PP1;
	PP3 = ((X26 & 0x7FF) * PP1) >> 11;
	PP4 = (PP2 + PP3) >> 10;

	CF = (2097152 + _calibration_data.C12 * DT2) >> 3;
	X31 = (((CF * _calibration_data.C10) >> 17) * PP4) >> 2;
	X32 = (((((CF * _calibration_data.C11) >> 15) * PP4) >> 18) * PP4);
	RP = ((X31 + X32) >> 15) + PP4 + 99880;

	_real_temperature = RT;
	_real_pressure = RP;

	perf_end(_perf_process);
} // uint8_t FBM::process() {



// ********************************************************************************
uint8_t FBM::_set_oversampling_rate(enum fbm_osr osr_setting) {
	uint8_t _address;
	uint8_t _buffer;

//	oversampling_rate = osr_setting;

	/* Setting conversion time for pressure measurement */
	switch (osr_setting) {
	case osr_1024:
		_cnvTime_pressure = FBM_CONVERSION_usTIME_OSR1024;
		_cmd_start_pressure = FBM_MEAS_PRESS_OVERSAMP_0;
		break;
	case osr_2048:
		_cnvTime_pressure = FBM_CONVERSION_usTIME_OSR2048;
		_cmd_start_pressure = FBM_MEAS_PRESS_OVERSAMP_1;
		break;
	case osr_4096:
		_cnvTime_pressure = FBM_CONVERSION_usTIME_OSR4096;
		_cmd_start_pressure = FBM_MEAS_PRESS_OVERSAMP_2;
		break;
	case osr_8192:
		_cnvTime_pressure = FBM_CONVERSION_usTIME_OSR8192;
		_cmd_start_pressure = FBM_MEAS_PRESS_OVERSAMP_3;
		break;
	case osr_16384:
		_cnvTime_pressure = FBM_CONVERSION_usTIME_OSR16384;

		_address = 0xa6;

		_interface->read(_address, sizeof(uint8_t), &_buffer);

		_buffer &= 0xf8;
		_buffer |= 0x6;

		_interface->write(_address, sizeof(uint8_t), &_buffer);

		_cmd_start_pressure = FBM_MEAS_PRESS_OVERSAMP_2;
		_interface->read(0xA6, sizeof(uint8_t), &_buffer);
		break;
	}

	/* Setting covversion time for temperature measurement */
	_cnvTime_temperature = FBM_CONVERSION_usTIME_OSR1024;

	return PX4_OK;
}







































