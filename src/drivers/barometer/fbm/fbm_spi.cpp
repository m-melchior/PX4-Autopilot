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
 * @file FBM_spi.cpp
 *
 * SPI interface for FBM (NOTE: untested!)
 */

#include <drivers/device/spi.h>

#include "fbm.h"


class FBM_SPI: public device::SPI, public IFBM {
public:
	FBM_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	virtual ~FBM_SPI() = default;

	int init();

	int8_t read(uint8_t addr, uint8_t length, uint8_t *buffer);
	int8_t write(uint8_t addr, uint8_t length, uint8_t *buffer);

	int8_t set_cs();
	int8_t check_chip_id();
	int8_t reset();
	int8_t get_calibration_data(fbm_calibration_data *calibration_data);

	uint32_t get_device_id() const override { return device::SPI::get_device_id(); }

	uint8_t get_device_address() const override { return device::SPI::get_device_address(); }
}; // class FBM_SPI: public device::SPI, public IFBM {



// ************************************************************************
IFBM *FBM_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode) {
	return new FBM_SPI(busnum, device, bus_frequency, spi_mode);
} // IFBM *FBM_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode) {



// ************************************************************************
FBM_SPI::FBM_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_BARO_DEVTYPE_FBM, MODULE_NAME, bus, device, spi_mode, bus_frequency) {
} // FBM_SPI::FBM_SPI



// ************************************************************************
int FBM_SPI::init() {
	int _result = PX4_OK;

	_result = SPI::init();
	if(_result != PX4_OK) {
		PX4_ERR("FBM SPI init failed");
		return -EIO;
	} // if(_result != PX4_OK) {

	_result = set_cs();
	if(_result != PX4_OK) {
		PX4_ERR("FBM set_cs failed");
		return -EIO;
	} // if(_result != PX4_OK) {

	_result = check_chip_id();
	if(_result != PX4_OK) {
		PX4_ERR("FBM wrong chip id");
		return -EIO;
	} // if(_result != PX4_OK) {

	return _result;
} // int8_t FBM_SPI::init() {



// ************************************************************************
int8_t FBM_SPI::read(uint8_t addr, uint8_t length, uint8_t *buffer) {
	int8_t _result = PX4_OK;

	uint8_t _cmd = 0;
	uint8_t _buffer[length + 2] = {0};


	switch (length) {
		case 1:
			_cmd = FBM_SPI_READ | FBM_SPI_1BYTE;
			break;
		case 2:
			_cmd = FBM_SPI_READ | FBM_SPI_2BYTE;
			break;
		case 3:
			_cmd = FBM_SPI_READ | FBM_SPI_3BYTE;
			break;
		default:
			_cmd = FBM_SPI_READ | FBM_SPI_4BYTE;
	}

	_buffer[0] = _cmd;
	_buffer[1] = addr + (length - 1);

	_result = transfer((uint8_t*)&_buffer, (uint8_t*)&_buffer, length + 2);

	if (_result != PX4_OK) {
		return _result;
	}

	memcpy (buffer, &_buffer[2], length);

	return _result;
} // int8_t FBM_SPI::read(uint8_t addr, uint8_t length, uint8_t *buffer) {



//// ************************************************************************
//int8_t FBM_SPI::read(uint8_t addr, uint8_t length, uint8_t *buffer) {
//	int8_t _result = PX4_OK;
//
//	uint8_t _buffer_tx[2] = {0};
//	uint8_t _cmd = 0;
//	uint8_t _tmp[length];
//	memset(&_tmp, 0xAA, length);
//
//
//	switch (length) {
//		case 1:
//			_cmd = FBM_SPI_READ | FBM_SPI_1BYTE;
//			break;
//		case 2:
//			_cmd = FBM_SPI_READ | FBM_SPI_2BYTE;
//			break;
//		case 3:
//			_cmd = FBM_SPI_READ | FBM_SPI_3BYTE;
//			break;
//		default:
//			_cmd = FBM_SPI_READ | FBM_SPI_4BYTE;
//	}
//
//	_buffer_tx[0] = _cmd;
//	_buffer_tx[1] = addr + (length - 1);
//
//	_result = transfer((uint8_t*)&_buffer_tx, nullptr, 2);
//	if (_result != PX4_OK) {
//		return _result;
//	}
//
////	usleep(3);
//
//	_result = transfer((uint8_t*)&_tmp, buffer, length);
//	if (_result != PX4_OK) {
//		return _result;
//	}
//
//	return _result;
//} // int8_t FBM_SPI::read(uint8_t addr, uint8_t length, uint8_t *buffer) {



// ************************************************************************
int8_t FBM_SPI::write(uint8_t addr, uint8_t length, uint8_t *buffer) {
	int8_t _result = PX4_OK;

	uint8_t _buffer[length + 2];
	uint8_t _cmd = 0;

	memset(&_buffer, 0, length + 2);

	switch (length) {
		case 1:
			_cmd = FBM_SPI_WRITE | FBM_SPI_1BYTE;
			break;
		case 2:
			_cmd = FBM_SPI_WRITE | FBM_SPI_2BYTE;
			break;
		case 3:
			_cmd = FBM_SPI_WRITE | FBM_SPI_3BYTE;
			break;
		default:
			_cmd = FBM_SPI_WRITE | FBM_SPI_4BYTE;
	}

	_buffer[0] = _cmd;
	_buffer[1] = addr + length - 1;
	memcpy(&_buffer[2], buffer, length);

	_result = transfer(&_buffer[0], nullptr, length + 2);

	if (_result != PX4_OK) {
		return _result;
	}

	return _result;
} // int8_t FBM_SPI::write(uint8_t addr, uint8_t length, uint8_t *buffer) {



// ************************************************************************
int8_t FBM_SPI::get_calibration_data(struct fbm_calibration_data *calibration_data) {
	uint8_t _result = PX4_OK;

	uint16_t _values[10] = {0};
	uint8_t _read_buffer[2] = {0};
	uint8_t _parameter_index = 0;


	for (_parameter_index = 0; _parameter_index < 9; _parameter_index++) {
//		_result = _fbm_interface.bus_read(FBM_CALIBRATION_DATA_START0 + (_parameter_index * 2), 1, &_read_buffer[0]);
		_result = read(FBM_CALIBRATION_DATA_START0 + (_parameter_index * 2), 1, &_read_buffer[0]);
		if (_result != PX4_OK) {
			return _result;
		}

//		_result = _fbm_interface.bus_read(FBM_CALIBRATION_DATA_START1 + (_parameter_index * 2), 1, &_read_buffer[1]);
		_result = read(FBM_CALIBRATION_DATA_START1 + (_parameter_index * 2), 1, &_read_buffer[1]);
		if (_result != PX4_OK) {
			return _result;
		}

		_values[_parameter_index] = ((uint8_t)_read_buffer[0] << 8 | _read_buffer[1]);
	} // for (_parameter_index = 0; _parameter_index < 9; _parameter_index++) {

//	_result = _fbm_interface.bus_read(FBM_CALIBRATION_DATA_START2, 1, &_read_buffer[0]);
	_result = read(FBM_CALIBRATION_DATA_START2, 1, &_read_buffer[0]);
	if (_result != PX4_OK) {
		return _result;
	}

//	_result = _fbm_interface.bus_read(FBM_CALIBRATION_DATA_START3, 1, &_read_buffer[1]);
	_result = read(FBM_CALIBRATION_DATA_START3, 1, &_read_buffer[1]);
	if (_result != PX4_OK) {
		return _result;
	}

	_values[9] = ((uint8_t)_read_buffer[0] << 8 | _read_buffer[1]);

	calibration_data->C0 = _values[0] >> 4;
	calibration_data->C1 = ((_values[1] & 0xFF00) >> 5) | (_values[2] & 7);
	calibration_data->C2 = ((_values[1] & 0xFF) << 1) | (_values[4] & 1);
	calibration_data->C3 = _values[2] >> 3;
	calibration_data->C4 = ((uint32_t)_values[3] << 2) | (_values[0] & 3);
	calibration_data->C5 = _values[4] >> 1;
	calibration_data->C6 = _values[5] >> 3;
	calibration_data->C7 = ((uint32_t)_values[6] << 3) | (_values[5] & 7);
	calibration_data->C8 = _values[7] >> 3;
	calibration_data->C9 = _values[8] >> 2;
	calibration_data->C10 = ((_values[9] & 0xFF00) >> 6) | (_values[8] & 3);
	calibration_data->C11 = _values[9] & 0xFF;
	calibration_data->C12 = ((_values[0] & 0x0C) << 1) | (_values[7] & 7);

	return _result;
} // int8_t FBM_SPI::get_calibration_data(fbm_calibration_data *calibration_data) {



// ************************************************************************
int8_t FBM_SPI::set_cs() {
	uint8_t _result = PX4_OK;
	uint8_t _value_tx;

	_value_tx = FBM_SPI_CTRL_REG_SDO_ACTIVE_EN;
	_result = write(FBM_SPI_CTRL_REG, 1, &_value_tx);

	if (_result != PX4_OK) {
		return _result;
	}

	return _result;
} // int8_t FBM_SPI::init_cs() {



// ************************************************************************
int8_t FBM_SPI::check_chip_id() {
	uint8_t _result = PX4_OK;
	uint8_t _value_rx;

	_result = read(FBM_CHIP_ID_REG, 1, &_value_rx);

	if (_result != PX4_OK) {
		return _result;
	}

	if (_value_rx != FBM_CHIP_ID) {
		return PX4_ERROR;
	}

	return _result;
} // int8_t FBM_SPI::check_chip_id() {



// ************************************************************************
int8_t FBM_SPI::reset() {
	uint8_t _result = PX4_OK;
	uint8_t _cmd = 0;

	_cmd = FBM_SOFTRESET_CMD;
	write(FBM_SOFTRESET_REG, sizeof(uint8_t), &_cmd);
	if (_result != PX4_OK) {
		return _result;
	}

	return _result;
} // int8_t FBM_SPI::reset() {










































