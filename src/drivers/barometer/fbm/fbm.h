/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file FBM88.h
 *
 * Shared defines for the FBM88 driver.
 */
#pragma once

#include <math.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>

#include "board_config.h"

// https://github.com/formosa-measurement-technology-inc

#define FBM_CHIP_ID  	0x42

/**
 * { I2C 7bit address setting for FBM }
 * 	If SDO pin is pulled low, I2C address is 6C.
 * 	If SDO pin is pulled high, I2C address is 6D.
 */
#define FBM_I2C_SLAVE_ADDR 0x6C

/* Define the oversampling rate setting of FBM.
 * Range of setting:
 * {osr_1024, osr_2048, osr_4096, osr_8192, osr_16384}
 */
 #define OVERSAMPLING_RATE_DEFAULT  osr_8192

/* Control registers address*/
// https://github.com/formosa-measurement-technology-inc/FMTI_fbm325_driver/blob/master/fbm325.h
#define FBM_SPI_CTRL_REG				0x00
#define FBM_SPI_CTRL_REG_SDO_ACTIVE_POS	(0)
#define FBM_SPI_CTRL_REG_SDO_ACTIVE_MSK	(0x81 << FBM_SPI_CTRL_REG_SDO_ACTIVE_POS)
#define FBM_SPI_CTRL_REG_SDO_ACTIVE_EN	(0x81 << FBM_SPI_CTRL_REG_SDO_ACTIVE_POS)
#define FBM_SPI_CTRL_REG_SDO_ACTIVE_DIS	(0 << << FBM_SPI_CTRL_REG_SDO_ACTIVE_POS)
#define FBM_TAKE_MEAS_REG				0xf4
#define FBM_READ_MEAS_REG_U				0xf6
#define FBM_READ_MEAS_REG_L				0xf7
#define FBM_READ_MEAS_REG_XL			0xf8
#define FBM_SOFTRESET_REG    			0xe0
#define FBM_CHIP_ID_REG	  				0x6b
#define FBM_VERSION_REG	  				0xa5
#define FBM_P_CONFIG_REG	  			0xa6
#define FBM_P_CONFIG_REG_GAIN_POS (3)
#define FBM_P_CONFIG_REG_GAIN_MAK (7 << FBM_P_CONFIG_REG_GAIN_POS)
#define FBM_P_CONFIG_REG_GAIN_X1 (0 << FBM_P_CONFIG_REG_GAIN_POS)
#define FBM_P_CONFIG_REG_GAIN_X2 (1 << FBM_P_CONFIG_REG_GAIN_POS)
#define FBM_P_CONFIG_REG_GAIN_X4 (2 << FBM_P_CONFIG_REG_GAIN_POS)
#define FBM_P_CONFIG_REG_GAIN_X8 (3 << FBM_P_CONFIG_REG_GAIN_POS)
#define FBM_P_CONFIG_REG_GAIN_X16 (4 << FBM_P_CONFIG_REG_GAIN_POS)
#define FBM_P_CONFIG_REG_GAIN_X32 (5 << FBM_P_CONFIG_REG_GAIN_POS)
#define FBM_P_CONFIG_REG_GAIN_X64 (6 << FBM_P_CONFIG_REG_GAIN_POS)
#define FBM_P_CONFIG_REG_GAIN_X128 (7 << FBM_P_CONFIG_REG_GAIN_POS)

/* CMD list */
#define FBM_MEAS_TEMP		        0x2e /* 2.5ms wait for measurement */
#define FBM_MEAS_PRESS_OVERSAMP_0	0x34 /* 2.5ms wait for measurement */
#define FBM_MEAS_PRESS_OVERSAMP_1	0x74 /* 3.7ms wait for measurement */
#define FBM_MEAS_PRESS_OVERSAMP_2	0xb4 /* 6ms wait for measurement */
#define FBM_MEAS_PRESS_OVERSAMP_3	0xf4 /* 10.7ms wait for measurement */
#define FBM_SOFTRESET_CMD			0xb6

#define FBM_CONVERSION_usTIME_OSR1024   2500  /*us*/
#define FBM_CONVERSION_usTIME_OSR2048   3700  /*us*/
#define FBM_CONVERSION_usTIME_OSR4096   6000  /*us*/
#define FBM_CONVERSION_usTIME_OSR8192   10700 /*us*/
#define FBM_CONVERSION_usTIME_OSR16384   20500 /*us*/

/* Calibration registers */
#define FPM_CALIB_DATA_ADDR				0xaa /* Calibraton data address
                                      	       * {0xf1, 0xd0, 0xbb:0xaa} */
#define FBM_CALIBRATION_DATA_START1		0xd0
#define FBM_CALIBRATION_DATA_START2		0xf1
#define FBM_CALIBRATION_DATA_LENGTH		20 /* bytes */

#define FBM_SPI_WRITE 0x00
#define FBM_SPI_READ 0x80

enum fbm320_osr {
	osr_1024 = 0x0,
	osr_2048 = 0x1,
	osr_4096 = 0x2,
	osr_8192 = 0x3,
	osr_16384 = 0x4
};

enum fbm320_hw_version {
	hw_ver_b1 = 0x0,
	hw_ver_b2 = 0x1,
	hw_ver_b3 = 0x3,
	hw_ver_b4 = 0x5,
    hw_ver_b5 = 0x6,
	hw_ver_unknown = 0xFF
};

#pragma pack(push,1)
struct fbm320_reg_calibration_data {
		int32_t C0;
		int32_t C1;
		int32_t C2;
		int32_t C3;
		int32_t C4;
		int32_t C5;
		int32_t C6;
		int32_t C7;
		int32_t C8;
		int32_t C9;
		int32_t C10;
		int32_t C11;
		int32_t C12;
		int32_t C13;
};
#pragma pack(pop)

struct fbm320_data {
	enum fbm320_osr oversampling_rate;
	struct fbm320_calibration_data calibration;
	enum  fbm320_hw_version hw_ver;
	uint8_t chip_id;
	uint8_t cmd_start_p;
	uint8_t cmd_start_t;
	uint32_t cnvTime_temp;					//unit:us
	uint32_t cnvTime_press;					//unit:us
	uint32_t raw_temperature;
	uint32_t raw_pressure;
	int32_t real_temperature;				//unit:0.01 degree Celsius
	int32_t real_pressure;					//unit: Pa
};




/*
 * FBM88 internal constants and data structures.
 */
class IFBM88
{
public:
	virtual ~IFBM88() = default;

	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// bulk read reg value
	virtual int get_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	// bulk read of calibration data into buffer, return same pointer
	virtual calibration_s *get_calibration(uint8_t addr) = 0;

	virtual uint32_t get_device_id() const = 0;

	virtual uint8_t get_device_address() const = 0;
};



class FBM88 : public I2CSPIDriver<FBM88>
{
public:
	FBM88(I2CSPIBusOption bus_option, int bus, IFBM88 *interface);
	virtual ~FBM88();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);

	static void print_usage();

	virtual int		init();

	void			print_status();

	void 			RunImpl();
private:
	PX4Barometer	_px4_baro;
	IFBM88			*_interface{nullptr};

	unsigned		_measure_interval{0};			// interval in microseconds needed to measure

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	calibration_s		*_cal {nullptr}; // stored calibration constants

	bool			_collect_phase{false};

	void 			start();
	int 			measure();
	int				collect(); //get results and publish
	uint32_t		get_measurement_time();

	bool			soft_reset();
	bool			check_chip_id()
	bool			get_calib_data();
	bool 			set_sensor_settings();
	bool			set_op_mode(uint8_t op_mode);

	bool 			get_sensor_data(uint8_t sensor_comp, struct FBM_data *comp_data);
	bool			compensate_data(uint8_t sensor_comp, const struct FBM_uncomp_data *uncomp_data, struct FBM_data *comp_data);
};


/* interface factories */
extern IFBM88 *FBM88_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
extern IFBM88 *FBM88_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
