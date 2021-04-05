px4_add_board(
	PLATFORM nuttx
	VENDOR simtoo
	MODEL moment
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT px4fmu_common
#	CONSTRAINED_FLASH
	SERIAL_PORTS
		GPS1:/dev/ttyS2
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS0
	DRIVERS
		#barometer/bmp280
		gps
		imu/invensense/mpu6500
		magnetometer/hmc5883
		#optical_flow/pmw3901
		#distance_sensor/vl53l1x
		pwm_out
		smart_battery/simtoo_xt200
	MODULES
#		airspeed_selector
#		attitude_estimator_q
#-		battery_status
		commander						# https:#dev.px4.io/v1.9.0/en/middleware/modules_system.html#commander
		dataman							# https:#dev.px4.io/v1.9.0/en/middleware/modules_system.html#dataman
		ekf2							# https:#dev.px4.io/v1.9.0/en/middleware/modules_estimator.html#ekf2
#		events
		flight_mode_manager				# https:#dev.px4.io/v1.9.0/en/middleware/modules_controller.html#flight_mode_manager
#		gyro_calibration				# ?
#		gyro_fft			
		land_detector					# https:#dev.px4.io/v1.9.0/en/middleware/modules_controller.html#land_detector
		landing_target_estimator		# ? required for local_position_estimator
#		load_mon						# https:#dev.px4.io/v1.9.0/en/middleware/modules_system.html#load_mon
		local_position_estimator		# https:#dev.px4.io/v1.9.0/en/middleware/modules_system.html#local_position_estimator
#-		logger							# https:#dev.px4.io/v1.9.0/en/middleware/modules_system.html#logger
		mavlink							# https:#dev.px4.io/v1.9.0/en/middleware/modules_communication.html#mavlink
		mc_att_control					# https:#dev.px4.io/v1.9.0/en/middleware/modules_controller.html#mc_att_control
		mc_hover_thrust_estimator		# https:#dev.px4.io/v1.9.0/en/middleware/modules_estimator.html#mc_hover_thrust_estimator
		mc_pos_control					# https:#dev.px4.io/v1.9.0/en/middleware/modules_controller.html#mc_pos_control
		mc_rate_control					# https:#dev.px4.io/v1.9.0/en/middleware/modules_controller.html#mc_rate_control
		navigator						# https:#dev.px4.io/v1.9.0/en/middleware/modules_controller.html#navigator
		sensors							# https:#dev.px4.io/v1.9.0/en/middleware/modules_system.html#sensors
#		temperature_compensation		# https:#dev.px4.io/v1.9.0/en/middleware/modules_system.html#temperature_compensation
#-		uorb							# https:#dev.px4.io/v1.9.0/en/middleware/modules_communication.html#uorb
#-		vmount							# https:#dev.px4.io/v1.9.0/en/middleware/modules_driver.html#vmount
	SYSTEMCMDS
		#bl_update
#-		dmesg							# bootup messages
		#dumpfile
		#esc_calib
		#gpio
		#hardfault_log
#-		i2cdetect						# check for battery
		#led_control
		#mft							# check wiki for correct command (mft vs mfd)
		#mixer
		#motor_ramp
		#motor_test
		mtd
		#nshterm
		param
		perf
		pwm								# check aux pwms
		reboot
		#reflect						# ?
		#system_time
		#shutdown						# ?
		#tests # tests and test runner
		top
		#topic_listener					# uorb listener
		tune_control
		uorb
		#ver
		work_queue						# check queue status
 )
