
px4_add_board(
	PLATFORM nuttx
	VENDOR simtoo
	MODEL moment
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT px4fmu_common
	CONSTRAINED_FLASH
	SERIAL_PORTS
		GPS1:/dev/ttyS2
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS0
	DRIVERS
		imu/invensense/mpu6500
#		barometer/bmp280
		magnetometer/hmc5883
		gps
		#optical_flow/pmw3901
		#distance_sensor/vl53l1x
		#pwm_out
	MODULES
		airspeed_selector
		#attitude_estimator_q
		#battery_status
		#camera_feedback
		commander
		dataman
		ekf2
		#esc_battery
		#events
		flight_mode_manager
		fw_att_control
		fw_pos_control_l1
		#gyro_fft
		land_detector
		#landing_target_estimator
		load_mon
		#local_position_estimator
		#logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		#micrortps_bridge
		navigator
		rc_update
		#rover_pos_control
		sensors
		#sih
		#temperature_compensation
		#vmount
		#vtol_att_control
	SYSTEMCMDS
		#bl_update
		#dmesg
		#dumpfile
		#esc_calib
		#hardfault_log
		#i2cdetect
		#led_control
		mft
		mixer
		#motor_ramp
		#motor_test
		mtd
		#nshterm
		param
		#perf
		#pwm
		reboot
		#reflect
		#sd_bench
		#shutdown
		#tests # tests and test runner
		top
		#topic_listener
		tune_control
		#uorb
		#usb_connected
		#ver
		#work_queue
	)

