message(STATUS ">>>>>>>Building flow sensor driver<<<<<<<<")

set(PARAM_PREFIX PWM_AUX)

px4_add_module(
	MODULE driver__flow_sensor
	MAIN flow_sensor
	COMPILE_FLAGS
		-DPARAM_PREFIX="${PARAM_PREFIX}"
	SRCS
		FLOW_SENSOR.cpp
		flow_sensor_main.cpp
	DEPENDS
		uORB
		arch_io_pins
		px4_work_queue
	)

