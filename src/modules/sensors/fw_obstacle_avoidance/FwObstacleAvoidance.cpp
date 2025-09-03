/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/getopt.h>
#include <lib/drivers/microbrain_r121_1/Microbrain_R121_1.hpp>
#include <px4_log.h>
#include <lib/parameters/param.h>

namespace fw_obs_avoidance {

Microbrain_R121_1 *fw_obs_dev{nullptr};

int
start(const char *module_name, const char *port, uint8_t rotation, float min_range, float max_range, float f_obs_h_fov, float f_obs_v_fov, uint32_t type)
{
	if (fw_obs_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	if (type == 1) {
		// Instantiate the driver.
		fw_obs_dev = new Microbrain_R121_1(module_name, port, rotation, min_range, max_range, f_obs_h_fov, f_obs_v_fov);

		if (fw_obs_dev == nullptr) {
			PX4_ERR("driver start failed");
			return PX4_ERROR;
		}

		if (OK != fw_obs_dev->init()) {
			PX4_ERR("driver start failed");
			delete fw_obs_dev;
			fw_obs_dev = nullptr;
			return PX4_ERROR;
		}
	} else {
		PX4_ERR("Unsupported Sensor");
	}

	return PX4_OK;
}

int
status()
{
	if (fw_obs_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", fw_obs_dev);
	fw_obs_dev->print_info();

	return 0;
}

int stop()
{
	if (fw_obs_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete fw_obs_dev;
		fw_obs_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the MIcroBrain Intelligent Microbrain_R121_1
 RADAR.

Most boards are configured to enable/start the driver on a specified UART using the B_OBS_PORT
_CFG parameter.

Setup/usage information: https://docs.px4.io/main/en/sensor/    bw_obstacle_avoidance
.html

### Examples

Attempt to start driver on a specified serial device.
$     fw_obstacle_avoidance
 start -d /dev/ttyS1 -r 25 -b 115200
Stop driver
$     fw_obstacle_avoidance
 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_obstacle_avoidance ", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status");
	return PX4_OK;
}
}

extern "C" __EXPORT int fw_obstacle_avoidance_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_FORWARD_FACING;
	char device_path[20]{};
	int myoptind = 1;
	// === Fetch all parameters to ensure they're registered ===
	int32_t f_obs_port;
	int32_t f_obs_enable;
	int32_t f_obs_type;
	float f_obs_h_fov;
	float f_obs_v_fov;
	float f_obs_min_rng;
	float f_obs_max_rng;
	float f_obs_sl_rng;
	float f_obs_st_rng;

	param_get(param_find("F_OBS_ENABLE"), &f_obs_enable);
	param_get(param_find("F_OBS_PORT"), &f_obs_port);
	param_get(param_find("F_OBS_TYPE"), &f_obs_type);
	param_get(param_find("F_OBS_H_FOV"), &f_obs_h_fov);
	param_get(param_find("F_OBS_V_FOV"), &f_obs_v_fov);
	param_get(param_find("F_OBS_MIN_RNG"), &f_obs_min_rng);
	param_get(param_find("F_OBS_MAX_RNG"), &f_obs_max_rng);
	param_get(param_find("F_OBS_SL_RNG"), &f_obs_sl_rng);
	param_get(param_find("F_OBS_ST_RNG"), &f_obs_st_rng);

	if (f_obs_enable <= 0) {
		PX4_INFO("Kindly set F_OBS_ENABLE to ONE");
		return PX4_ERROR;
	}

	if (f_obs_port > 0 && f_obs_port < 10) {
		char port_num = '0' + f_obs_port;
		// store port name
		char fixed_path[] = "/dev/ttyS";
		strncpy(device_path, fixed_path, sizeof(fixed_path) - 1);
		int base_port_length = strlen(fixed_path);
		device_path[base_port_length] = port_num;
		device_path[base_port_length + 1] = '\0';

	} else {
		PX4_INFO("Kindly set valid F_OBS_PORT");
		return PX4_ERROR;
	}

	/*
	PX4_INFO("MODULE_NAME: %s", MODULE_NAME);
	PX4_INFO("F_OBS_ENABLE: %ld", f_obs_enable);
	PX4_INFO("F_OBS_PORT: %s", device_path);
	PX4_INFO("F_OBS_TYPE: %ld", f_obs_type);
	PX4_INFO("F_OBS_H_FOV: %.2f", (double)f_obs_h_fov);
	PX4_INFO("F_OBS_V_FOV: %.2f", (double)f_obs_v_fov);
	PX4_INFO("F_OBS_MIN_RNG: %.2f", (double)f_obs_min_rng);
	PX4_INFO("F_OBS_MAX_RNG: %.2f", (double)f_obs_max_rng);
	PX4_INFO("F_OBS_SL_RNG: %.2f", (double)f_obs_sl_rng);
	PX4_INFO("F_OBS_ST_RNG: %.2f", (double)f_obs_st_rng);
	*/

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return fw_obs_avoidance::start(MODULE_NAME, device_path, rotation, f_obs_min_rng, f_obs_max_rng, f_obs_h_fov, f_obs_v_fov, f_obs_type);

		} else {
			PX4_WARN("Please specify device path!");
			return fw_obs_avoidance::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return fw_obs_avoidance::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return fw_obs_avoidance::status();
	}

	return fw_obs_avoidance::usage();
}

