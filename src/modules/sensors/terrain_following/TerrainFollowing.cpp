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
#include <lib/drivers/microbrain_h201_1/Microbrain_H201_1.hpp>
#include <px4_log.h>
#include <lib/parameters/param.h>  // Required for param_get()

namespace terrain_following {

Microbrain_H201_1 *terrain_dev{nullptr};

int
start(const char *module_name, const char *port, uint8_t rotation, float min_range, float max_range, float h_fov, float v_fov, uint32_t type)
{
	if (terrain_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	if (type == 1) {
		// Instantiate the driver.
		terrain_dev = new Microbrain_H201_1(module_name, port, rotation, min_range, max_range, h_fov, v_fov);

		if (terrain_dev == nullptr) {
			PX4_ERR("driver start failed");
			return PX4_ERROR;
		}

		if (OK != terrain_dev->init()) {
			PX4_ERR("driver start failed");
			delete terrain_dev;
			terrain_dev = nullptr;
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
	if (terrain_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", terrain_dev);
	terrain_dev->print_info();

	return 0;
}

int stop()
{
	if (terrain_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete terrain_dev;
		terrain_dev = nullptr;
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

Serial bus driver for the MIcroBrain Intelligent Microbrain_H201_1
 RADAR.

Most boards are configured to enable/start the driver on a specified UART using the TR_FL_PORT
_CFG parameter.

Setup/usage information: https://docs.px4.io/main/en/sensor/    terrain_following
.html

### Examples

Attempt to start driver on a specified serial device.
$     terrain_following
 start -d /dev/ttyS1 -r 25 -b 115200
Stop driver
$     terrain_following
 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("terrain_following ", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	// PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status");
	return PX4_OK;
}
}

extern "C" __EXPORT int terrain_following_main(int argc, char *argv[])
{
	//int ch = 0;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	char device_path[20] {};
	int myoptind = 1;
	// const char *myoptarg = nullptr;

	// while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
	// 	switch (ch) {
	// 	case 'd':
	// 		device_path = myoptarg;
	// 		break;

	// 	default:
	// 		PX4_WARN("Unknown option!");
	// 		return PX4_ERROR;
	// 	}
	// }

	// if (myoptind >= argc) {
	// 	PX4_ERR("unrecognized command");
	// 	return terrain_following::usage();
	// }



	// === Load parameters ===
	int32_t tr_fl_port;
	int32_t tr_fl_enable;
	int32_t tr_fl_type;
	float tr_fl_h_fov;
	float tr_fl_v_fov;
	float tr_fl_min_rng;
	float tr_fl_max_rng;
	float tr_fl_rng;


	param_get(param_find("TR_FL_ENABLE"), &tr_fl_enable);
	param_get(param_find("TR_FL_PORT"), &tr_fl_port);
	param_get(param_find("TR_FL_TYPE"), &tr_fl_type);
	param_get(param_find("TR_FL_H_FOV"), &tr_fl_h_fov);
	param_get(param_find("TR_FL_V_FOV"), &tr_fl_v_fov);
	param_get(param_find("TR_FL_MIN_RNG"), &tr_fl_min_rng);
	param_get(param_find("TR_FL_MAX_RNG"), &tr_fl_max_rng);
	param_get(param_find("TR_FL_RNG"), &tr_fl_rng);

	if (tr_fl_enable <= 0) {
		PX4_INFO("Kindly set TR_FL_ENABLE to ONE");
		return PX4_ERROR;
	}

	if (tr_fl_port > 0 && tr_fl_port < 10) {
		char port_num = '0' + tr_fl_port;
		// store port name
		char fixed_path[] = "/dev/ttyS";
		strncpy(device_path, fixed_path, sizeof(fixed_path) - 1);
		int base_port_length = strlen(fixed_path);
		device_path[base_port_length] = port_num;
		device_path[base_port_length + 1] = '\0';

	} else {
		PX4_INFO("Kindly set valid TR_FL_PORT");
		return PX4_ERROR;
	}

	PX4_INFO("MODULE_NAME: %s", MODULE_NAME);
	/*PX4_INFO("TR_FL_ENABLE: %ld", tr_fl_enable);
	PX4_INFO("TR_FL_PORT: %s", device_path);
	PX4_INFO("TR_FL_TYPE: %ld", tr_fl_type);
	PX4_INFO("TR_FL_H_FOV: %.2f", (double)tr_fl_h_fov);
	PX4_INFO("TR_FL_V_FOV: %.2f", (double)tr_fl_v_fov);
	PX4_INFO("TR_FL_MIN_RNG: %.2f m", (double)tr_fl_min_rng);
	PX4_INFO("TR_FL_MAX_RNG: %.2f m", (double)tr_fl_max_rng);
	PX4_INFO("TR_FL_RNG: %.2f m", (double)tr_fl_rng);*/

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return terrain_following::start(MODULE_NAME, device_path, rotation, tr_fl_min_rng, tr_fl_max_rng, tr_fl_h_fov, tr_fl_v_fov, tr_fl_type);

		} else {
			PX4_WARN("Please specify device path!");
			return terrain_following::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return terrain_following::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return terrain_following::status();
	}

	return terrain_following::usage();
}
