/****************************************************************************
 *
 *   Copyright (c) 2025.
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
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "TF02iCAN.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

/**
 * Local functions in support of the shell command.
 */
namespace tf02i_can
{

TF02iCAN *g_dev{nullptr};

int start(uint32_t can_id, uint8_t rotation);
int status();
int stop();
int usage();

int
start(uint32_t can_id, uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver
	g_dev = new TF02iCAN(can_id, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("driver init failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
	} else {
		g_dev->print_info();
	}
	usage();

	return PX4_OK;
}

int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		g_dev->stop();
		delete g_dev;
		g_dev = nullptr;
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
CAN bus driver for the Benewake TF02-i LiDAR (CAN interface).

Most boards are configured to enable/start the driver using the SENS_EN_TF02I_CAN parameter.

### Examples
Start driver with default CAN ID (0x03):
$ tf02i_can start -i 3

Stop driver:
$ tf02i_can stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tf02i_can", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', 3, 0, 2047, "CAN ID (standard frame, 11-bit)", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	return PX4_OK;
}

} // namespace tf02i_can


extern "C" __EXPORT int tf02i_can_main(int argc, char *argv[])
{
	int ch = 0;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	uint32_t can_id = 0x00000003; // default CAN ID per TF02-i manual
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'i':
			can_id = (uint32_t)strtoul(myoptarg, nullptr, 0);
			break;

		default:
			PX4_WARN("Unknown option!");
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		PX4_ERR("unrecognized command");
		return tf02i_can::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return tf02i_can::start(can_id, rotation);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return tf02i_can::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return tf02i_can::status();
	}

	return tf02i_can::usage();
}
