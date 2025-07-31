#ifndef MICROBRAIN_H201_1_HPP
#define MICROBRAIN_H201_1_HPP

#pragma once

#include <cstdint>
#include <cstddef>
#include <unistd.h>
#include <cstring>
#include <termios.h>
#include <math.h>
#include <fcntl.h>
#include <px4_log.h>
#include <poll.h>


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>



using namespace time_literals;

class Microbrain_H201_1 : public px4::ScheduledWorkItem
{
public:
	Microbrain_H201_1(const char* module_name, const char* port, uint8_t rotation, float_t min_range, float_t max_range, float_t h_fov, float_t v_fov);
	virtual ~Microbrain_H201_1();

	int init();
	void print_info();

private:
	int collect();
	void Run() override;
	int open_port();

	void start();
	void stop();

	PX4Rangefinder	_px4_rangefinder;

	char _port[20] {};
	uint8_t _rotation;
	float_t _min_range;
	float_t _max_range;
	float_t _h_fov;
	float_t _v_fov;

	static constexpr int kCONVERSIONINTERVAL{9_ms};

	int _fd{-1};

	hrt_abstime _last_read{0};
	struct pollfd _fds;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

};

#endif // MICROBRAIN_H201_1_HPP
