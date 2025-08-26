#ifndef TF02ICAN_HPP
#define TF02ICAN_HPP

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>


#define TF02I_CAN_MODULE_NAME "TF02iCAN"
using namespace time_literals;

class TF02iCAN : public px4::ScheduledWorkItem
{
public:
	TF02iCAN(uint32_t can_id, uint8_t rotation);
	~TF02iCAN() override;

	int init();
	void print_info();
	void stop();

private:
	int collect();
	void Run() override;

	void start();

	PX4Rangefinder _px4_rangefinder;

	// Parameters
	int32_t _can_id{3};          // Default CAN ID
	int32_t _baudrate{115200};        // Default baudrate
	int32_t _interval_ms{100};        // Default interval
	float _min_range{0.1f};      // Default min distance (m)
	float _max_range{40.0f};       // Default max distance (m)
	float _hfov{0.0f};            // Horizontal FOV (optional, unused)
	float _vfov{0.0f};            // Vertical FOV (optional, unused)

        int _can_fd{-1};   // CAN device file descriptor


	// Performance counters
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, TF02I_CAN_MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, TF02I_CAN_MODULE_NAME": com_err")};
	struct can_msg_s {
	    struct {
	        uint32_t ch_id   : 29;   // CAN ID
	        uint32_t ch_rtr  : 1;    // Remote frame
	        uint32_t ch_eff  : 1;    // Extended frame
	        uint32_t ch_len  : 4;    // Data length (0..8)
	    } cm_hdr;
	    uint8_t cm_data[8];          // Payload
	};
};

#endif // TF02ICAN_HPP
