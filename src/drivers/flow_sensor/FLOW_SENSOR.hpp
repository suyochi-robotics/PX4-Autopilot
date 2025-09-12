
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/events.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform/micro_hal.h>

#include <px4_arch/io_timer.h>
#include <board_config.h>
#include <px4_platform_common/log.h>
#include <parameters/param.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_flow_sensor.h>

using namespace time_literals;

class FlowSensor : public px4::ScheduledWorkItem, public ModuleBase<FlowSensor>, public ModuleParams
{
public:
	FlowSensor();
	~FlowSensor() override;

	bool init();
	static void stop();

	void Run() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static constexpr hrt_abstime INTERVAL = 1_s;
private:
	// static constexpr uint32_t PARAM_FUNCTION_ID = 2071; // Make sure this matches your config

	int _channel{-1};
	uint32_t _flow_gpio{0};
	px4::atomic<uint32_t> _pulse_count{0};
	px4::atomic<uint32_t> count{0};

	hrt_abstime _last_publish_time{0};

	uORB::PublicationMulti<sensor_flow_sensor_s> _flow_pub{ORB_ID(sensor_flow_sensor)};

	static int gpio_interrupt_callback(int irq, void *context, void *arg);

	// 👉 New static counter for how many times "start" was called
	static int _start_called_count;
	static int _instances_created_count;


};

