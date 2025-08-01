// #pragma once

// #include <px4_platform_common/module.h>
// #include <px4_platform_common/module_params.h>

// #include <drivers/device/device.h>
// #include <lib/perf/perf_counter.h>
// #include <uORB/PublicationMulti.hpp>
// #include <uORB/topics/sensor_optical_flow.h>
// // #include<platforms/common/uORB/uORBTopics.h>

// #include <px4_platform_common/getopt.h>

// #include <uORB/Subscription.hpp>
// #include <board_config.h>



// #include <lib/perf/perf_counter.h>


// #include <px4_platform_common/defines.h>

// #include <lib/parameters/param.h>
// #include <drivers/drv_hrt.h>
// #include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
// #include <uORB/Publication.hpp>
// #include <px4_arch/micro_hal.h>


// #include <px4_arch/io_timer.h>

// using namespace time_literals;

// class FlowSensor : public ModuleBase<FlowSensor>, public px4::ScheduledWorkItem, public ModuleParams
// {
// public:
// 	// FlowSensor(uint8_t timer_channel);
// 	FlowSensor();
// 	~FlowSensor() override;
//     FlowSensor* _instance;
// 	static FlowSensor *instance();
// 	bool init();
// 	void start();
// 	void stop();

// 	void Run() override;

// 	void captureCallback();

// 	static int gpio_interrupt_callback(int irq, void *context, void *arg);


// 	uint8_t channel() const { return _timer_channel; }

// private:
// 	int collect();

// 	uint32_t get_and_reset_pulse_count();

// 	void configure_input_capture();
// 	void disable_input_capture();

// 	uint8_t _timer_channel;
// 	volatile uint32_t _pulse_count{0};

// 	hrt_abstime _last_time{0};
// 	hrt_abstime _last_save_time{0};
// 	float _cumulative_volume{0.0f};

// 	// uORB::Publication<Flowsensor> _flow_pub{ORB_ID(SensorFlowsensor)};
// 	// flow_sensor _flow_msg{};

// 	// DEFINE_PARAMETERS(
// 	// 	(ParamFloat<px4::params::FLOWSENSOR_CAL>) _param_calibration,
// 	// 	(ParamFloat<px4::params::FLOWSENSOR_TOTAL>) _param_total_volume

// };
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/events.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform/micro_hal.h>

#include <px4_arch/io_timer.h>
#include <board_config.h>

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
};

