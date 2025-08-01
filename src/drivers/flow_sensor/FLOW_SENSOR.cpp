// #include "FLOWSENSOR.hpp"

// #include <px4_arch/io_timer.h>
// #include <lib/parameters/param.h>
// #include <px4_platform_common/log.h>
// #define PREFIX "PWM_AUX"
// // FlowSensor * FlowSensor::instance() = nullptr;

// FlowSensor::FlowSensor() :
// 	ScheduledWorkItem(MODULE_NAME , px4::wq_configurations::hp_default),
// 	ModuleParams(nullptr)
// {
// 	_instance = this;

// 	// _param_calib_factor_handle = param_find("FLOW_CALIB");
// 	// _param_cum_flow_handle = param_find("FLOW_CUM");

//      param_t param_aux_en_handle;
// 	//  param_t param_aux_pin_handle;

// 	_param_aux_en_handle=param_find("FLOW_CAP_ENABLE");
// 	// _param_aux_pin_handle = param_find("FLOW_SENS_PIN");

// 	// param_get(_param_calib_factor_handle, &_param_calib_factor);
// 	// param_get(_param_en_flow_handle, &_param_en_flow_handle);
// 	// param_get(_param_aux_pin_handle, &_param_aux_pin_handle);
// 	init();
// }

// FlowSensor::~FlowSensor()
// {
// 	stop();
// }

// FlowSensor *FlowSensor::instance()
// {
// 	return _instance;
// }

// bool FlowSensor::init()
// {


// 	for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; ++i) {
// 		char param_name[17];
// 		snprintf(param_name, sizeof(param_name), "%s_%s%d", PREFIX, "FUNC", i + 1);
// 		param_t function_handle = param_find(param_name);
// 		int32_t function;

// 		if (function_handle != PARAM_INVALID && param_get(function_handle, &function) == 0) {
// 			if (function == 2070) { // RPM_Input see mixer_module/output_functions.yaml parameter metadata definition
// 				_channel = i;
// 			}
// 		}
// 	}
// 	uin8_t _param_aux_pin ;
//     _param_aux_pin=_channel;

// 	if (!configure_timer()) {
// 		PX4_ERR("Timer configuration failed");
// 		return false;
// 	}

// 	_pulse_count = 0;
// 	_cumulative_volume = _param_cum_flow;
// 	_start_time = _last_save_time = hrt_absolute_time();

// 	_water_flow_pub.advertise();
// 	ScheduleOnInterval(1_s);

// 	PX4_INFO("FlowSensor started on AUX%d", _param_aux_pin);
// 	return true;
// }

// void FlowSensor::start()
// {
// 	if (!init()) {
// 		PX4_ERR("Failed to start FlowSensor");
// 	}
// }

// void FlowSensor::stop()
// {
// 	ScheduleClear();
// 	disable_timer();
// 	_water_flow_pub.unadvertise();
// 	PX4_INFO("FlowSensor stopped");
// }

// void FlowSensor::Run()
// {
// 	collect();
// }

// void FlowSensor::collect()
// {
// 	uint32_t pulses;
// 	// Atomically get and reset
// 	irqstate_t flags = px4_enter_critical_section();
// 	pulses = _pulse_count;
// 	_pulse_count = 0;
// 	px4_leave_critical_section(flags);

// 	const float flow_rate = pulses / _param_calib_factor; // L/min
// 	const float volume = flow_rate / 60.0f;
// 	_cumulative_volume += volume;

// 	const hrt_abstime now = hrt_absolute_time();

// 	// _flow_msg.timestamp = now;
// 	// _flow_msg.flow_rate_lpm = flow_rate;
// 	// _flow_msg.total_volume_liters = _cumulative_volume;

// 	// _flow_pub.publish(_flow_msg);

// 	PX4_INFO("Flow: %.2f L/min, Total: %.2f L", flow_rate, _cumulative_volume);

// 	if (now - _last_save_time > 60_s) {
// 		_param_cum_flow = _cumulative_volume;
// 		param_set(_param_cum_flow_handle, &_param_cum_flow);
// 		_last_save_time = now;
// 	}
// }

// void FlowSensor::gpio_interrupt_callback(uint8_t timer, uint8_t channel, uint16_t /*count*/, uint16_t /*overflow*/)
// {
// 	if (_instance) {
// 		_instance->_pulse_count++;
// 	}
// }

// bool FlowSensor::configure_timer()
// {
// 	if (_param_aux_pin < 1 || _param_aux_pin > 4) {
// 		PX4_ERR("Invalid AUX pin: %d", _param_aux_pin);
// 		return false;
// 	}

// 	// Disable conflicting PWM outputs
// 	int32_t pwm_aux_dis = 0;
// 	param_get(param_find("PWM_AUX_DIS"), &pwm_aux_dis);
// 	if (pwm_aux_dis == 0) {
// 		PX4_ERR("PWM_AUX_DIS must be set");
// 		return false;
// 	}

// 	io_timer_channel_index_t chan_index = io_timer_channel_get_as_pwm_input(_param_aux_pin - 1);
// 	if (chan_index < 0) {
// 		PX4_ERR("Invalid channel index for AUX%d", _param_aux_pin);
// 		return false;
// 	}

// 	if (io_timer_channel_init(chan_index, IOTimerChanMode::Capture) != 0) {
// 		PX4_ERR("Failed to init capture channel");
// 		return false;
// 	}

// 	io_timer_set_enable(chan_index, true, FlowSensor::capture_callback);

// 	return true;
// }

// bool FlowSensor::disable_timer()
// {
// 	if (_param_aux_pin < 1 || _param_aux_pin > 4) {
// 		return false;
// 	}

// 	io_timer_channel_index_t chan_index = io_timer_channel_get_as_pwm_input(_param_aux_pin - 1);
// 	if (chan_index >= 0) {
// 		io_timer_set_enable(chan_index, false, nullptr);
// 	}

// 	return true;
// }

/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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




/******************************************************************************************************** */



#include "FLOW_SENSOR.hpp"


FlowSensor::FlowSensor() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr)
{
	_flow_pub.advertise();
	ScheduleNow();

}

FlowSensor::~FlowSensor()
{
	if (_channel >= 0) {
		io_timer_unallocate_channel(_channel);
		px4_arch_gpiosetevent(_flow_gpio, false, false, false, nullptr, nullptr);
	}

	ScheduleClear();
}

bool FlowSensor::init()
{
	bool success = false;

	/**************Directly setting the parameter vaslue use the below code ********** */
	for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; ++i) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_%s%d", PARAM_PREFIX, "FUNC", i + 1);
		param_t function_handle = param_find(param_name);
		int32_t function;

		if (function_handle != PARAM_INVALID && param_get(function_handle, &function) == 0) {
			// PX4_INFO(" param : %s, value: %ld", param_name, function);
			if (function == 2071) { // FlowSensor input function id
				_channel = i;
				break; // Exit loop once we find the channel
			}
		}
	}

	PX4_INFO("FlowSensor set to channel: %d", _channel);

	if (_channel == -1) {
		PX4_WARN("No FlowSensor channel configured");
		return false;
	}

	int ret = io_timer_allocate_channel(_channel, IOTimerChanMode_Capture);

	if (ret != PX4_OK) {
		return false;
	}

	_flow_gpio = PX4_MAKE_GPIO_EXTI(io_timer_channel_get_as_pwm_input(_channel));
	int ret_val = px4_arch_gpiosetevent(_flow_gpio, false, true, true, &FlowSensor::gpio_interrupt_callback, this);

	if (ret_val == PX4_OK) {
		success = true;
	}

	_last_publish_time = hrt_absolute_time();

	return success;
}

void FlowSensor::Run()
{
	 // Default calibration factor
	param_t flow_cal_handle = param_find("FLOW_CAL_FACTOR");
	// if(flow_cal_handle == PARAM_INVALID) {
	// 	PX4_ERR("FLOW_CAL_FACTOR parameter not found");
	// 	return;
	// }

	float param_cal_factor = 0.0f;
	param_get(flow_cal_handle, &param_cal_factor);
	int ret = param_set(flow_cal_handle, &param_cal_factor);
	if (ret != PX4_OK) {
		PX4_ERR("Failed to set FLOW_CAL_FACTOR parameter");
		return;
	}
	// else if(ret == PX4_OK) {
	// 	PX4_ERR("Flow Calib factor is  %.2f", (double)param_calib_factor);

	// }

	if (should_exit()) {
		exit_and_cleanup();
		return;

	}

	hrt_abstime now = hrt_absolute_time();

	if ((now - _last_publish_time) >= INTERVAL) {
		count = _pulse_count;
		_pulse_count.store(0); // Reset after counting

		if (param_cal_factor <= 0.0f) {
			// PX4_ERR("Calibration factor is zero or -ve, cannot calculate flow rate");
			return;
		}

		float flow_rate_lpm = (static_cast<float>(count.load())) / param_cal_factor;

		sensor_flow_sensor_s flow_msg{};
		flow_msg.timestamp = now;
		flow_msg.flow_rate_lpm = flow_rate_lpm;
		flow_msg.cal_factor = param_cal_factor;
		flow_msg.pulse_count = count.load();
		_flow_pub.publish(flow_msg);

		PX4_INFO("Flow rate: %.2f L/min (%ld pulses)", (double)flow_rate_lpm, count.load());
		_last_publish_time = hrt_absolute_time();
	}

	ScheduleDelayed(INTERVAL);
}

int FlowSensor::gpio_interrupt_callback(int irq, void *context, void *arg)
{
	FlowSensor *instance = static_cast<FlowSensor *>(arg);
	// Increment pulse count atomically
	instance->_pulse_count.fetch_add(1);

	return PX4_OK;
}

int FlowSensor::task_spawn(int argc, char *argv[])
{
	FlowSensor *instance = new FlowSensor();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int FlowSensor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlowSensor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("flow_sensor_capture", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

void FlowSensor::stop()
{
	exit_and_cleanup();
}
