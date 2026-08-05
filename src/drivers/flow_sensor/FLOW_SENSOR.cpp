/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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



#include "FLOW_SENSOR.hpp"

FlowSensor::FlowSensor() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr)
{
}

FlowSensor::~FlowSensor()
{
	ScheduleClear();

	if (_channel >= 0) {
		px4_arch_gpiosetevent(_flow_gpio, false, false, false, nullptr, nullptr);
		io_timer_unallocate_channel(_channel);
		_channel = -1;
	}
}

bool FlowSensor::init()
{
	for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; ++i) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_%s%d", PARAM_PREFIX, "FUNC", i + 1);
		param_t function_handle = param_find(param_name);
		int32_t function;

		if (function_handle != PARAM_INVALID && param_get(function_handle, &function) == 0) {
			// PX4_INFO(" param : %s, value: %ld", param_name, function);
			if (function == FLOW_SENSOR_FUNCTION_ID) {
				_channel = i;
				break; // Exit loop once we find the channel
			}
		}
	}

	if (_channel == -1) {
		PX4_WARN("No FlowSensor channel configured");
		return false;
	}

	int ret = io_timer_allocate_channel(_channel, IOTimerChanMode_Capture);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to allocate flow sensor channel %d (%d)", _channel, ret);
		return false;
	}

	_flow_gpio = PX4_MAKE_GPIO_EXTI(io_timer_channel_get_as_pwm_input(_channel));
	int ret_val = px4_arch_gpiosetevent(_flow_gpio, false, true, true, &FlowSensor::gpio_interrupt_callback, this);

	if (ret_val != PX4_OK) {
		PX4_ERR("Failed to configure flow sensor interrupt (%d)", ret_val);
		io_timer_unallocate_channel(_channel);
		_channel = -1;
		return false;
	}

	const param_t flow_cal_handle = param_find("FLOW_CAL_FACTOR");

	if (flow_cal_handle == PARAM_INVALID || param_get(flow_cal_handle, &_cal_factor) != PX4_OK
	    || !PX4_ISFINITE(_cal_factor) || _cal_factor <= 0.0f) {
		PX4_ERR("FLOW_CAL_FACTOR must be finite and greater than zero");
		px4_arch_gpiosetevent(_flow_gpio, false, false, false, nullptr, nullptr);
		io_timer_unallocate_channel(_channel);
		_channel = -1;
		return false;
	}

	_last_publish_time = hrt_absolute_time();
	_rate_samples[0] = {_last_publish_time, _pulse_count.load()};
	_rate_sample_count = 1;
	_next_rate_sample = 1;
	ScheduleDelayed(INTERVAL);
	return true;
}

void FlowSensor::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const uint32_t pulse_count = _pulse_count.load();

	_rate_samples[_next_rate_sample] = {now, pulse_count};
	_next_rate_sample = (_next_rate_sample + 1) % RATE_SAMPLE_COUNT;

	if (_rate_sample_count < RATE_SAMPLE_COUNT) {
		++_rate_sample_count;
	}

	// Use the sample nearest to, but not newer than, one second ago. Before
	// the window has filled, use the oldest available sample.
	const hrt_abstime window_start = (now > RATE_WINDOW) ? now - RATE_WINDOW : 0;
	const RateSample *reference = nullptr;

	for (uint8_t i = 0; i < _rate_sample_count; ++i) {
		const RateSample &sample = _rate_samples[i];

		if (reference == nullptr || sample.timestamp < reference->timestamp) {
			reference = &sample;
		}

		if (sample.timestamp <= window_start
		    && (reference->timestamp > window_start || sample.timestamp > reference->timestamp)) {
			reference = &sample;
		}
	}

	const float elapsed_s = static_cast<float>(now - reference->timestamp) * 1e-6f;
	const uint32_t pulses_in_window = pulse_count - reference->pulse_count;
	const float flow_rate_lpm = (elapsed_s > 0.0f)
				    ? (static_cast<float>(pulses_in_window) / _cal_factor) / elapsed_s : 0.0f;

	// A pulse represents 1 / (FLOW_CAL_FACTOR * 60) litres. Use the monotonic
	// count so the accumulated volume is independent of publish timing.
	_total_volume_liters = static_cast<float>(pulse_count) / (_cal_factor * 60.0f);

	sensor_flow_sensor_s flow_msg{};
	flow_msg.timestamp = now;
	flow_msg.flow_rate_lpm = flow_rate_lpm;
	flow_msg.pulse_count = pulse_count;
	flow_msg.total_volume_liters = _total_volume_liters;
	flow_msg.cal_factor = _cal_factor;
	_flow_pub.publish(flow_msg);

	_last_publish_time = now;

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
