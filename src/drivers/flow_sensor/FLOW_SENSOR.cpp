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

ModuleBase::Descriptor FlowSensor::desc{task_spawn, custom_command, print_usage};

FlowSensor::FlowSensor() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr)
{
	_flow_pub.advertise();

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
			PX4_INFO(" param : %s, value: %ld", param_name, function);

			if (function == 2071) {
				_channel = i;
				break;
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

	_flow_cal_factor = _param_flow_cal_factor.get();
	_flow_enabled = _param_flow_cap_enable.get();

	PX4_INFO("Flow Cal Factor: %.2f", (double)_flow_cal_factor);
	PX4_INFO("Flow Enabled: %s", _flow_enabled ? "true" : "false");

	if (!_flow_enabled) {
		PX4_WARN("Flow sensor disabled");
		return false;
	}

	_flow_gpio = PX4_MAKE_GPIO_EXTI(io_timer_channel_get_as_pwm_input(_channel));
	int ret_val = px4_arch_gpiosetevent(_flow_gpio, false, true, true, &FlowSensor::gpio_interrupt_callback, this);

	if (ret_val == PX4_OK) {
		success = true;
	}

	_last_publish_time = hrt_absolute_time();


	if (success) {
		ScheduleNow();
	}

	return success;
}

void FlowSensor::Run()
{

	if (should_exit()) {
		exit_and_cleanup(desc);
		return;

	}

	hrt_abstime now = hrt_absolute_time();

	if ((now - _last_publish_time) >= INTERVAL) {
		uint32_t pulse_count = _pulse_count.load();
		_pulse_count.store(0);

		if (_flow_cal_factor <= 0.0f) {
			PX4_ERR("Invalid calibration factor");
			return;
		}

		float flow_rate_lpm = (static_cast<float>(pulse_count)) / _flow_cal_factor;

		sensor_flow_sensor_s flow_msg{};
		flow_msg.timestamp = now;
		flow_msg.flow_rate_lpm = flow_rate_lpm;
		flow_msg.cal_factor = _flow_cal_factor;
		flow_msg.pulse_count = pulse_count;
		_flow_pub.publish(flow_msg);

		PX4_DEBUG("Flow rate: %.2f L/min (%lu pulses)", (double)flow_rate_lpm, pulse_count);
		_last_publish_time = now;
	}

	ScheduleDelayed(INTERVAL);
}

int FlowSensor::gpio_interrupt_callback(int irq, void *context, void *arg)
{
	FlowSensor *instance = static_cast<FlowSensor *>(arg);
	PX4_DEBUG("IRQ Triggered");
	instance->_pulse_count.fetch_add(1);

	return PX4_OK;
}

int FlowSensor::task_spawn(int argc, char *argv[])
{
	FlowSensor *instance = new FlowSensor();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

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
	exit_and_cleanup(desc);
}
