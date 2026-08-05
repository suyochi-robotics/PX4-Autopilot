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



#include "SPRAY_SYSTEM.hpp"

SpraySystem::SpraySystem() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr)
{
	_spray_pub.advertise();
}

int SpraySystem::init()
{
	updateParams();

	_pump_min_pwm = _param_pump_min_pwm.get();
	_pump_max_pwm = math::max(_param_pump_max_pwm.get(), _pump_min_pwm);
	_pump_max_flow_rate = _param_pump_max_flow_rate.get();
	_sprayer_min_pwm = _param_sprayer_min_pwm.get();
	_sprayer_max_pwm = math::max(_param_sprayer_max_pwm.get(), _sprayer_min_pwm);
	_spray_mode = _en_mode.get();

	if (_spray_mode == 0) {
		PX4_INFO("spray system disabled (SPRAY_EN_MODE=0)");
		return PX4_ERROR;
	}

	ScheduleNow();

	return PX4_OK;
}

float SpraySystem::pwmToActuatorValue(float pwm, float min_pwm, float max_pwm)
{
	if (max_pwm <= min_pwm) {
		return -1.f;
	}

	// Mixer functions use [-1, 1]. Derive that value from this actuator's
	// calibrated PWM range instead of assuming a fixed 1000-2000 us range.
	const float normalized_pwm = (pwm - min_pwm) / (max_pwm - min_pwm);
	return math::constrain(2.f * normalized_pwm - 1.f, -1.f, 1.f);
}

void SpraySystem::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// handle parameter updates
	parameter_update_s pupdate;

	if (_parameter_update_sub.updated()) {
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	//read vehicle status
	vehicle_status_s status{};

	if (_vehicle_status_sub.copy(&status)) {

		// check if vehicle is armed
		_armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

		// check if vehicle is in AUTO mission
		_in_auto = (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	}

	const bool spray_active = (_spray_mode == 1)
				  || (_spray_mode == 2 && _armed)
				  || (_spray_mode == 3 && _in_auto)
				  || (_spray_mode == 4 && _en_man.get() != 0);

	//create spray message
	spray_system_status_s msg{};
	msg.timestamp = hrt_absolute_time();

	if (spray_active) {

		const float expected_flow = _pump_expected_flow_rate.get();
		const float expected_speed = _sprayer_expected_speed.get();

		if (expected_flow >= 0.f && _pump_max_flow_rate > 0.f) {
			const float requested_flow = math::min(expected_flow, _pump_max_flow_rate);
			const float flow_fraction = requested_flow / _pump_max_flow_rate;
			const float pump_pwm = _pump_min_pwm + flow_fraction * (_pump_max_pwm - _pump_min_pwm);
			msg.pump_output = pwmToActuatorValue(pump_pwm, _pump_min_pwm, _pump_max_pwm);

		} else {
			msg.pump_output = -1.f;
		}

		if (expected_speed >= 0.f) {
			const float speed_fraction = math::constrain(expected_speed / 100.f, 0.f, 1.f);
			const float sprayer_pwm = _sprayer_min_pwm + speed_fraction * (_sprayer_max_pwm - _sprayer_min_pwm);
			msg.nozzle_output = pwmToActuatorValue(sprayer_pwm, _sprayer_min_pwm, _sprayer_max_pwm);

		} else {
			msg.nozzle_output = -1.f;
		}

	} else {

		msg.pump_output = -1.f;
		msg.nozzle_output = -1.f;
	}

	//publish message
	_spray_pub.publish(msg);

	//run again after 100 ms
	ScheduleOnInterval(100_ms);
}

int SpraySystem::task_spawn(int argc, char *argv[])
{
	SpraySystem *instance = new SpraySystem();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SpraySystem::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "status")) {

		SpraySystem *instance = _object.load();

		if (!instance) {
			PX4_WARN("spray_system not running");
			return PX4_ERROR;
		}

		return instance->print_status();
	}

	return print_usage("unknown command");
}

int SpraySystem::print_status()
{
	PX4_INFO("Spray System Status:");

	PX4_INFO(" Armed        : %s", _armed ? "YES" : "NO");
	PX4_INFO(" Auto Mission : %s", _in_auto ? "YES" : "NO");
	PX4_INFO(" Mode         : %d", (int)_spray_mode);
	PX4_INFO(" Manual enable: %s", _en_man.get() ? "YES" : "NO");
	PX4_INFO(" Pump flow    : %.2f LPM", (double)_pump_expected_flow_rate.get());
	PX4_INFO(" Sprayer speed: %.2f %%", (double)_sprayer_expected_speed.get());
	return PX4_OK;
}

void SpraySystem::stop()
{
	exit_and_cleanup();
}
int SpraySystem::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_USAGE_NAME("spray_system", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return PX4_OK;
}
