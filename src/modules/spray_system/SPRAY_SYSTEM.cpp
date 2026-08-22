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

#include <cmath>

SpraySystem::SpraySystem() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr)
{
	_spray_pub.advertise();
}

int SpraySystem::init()
{
	updateParams();

	_spray_system_configured = _param_spray_enable.get() != 0;
	_flow_sensor_configured = _param_flow_cap_enable.get() != 0;

	const float configured_min_flow = _param_spry_flow_min.get();
	_flow_min_lpm = PX4_ISFINITE(configured_min_flow) ? math::max(configured_min_flow, 0.f) : 0.f;

	const float configured_timeout = _param_spry_flow_tout.get();
	const float timeout_s = PX4_ISFINITE(configured_timeout) ? math::max(configured_timeout, 0.1f) : 5.f;
	_flow_timeout_us = static_cast<hrt_abstime>(timeout_s * 1e6f);
	_pump_min_pwm = _param_pump_min_pwm.get();
	_pump_max_pwm = math::max(_param_pump_max_pwm.get(), _pump_min_pwm);
	_sprayer_min_pwm = _param_sprayer_min_pwm.get();
	_sprayer_max_pwm = math::max(_param_sprayer_max_pwm.get(), _sprayer_min_pwm);

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

void SpraySystem::updateSprayEnable()
{
	vehicle_command_s command{};

	while (_vehicle_command_sub.update(&command)) {
		if (command.command == vehicle_command_s::VEHICLE_CMD_DO_SPRAYER) {
			const bool valid_enable = PX4_ISFINITE(command.param1)
						  && ((command.param1 >= -0.001f && command.param1 <= 0.001f)
						      || (command.param1 >= 0.999f && command.param1 <= 1.001f));
			const bool valid_pump_speed = std::isnan(command.param2)
						      || (PX4_ISFINITE(command.param2) && command.param2 >= 0.f && command.param2 <= 100.f);
			const bool valid_nozzle_speed = std::isnan(command.param3)
							|| (PX4_ISFINITE(command.param3) && command.param3 >= 0.f && command.param3 <= 100.f);

			if (!valid_enable || !valid_pump_speed || !valid_nozzle_speed) {
				PX4_WARN("Ignoring invalid DO_SPRAYER command");
				continue;
			}

			_spray_enabled = command.param1 > 0.5f;
			_command_pump_speed = command.param2;
			_command_nozzle_speed = command.param3;
		}
	}
}

bool SpraySystem::flowFailsafeMonitoringRequired() const
{
	const vehicle_status_s &vehicle_status = _vehicle_status_sub.get();
	const vehicle_land_detected_s &land_detected = _vehicle_land_detected_sub.get();
	const bool manual_pump_enabled = (_spray_enable_manual.get() != 0)
					 && PX4_ISFINITE(_pump_expected_speed.get()) && (_pump_expected_speed.get() > 0.f);

	return _spray_system_configured
	       && _flow_sensor_configured
	       && (vehicle_status.timestamp != 0)
	       && (land_detected.timestamp != 0)
	       && (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
	       && !land_detected.landed
	       && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
	       && (manual_pump_enabled || _spray_enabled);
}

bool SpraySystem::flowSensorDataIsFresh(hrt_abstime now) const
{
	return (_flow_sensor.timestamp != 0)
	       && (now >= _flow_sensor.timestamp)
	       && ((now - _flow_sensor.timestamp) <= FLOW_SENSOR_DATA_TIMEOUT)
	       && PX4_ISFINITE(_flow_sensor.flow_rate_lpm);
}

void SpraySystem::requestFlowFailsafeRTL(hrt_abstime now)
{
	const vehicle_status_s &vehicle_status = _vehicle_status_sub.get();
	vehicle_command_s rtl_command{};
	rtl_command.timestamp = now;
	rtl_command.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
	rtl_command.source_system = vehicle_status.system_id;
	rtl_command.source_component = vehicle_status.component_id;
	rtl_command.target_system = vehicle_status.system_id;
	rtl_command.target_component = vehicle_status.component_id;
	rtl_command.confirmation = false;
	rtl_command.from_external = false;
	_vehicle_command_pub.publish(rtl_command);

	PX4_ERR("Spray flow lost for %.1f s, returning to launch", static_cast<double>(_flow_timeout_us) * 1e-6);
}

void SpraySystem::updateFlowFailsafe(hrt_abstime now)
{
	_vehicle_status_sub.update();
	_vehicle_land_detected_sub.update();

	sensor_flow_sensor_s flow_sensor{};

	if (_flow_sensor_sub.update(&flow_sensor)) {
		_flow_sensor = flow_sensor;
	}

	if (_flow_fault_latched) {
		// The latch inhibits the pump/nozzle through RTL. Clear it only after a
		// disarm, when a subsequent flight starts with a clean monitor state.
		if (_vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			_flow_fault_latched = false;
		}

		_zero_flow_start = 0;
		return;
	}

	if (!flowFailsafeMonitoringRequired() || !flowSensorDataIsFresh(now)) {
		_zero_flow_start = 0;
		return;
	}

	if (_flow_sensor.flow_rate_lpm > _flow_min_lpm) {
		_zero_flow_start = 0;
		return;
	}

	if (_zero_flow_start == 0) {
		_zero_flow_start = now;
		return;
	}

	if ((now - _zero_flow_start) >= _flow_timeout_us) {
		_flow_fault_latched = true;
		_zero_flow_start = 0;
		requestFlowFailsafeRTL(now);
	}
}

void SpraySystem::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	parameter_update_s pupdate;

	if (_parameter_update_sub.updated()) {
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	updateSprayEnable();

	const hrt_abstime now = hrt_absolute_time();
	updateFlowFailsafe(now);

	//create spray message
	spray_system_status_s msg{};
	msg.timestamp = now;
	const bool spray_active = _spray_enabled || (_spray_enable_manual.get() != 0);

	if (spray_active && !_flow_fault_latched) {

		const float expected_pump_speed = PX4_ISFINITE(_command_pump_speed) ? _command_pump_speed : _pump_expected_speed.get();
		const float expected_speed = PX4_ISFINITE(_command_nozzle_speed) ? _command_nozzle_speed :
					     _sprayer_expected_speed.get();

		if (expected_pump_speed >= 0.f) {
			const float speed_fraction = math::constrain(expected_pump_speed / 100.f, 0.f, 1.f);
			const float pump_pwm = _pump_min_pwm + speed_fraction * (_pump_max_pwm - _pump_min_pwm);
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

	PX4_INFO(" Enabled      : %s", _spray_enabled ? "YES" : "NO");
	PX4_INFO(" Flow fault   : %s", _flow_fault_latched ? "YES" : "NO");
	PX4_INFO(" Manual enable: %s", _spray_enable_manual.get() ? "YES" : "NO");
	PX4_INFO(" Pump speed   : %.2f %%", (double)_pump_expected_speed.get());
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
