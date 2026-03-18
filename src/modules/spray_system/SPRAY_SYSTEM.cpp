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

ModuleBase::Descriptor SpraySystem::desc{task_spawn, custom_command, print_usage};

SpraySystem::SpraySystem() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr)
{
	_spray_pub.advertise();
	ScheduleNow();
}

bool SpraySystem::init()
{
	return PX4_OK;
}

void SpraySystem::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
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
		_in_auto = (status.nav_state >= vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	}

	//read spray mode parameter
	_spray_mode = _en_mode.get();

	bool spray_active = false;

	//decide when spray should run
	if (_spray_mode == 1) {

                // MODE 1 → always spray
		spray_active = true;

	} else if (_spray_mode == 2 && _armed) {

                // MODE 2 → spray only when armed
		spray_active = true;

	} else if (_spray_mode == 3 && _in_auto) {

                // MODE 3 → spray only during AUTO mission
		spray_active = true;
	}

	//create spray message
	spray_system_status_s msg{};
	msg.timestamp = hrt_absolute_time();

	if (spray_active) {

		// Pump logic (LPM → normalized 0-1)
		float pump_norm = math::constrain(_pump_lpm.get(), 0.f, 8.f) / 8.f;

		// Nozzle logic (RPM → normalized 0-1)
		float nozzle_norm = math::constrain(_cent_rpm.get(), 1000.f, 20000.f);
		nozzle_norm = (nozzle_norm - 1000.f) / (20000.f - 1000.f);

		msg.pump_output = pump_norm;
		msg.nozzle_output = nozzle_norm;

	} else {

		msg.pump_output = 0.f;
		msg.nozzle_output = 0.f;
	}

	//publish message
	_spray_pub.publish(msg);

	//run again after 100 ms
	ScheduleOnInterval(100_ms);
}

int SpraySystem::task_spawn(int argc, char *argv[])
{
	if (desc.object.load()) {
		PX4_WARN("Task already running");
		return PX4_ERROR;
        }

	SpraySystem *instance = new SpraySystem();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (instance->init() == PX4_OK) {
		return PX4_OK;
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int SpraySystem::custom_command(int argc, char *argv[])
{
	if (argc == 0) {
		return print_usage("missing command");
	}

	if (!strcmp(argv[0], "status")) {

		SpraySystem *instance =
			static_cast<SpraySystem *>(desc.object.load());

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
	PX4_INFO(" Mode         : %d", (int)_en_mode.get());
	PX4_INFO(" Pump LPM     : %.2f", (double)_pump_lpm.get());
	PX4_INFO(" Cent RPM     : %.2f", (double)_cent_rpm.get());
	return PX4_OK;
}

void SpraySystem::stop()
{
	exit_and_cleanup(desc);
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
