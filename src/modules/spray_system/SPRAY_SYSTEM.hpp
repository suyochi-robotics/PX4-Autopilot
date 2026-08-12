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


#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/spray_system_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/parameter_update.h>

#include <mathlib/mathlib.h>

using namespace time_literals;

class SpraySystem : public px4::ScheduledWorkItem, public ModuleBase<SpraySystem>, public ModuleParams
{
public:
	SpraySystem();
	~SpraySystem() override = default;

	int init();
	static void stop();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status();

private:
	uORB::Publication<spray_system_status_s> _spray_pub{ORB_ID(spray_system_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	bool _spray_enabled{false};
	float _command_pump_speed{NAN};
	float _command_nozzle_speed{NAN};

	// PWM calibration parameters are intentionally cached during init(). Their
	// metadata marks them reboot-required, so changes take effect after restart.
	float _pump_min_pwm{1050.f};
	float _pump_max_pwm{1950.f};
	float _sprayer_min_pwm{1050.f};
	float _sprayer_max_pwm{1950.f};

	static float pwmToActuatorValue(float pwm, float min_pwm, float max_pwm);
	void updateSprayEnable();

	void Run() override;
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PUMP_MIN_PWM>) _param_pump_min_pwm,
		(ParamFloat<px4::params::PUMP_MAX_PWM>) _param_pump_max_pwm,
		(ParamFloat<px4::params::PUMP_EXP_SPD>) _pump_expected_speed,
		(ParamFloat<px4::params::SPRAYER_MIN_PWM>) _param_sprayer_min_pwm,
		(ParamFloat<px4::params::SPRAYER_MAX_PWM>) _param_sprayer_max_pwm,
		(ParamFloat<px4::params::SPRYAER_EXP_SPD>) _sprayer_expected_speed,
		(ParamInt<px4::params::SPRAY_EN_MAN>) _spray_enable_manual
	)
};
