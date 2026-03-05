#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/spray_system_status.h>
#include <uORB/topics/vehicle_status.h>

#include <mathlib/mathlib.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class SpraySystem : public px4::ScheduledWorkItem, public ModuleBase<SpraySystem>, public ModuleParams
{
 public:
	SpraySystem();
	~SpraySystem() override = default;

	bool init();
	static void stop();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status();

 private:
	uORB::Publication<spray_system_status_s> _spray_pub{ORB_ID(spray_system_status)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	bool _armed{false};
	bool _in_auto{false};
	int32_t _spray_mode{0};

	void Run() override;

 DEFINE_PARAMETERS(
	(ParamFloat<px4::params::SPRAY_PUMP_LPM>) _pump_lpm,
	(ParamFloat<px4::params::SPRAY_CENT_RPM>) _cent_rpm,
	(ParamInt<px4::params::SPRAY_EN_MODE>) _en_mode
  )
};



