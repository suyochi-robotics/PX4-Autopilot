#pragma once

#include "FunctionProviderBase.hpp"

#include <uORB/topics/spray_system_status.h>

/**
 * @brief Function: Spray System (Pump + Centrifugal Nozzles)
 */
class FunctionSpraySystem : public FunctionProviderBase
{
public:
	FunctionSpraySystem() = default;

	static FunctionProviderBase *allocate(const Context &context)
	{
		return new FunctionSpraySystem();
	}

	void update() override
	{
		spray_system_status_s spray;

		if (_spray_sub.update(&spray)) {
			_pump_output = spray.pump_output;
			_nozzle_output = spray.nozzle_output;
		}
	}

	float value(OutputFunction func) override
	{
		switch (func) {

		case OutputFunction::Spray_Pump:
			return _pump_output;

		case OutputFunction::Centrifugal_Nozzles:
			return _nozzle_output;

		default:
			return NAN;
		}
	}

private:

	uORB::Subscription _spray_sub{ORB_ID(spray_system_status)};

	float _pump_output{0.f};
	float _nozzle_output{0.f};
};
