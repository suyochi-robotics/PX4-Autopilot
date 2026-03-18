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




/******************************************************************************************************** */#pragma once

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
