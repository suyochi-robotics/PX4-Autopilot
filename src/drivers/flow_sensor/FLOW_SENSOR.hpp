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
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/events.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform/micro_hal.h>

#include <px4_arch/io_timer.h>
#include <board_config.h>

#include <parameters/param.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_flow_sensor.h>

using namespace time_literals;

class FlowSensor : public px4::ScheduledWorkItem, public ModuleBase<FlowSensor>, public ModuleParams
{
public:
	FlowSensor();
	~FlowSensor() override;

	bool init();
	static void stop();

	void Run() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static constexpr hrt_abstime INTERVAL = 1_s;
private:
	// static constexpr uint32_t PARAM_FUNCTION_ID = 2071; // Make sure this matches your config

	int _channel{-1};
	uint32_t _flow_gpio{0};
	px4::atomic<uint32_t> _pulse_count{0};
	px4::atomic<uint32_t> count{0};

	hrt_abstime _last_publish_time{0};

	uORB::PublicationMulti<sensor_flow_sensor_s> _flow_pub{ORB_ID(sensor_flow_sensor)};

	static int gpio_interrupt_callback(int irq, void *context, void *arg);
};
