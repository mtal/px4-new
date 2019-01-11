/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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

/**
 * @file servo.cpp
 *
 * @author Biryukov Aleksey <abiryukov1996@gmail.com>
 */

#include "servo.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

UavcanServoController::UavcanServoController(uavcan::INode &node) :
	node(node),
	uavcanPublisher(node)
{
	this->uavcanPublisher.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);

	if (_perfcnt_invalid_input == nullptr)
		errx(1, "uavcan: couldn't allocate _perfcnt_invalid_input");
    else if (_perfcnt_scaling_error == nullptr)
		errx(1, "uavcan: couldn't allocate _perfcnt_scaling_error");
}

UavcanServoController::~UavcanServoController()
{
	perf_free(_perfcnt_invalid_input);
	perf_free(_perfcnt_scaling_error);
}

int UavcanServoController::Init()
{
	return 0;
}

void UavcanServoController::UpdateOutputs(float *outputs, unsigned num_outputs)
{
	if ((outputs == nullptr)
        || (num_outputs > uavcan::equipment::actuator::ArrayCommand::FieldTypes::commands::MaxSize))
    {
		perf_count(_perfcnt_invalid_input);
		return;
	}

	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = this->node.getMonotonicTime();

	if ((timestamp - this->previousPublication).toUSec() < (1000000 / MAX_RATE_HZ))
		return;

	this->previousPublication = timestamp;

	/*
	 * Fill the command message
	 * If unarmed, we publish an empty message anyway
	 */
	uavcan::equipment::actuator::ArrayCommand msg;

	static const float cmd_max  = 2000.0F;
	static const float cmd_min  = 0.0F;

    for (unsigned i = 0; i < num_outputs; i++)
    {
        float scaled = (outputs[i] + 1.0F) * 0.5F * cmd_max;

        // trim negative values back to minimum
        if (scaled < cmd_min)
        {
            scaled = cmd_min;
            perf_count(_perfcnt_scaling_error);
        }

        if (scaled > cmd_max)
        {
            scaled = cmd_max;
            perf_count(_perfcnt_scaling_error);
        }

        msg.commands[i].actuator_id     = i;
        msg.commands[i].command_value   = static_cast<int>(scaled);
    }

	/*
	 * Publish the command message to the bus
	 * Note that for a servo it takes one CAN frame
	 */
	(void)this->uavcanPublisher.broadcast(msg);
}