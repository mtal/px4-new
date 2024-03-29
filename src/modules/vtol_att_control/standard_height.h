/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

#ifndef STANDARD_HEIGHT_H
#define STANDARD_HEIGHT_H
#include "vtol_type.h"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>

class StandardHeight : public VtolType
{

public:

    StandardHeight(VtolAttitudeControl *_att_controller);
	~StandardHeight();

	virtual void update_vtol_state();
	virtual void update_transition_state();
	virtual void update_fw_state();
	virtual void update_mc_state();
	virtual void fill_actuator_outputs();
	virtual void waiting_on_tecs();

private:

	struct {
		float pusher_ramp_dt;
		float back_trans_ramp;
		float down_pitch_max;
		float forward_thrust_scale;
		float pitch_setpoint_offset;
		float reverse_output;
		float reverse_delay;
	} _params_standard;

	struct {
        float front_trans_height;
        float front_trans_height_safety;
	}  _params_standard_height;

	struct {
		param_t pusher_ramp_dt;
		param_t back_trans_ramp;
		param_t down_pitch_max;
		param_t forward_thrust_scale;
		param_t pitch_setpoint_offset;
		param_t reverse_output;
		param_t reverse_delay;
	} _params_handles_standard;

    struct {
        param_t front_trans_height;
        param_t front_trans_height_safety;
    } _params_handles_standard_height;

    enum vtol_mode {
		MC_MODE = 0,
		TRANSITION_TO_FW,
		TRANSITION_TO_MC,
		FW_MODE
	};

	struct {
		vtol_mode flight_mode;			// indicates in which mode the vehicle is in
        float transition_start_z;     // at what z position we start a front transition
		hrt_abstime transition_start;	// at what time did we start a transition (front- or backtransition)
	} _vtol_schedule;

	float _pusher_throttle;
	float _reverse_output;
	float _airspeed_trans_blend_margin;

	virtual void parameters_update();
};
#endif
