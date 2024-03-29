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

#include "standard_height.h"
#include "vtol_att_control_main.h"

#include <float.h>

using matrix::wrap_pi;

StandardHeight::StandardHeight(VtolAttitudeControl *attc) :
	VtolType(attc),
	_pusher_throttle(0.0f),
	_reverse_output(0.0f),
	_airspeed_trans_blend_margin(0.0f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;
	_vtol_schedule.transition_start_z = 0;
	_pusher_active = false;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
	_mc_throttle_weight = 1.0f;

	_params_handles_standard.pusher_ramp_dt = param_find("VT_PSHER_RMP_DT");
	_params_handles_standard.back_trans_ramp = param_find("VT_B_TRANS_RAMP");
	_params_handles_standard.down_pitch_max = param_find("VT_DWN_PITCH_MAX");
	_params_handles_standard.forward_thrust_scale = param_find("VT_FWD_THRUST_SC");
	_params_handles_standard.pitch_setpoint_offset = param_find("FW_PSP_OFF");
	_params_handles_standard.reverse_output = param_find("VT_B_REV_OUT");
	_params_handles_standard.reverse_delay = param_find("VT_B_REV_DEL");

	_params_handles_standard_height.front_trans_height = param_find("VT_F_TR_HGHT");
	_params_handles_standard_height.front_trans_height_safety = param_find("VT_F_TR_HGHT_SF");
}

StandardHeight::~StandardHeight() = default;

void
StandardHeight::parameters_update()
{
	float v;

	/* duration of a forwards transition to fw mode */
	param_get(_params_handles_standard.pusher_ramp_dt, &v);
	_params_standard.pusher_ramp_dt = math::constrain(v, 0.0f, 20.0f);

	/* MC ramp up during back transition to mc mode */
	param_get(_params_handles_standard.back_trans_ramp, &v);
	_params_standard.back_trans_ramp = math::constrain(v, 0.0f, _params->back_trans_duration);

	_airspeed_trans_blend_margin = _params->transition_airspeed - _params->airspeed_blend;

	/* maximum down pitch allowed */
	param_get(_params_handles_standard.down_pitch_max, &v);
	_params_standard.down_pitch_max = math::radians(v);

	/* scale for fixed wing thrust used for forward acceleration in multirotor mode */
	param_get(_params_handles_standard.forward_thrust_scale, &_params_standard.forward_thrust_scale);

	/* pitch setpoint offset */
	param_get(_params_handles_standard.pitch_setpoint_offset, &v);
	_params_standard.pitch_setpoint_offset = math::radians(v);

	/* reverse output */
	param_get(_params_handles_standard.reverse_output, &v);
	_params_standard.reverse_output = math::constrain(v, 0.0f, 1.0f);

	/* reverse output */
	param_get(_params_handles_standard.reverse_delay, &v);
	_params_standard.reverse_delay = math::constrain(v, 0.0f, 10.0f);

	/* front transition height */
	param_get(_params_handles_standard_height.front_trans_height, &v);
	_params_standard_height.front_trans_height = math::constrain(v, 10.0f, 100.0f);

	/* front transition height safety */
    param_get(_params_handles_standard_height.front_trans_height_safety, &v);
    _params_standard_height.front_trans_height_safety = math::constrain(v, 0.0f, 2.0f);

}

void StandardHeight::update_vtol_state()
{
	/* After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
	 */

	float mc_weight = _mc_roll_weight;
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (!_attc->is_fixed_wing_requested()) {

		// the transition to fw mode switch is off
		if (_vtol_schedule.flight_mode == MC_MODE) {
			// in mc mode
			_vtol_schedule.flight_mode = MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;
			_reverse_output = 0.0f;

		} else if (_vtol_schedule.flight_mode == FW_MODE) {
			// transition to mc mode
			if (_vtol_vehicle_status->vtol_transition_failsafe == true) {
				// Failsafe event, engage mc motors immediately
				_vtol_schedule.flight_mode = MC_MODE;
				_pusher_throttle = 0.0f;
				_reverse_output = 0.0f;


			} else {
				// Regular backtransition
				_vtol_schedule.flight_mode = TRANSITION_TO_MC;
				_vtol_schedule.transition_start = hrt_absolute_time();
                _vtol_schedule.transition_start_z = _local_pos->z;
				_reverse_output = _params_standard.reverse_output;

			}

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_schedule.flight_mode = MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;
			_reverse_output = 0.0f;


		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// transition to MC mode if transition time has passed or forward velocity drops below MPC cruise speed

			const matrix::Dcmf R_to_body(matrix::Quatf(_v_att->q).inversed());
			const matrix::Vector3f vel = R_to_body * matrix::Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);

			float x_vel = vel(0);

			if (time_since_trans_start > _params->back_trans_duration ||
			    (_local_pos->v_xy_valid && x_vel <= _params->mpc_xy_cruise)) {
				_vtol_schedule.flight_mode = MC_MODE;
			}

		}

	} else {
		// the transition to fw mode switch is on
		if (_vtol_schedule.flight_mode == MC_MODE || _vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// start transition to fw mode
			/* NOTE: The failsafe transition to fixed-wing was removed because it can result in an
			 * unsafe flying state. */
			_vtol_schedule.flight_mode = TRANSITION_TO_FW;
			_vtol_schedule.transition_start = hrt_absolute_time();
            _vtol_schedule.transition_start_z = _local_pos->z;

		} else if (_vtol_schedule.flight_mode == FW_MODE) {
			// in fw mode
			_vtol_schedule.flight_mode = FW_MODE;
			mc_weight = 0.0f;

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// continue the transition to fw mode while monitoring airspeed and height for a final switch to fw mode

			//TODO: check acceptance radius ?

            // z is negative position!!!!
            float height_since_trans_start = -(_local_pos->z - _vtol_schedule.transition_start_z);

            if (((_params->airspeed_disabled ||
			      _airspeed->indicated_airspeed_m_s >= _params->transition_airspeed) &&
			     time_since_trans_start > _params->front_trans_time_min &&
                    height_since_trans_start - _params_standard_height.front_trans_height > FLT_EPSILON) ||
			    can_transition_on_ground()) {

				_vtol_schedule.flight_mode = FW_MODE;

				// don't set pusher throttle here as it's being ramped up elsewhere
				_trans_finished_ts = hrt_absolute_time();
			}

		}
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;

	// map specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		break;

	case FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		break;

	case TRANSITION_TO_FW:
		_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case TRANSITION_TO_MC:
		_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void StandardHeight::update_transition_state()
{
	float mc_weight = 1.0f;
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	VtolType::update_transition_state();

	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
		if (_params_standard.pusher_ramp_dt <= 0.0f) {
			// just set the final target throttle value
			_pusher_throttle = _params->front_trans_throttle;

		} else if (_pusher_throttle <= _params->front_trans_throttle) {
			// ramp up throttle to the target throttle value
			_pusher_throttle = _params->front_trans_throttle * time_since_trans_start / _params_standard.pusher_ramp_dt;
		}

		// z is negative position!!!!
        float height_since_trans_start = -(_local_pos->z - _vtol_schedule.transition_start_z);

		// double safety for me
		float trans_height = _params_standard_height.front_trans_height - _params_standard_height.front_trans_height_safety;

        // height based blending ignoring whether or not airspeed sensor is set
        mc_weight = 1.0f - height_since_trans_start / trans_height;

        // triple safety required to Zamaleev's acceptance
        mc_weight = math::constrain(2.0f * mc_weight, 0.0f, 1.0f);



		// ramp up FW_PSP_OFF
		_v_att_sp->pitch_body = _params_standard.pitch_setpoint_offset * (1.0f - mc_weight);
		matrix::Quatf q_sp(matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		_v_att_sp->q_d_valid = true;

		// check front transition timeout
		if (_params->front_trans_timeout > FLT_EPSILON) {
			if (time_since_trans_start > _params->front_trans_timeout) {
				// transition timeout occured, abort transition
				_attc->abort_front_transition("Transition timeout");
			}
		}

	} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {

		// maintain FW_PSP_OFF
		_v_att_sp->pitch_body = _params_standard.pitch_setpoint_offset;
		matrix::Quatf q_sp(matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		_v_att_sp->q_d_valid = true;

		_pusher_throttle = 0.0f;

		if (time_since_trans_start >= _params_standard.reverse_delay) {
			// Handle throttle reversal for active breaking
			float thrscale = (time_since_trans_start - _params_standard.reverse_delay) / (_params_standard.pusher_ramp_dt);
			thrscale = math::constrain(thrscale, 0.0f, 1.0f);
			_pusher_throttle = thrscale * _params->back_trans_throttle;
		}

		// continually increase mc attitude control as we transition back to mc mode
		if (_params_standard.back_trans_ramp > FLT_EPSILON) {
			mc_weight = time_since_trans_start / _params_standard.back_trans_ramp;

		}

		// in back transition we need to start the MC motors again
		if (_motor_state != ENABLED) {
			_motor_state = set_motor_state(_motor_state, ENABLED);
		}
	}

	mc_weight = math::constrain(mc_weight, 0.0f, 1.0f);

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;
}

void StandardHeight::update_mc_state()
{
	VtolType::update_mc_state();

	// if the thrust scale param is zero or the drone is on manual mode,
	// then the pusher-for-pitch strategy is disabled and we can return
	if (_params_standard.forward_thrust_scale < FLT_EPSILON ||
	    !_v_control_mode->flag_control_position_enabled) {
		return;
	}

	// Do not engage pusher assist during a failsafe event
	// There could be a problem with the fixed wing drive
	if (_attc->get_vtol_vehicle_status()->vtol_transition_failsafe) {
		return;
	}

	// disable pusher assist during landing
	if (_attc->get_pos_sp_triplet()->current.valid
	    && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		return;
	}

	matrix::Dcmf R(matrix::Quatf(_v_att->q));
	matrix::Dcmf R_sp(matrix::Quatf(_v_att_sp->q_d));
	matrix::Eulerf euler(R);
	matrix::Eulerf euler_sp(R_sp);
	_pusher_throttle = 0.0f;

	// direction of desired body z axis represented in earth frame
	matrix::Vector3f body_z_sp(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	// rotate desired body z axis into new frame which is rotated in z by the current
	// heading of the vehicle. we refer to this as the heading frame.
	matrix::Dcmf R_yaw = matrix::Eulerf(0.0f, 0.0f, -euler(2));
	body_z_sp = R_yaw * body_z_sp;
	body_z_sp.normalize();

	// calculate the desired pitch seen in the heading frame
	// this value corresponds to the amount the vehicle would try to pitch forward
	float pitch_forward = atan2f(body_z_sp(0), body_z_sp(2));

	// only allow pitching forward up to threshold, the rest of the desired
	// forward acceleration will be compensated by the pusher
	if (pitch_forward < -_params_standard.down_pitch_max) {
		// desired roll angle in heading frame stays the same
		float roll_new = -asinf(body_z_sp(1));

		_pusher_throttle = (sinf(-pitch_forward) - sinf(_params_standard.down_pitch_max))
				   * _params_standard.forward_thrust_scale;

		// return the vehicle to level position
		float pitch_new = 0.0f;

		// create corrected desired body z axis in heading frame
		matrix::Dcmf R_tmp = matrix::Eulerf(roll_new, pitch_new, 0.0f);
		matrix::Vector3f tilt_new(R_tmp(0, 2), R_tmp(1, 2), R_tmp(2, 2));

		// rotate the vector into a new frame which is rotated in z by the desired heading
		// with respect to the earh frame.
		const float yaw_error = wrap_pi(euler_sp(2) - euler(2));
		matrix::Dcmf R_yaw_correction = matrix::Eulerf(0.0f, 0.0f, -yaw_error);
		tilt_new = R_yaw_correction * tilt_new;

		// now extract roll and pitch setpoints
		_v_att_sp->pitch_body = atan2f(tilt_new(0), tilt_new(2));
		_v_att_sp->roll_body = -asinf(tilt_new(1));
		R_sp = matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, euler_sp(2));
		matrix::Quatf q_sp(R_sp);
		q_sp.copyTo(_v_att_sp->q_d);
	}

	_pusher_throttle = _pusher_throttle < 0.0f ? 0.0f : _pusher_throttle;

}

void StandardHeight::update_fw_state()
{
	VtolType::update_fw_state();
}

/**
 * Prepare message to acutators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void StandardHeight::fill_actuator_outputs()
{
	// multirotor controls
	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

	// roll
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight;
	// pitch
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	// yaw
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_yaw_weight;
	// throttle
	_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;


	// fixed wing controls
	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	if (_vtol_schedule.flight_mode != MC_MODE) {
		// roll
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];

		// pitch
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];
		// yaw
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];

		_actuators_out_1->control[actuator_controls_s::INDEX_AIRBRAKES] = _reverse_output;

	} else {

		if (_params->elevons_mc_lock) {
			// zero outputs when inactive
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.0f;
			_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_actuators_out_1->control[actuator_controls_s::INDEX_AIRBRAKES] = 0.0f;

		} else {
			// roll
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
				-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];

			// pitch
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];

			_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_actuators_out_1->control[actuator_controls_s::INDEX_AIRBRAKES] = 0.0f;
		}
	}

	// set the fixed wing throttle control
	if (_vtol_schedule.flight_mode == FW_MODE) {

		// take the throttle value commanded by the fw controller
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

	} else {
		// otherwise we may be ramping up the throttle during the transition to fw mode
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] = _pusher_throttle;
	}


}

void
StandardHeight::waiting_on_tecs()
{
	// keep thrust from transition
	_v_att_sp->thrust = _pusher_throttle;
};
