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

/**
 * @file FlightTaskPrecisionLanding.cpp
 *
 */

#include "FlightTaskPrecisionLanding.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskPrecisionLanding::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_precland_state.nav_state = prec_land_status_s::PREC_LAND_NAV_STATE_START;
	_precland_state.state = prec_land_status_s::PREC_LAND_STATE_ONGOING;

	_search_count = 0;

	_position_setpoint = _position;

	_initial_yaw = _yaw;
	_initial_yawspeed = 0;
	_initial_position = _position;

	// _is_activated = true;
	_land_detected = false;
	return ret;
}

void FlightTaskPrecisionLanding::do_state_transition(uint8_t new_state)
{
	_initial_yaw = _yaw;
	_precland_state.nav_state = new_state;
	_state_start_time = hrt_absolute_time();
}

bool FlightTaskPrecisionLanding::inside_acceptance_radius()
{
	// TODO: Reuse what FlightTask has...
	return matrix::Vector3f(_position_setpoint - _position).norm() <= _param_pld_hacc_rad.get();
}

bool FlightTaskPrecisionLanding::precision_target_available()
{
	const bool ever_received = _landing_target_pose.timestamp != 0;
	const bool timed_out =  (hrt_absolute_time() - _landing_target_pose.timestamp) > (_param_pld_btout.get() * SEC2USEC);
	return ever_received && !timed_out;
}

void FlightTaskPrecisionLanding::generate_pos_xy_setpoints()
{
	switch (_precland_state.nav_state) {

	case prec_land_status_s::PREC_LAND_NAV_STATE_START:
		_position_setpoint(0) = _initial_position(0);
		_position_setpoint(1) = _initial_position(1);
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_HORIZONTAL:
	case prec_land_status_s::PREC_LAND_NAV_STATE_DESCEND:
	case prec_land_status_s::PREC_LAND_NAV_STATE_FINAL:
		_position_setpoint(0) = _landing_target_pose.x_abs;
		_position_setpoint(1) = _landing_target_pose.y_abs;
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_SEARCH:
	case prec_land_status_s::PREC_LAND_NAV_STATE_FALLBACK:
		_position_setpoint(0) = _position(0);
		_position_setpoint(1) = _position(1);
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_DONE:
		break;

	}
}

void FlightTaskPrecisionLanding::generate_pos_z_setpoints()
{
	const float search_rel_altitude = _param_pld_srch_alt.get();

	switch (_precland_state.nav_state) {

	case prec_land_status_s::PREC_LAND_NAV_STATE_START:
	case prec_land_status_s::PREC_LAND_NAV_STATE_HORIZONTAL:
		_position_setpoint(2) = _position(2);
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_DESCEND:
	case prec_land_status_s::PREC_LAND_NAV_STATE_FINAL:
	case prec_land_status_s::PREC_LAND_NAV_STATE_FALLBACK:
		// Dont use position setpoint when descending, velocity is being set.
		_position_setpoint(2) = NAN;
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_SEARCH:
		if (PX4_ISFINITE(_landing_target_pose.z_abs)) {
			_position_setpoint(2) = math::min(_sub_home_position.get().z, _landing_target_pose.z_abs) - search_rel_altitude;
		} else {
			_position_setpoint(2) = _sub_home_position.get().z - search_rel_altitude;
		}
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_DONE:
		break;
	}

}

void FlightTaskPrecisionLanding::generate_vel_setpoints()
{

	switch (_precland_state.nav_state) {

	case prec_land_status_s::PREC_LAND_NAV_STATE_START:
	case prec_land_status_s::PREC_LAND_NAV_STATE_SEARCH:
		_velocity_setpoint.setNaN();
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_FALLBACK:
		_velocity_setpoint(0) = 0;
		_velocity_setpoint(1) = 0;
		_velocity_setpoint(2) = _param_mpc_land_speed.get();
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_HORIZONTAL:
		_velocity_setpoint(0) = 0;
		_velocity_setpoint(1) = 0;

#if defined(CONFIG_VTEST_MOVING)
		_velocity_setpoint(0) = _velocity(0) + _landing_target_pose.vx_rel;
		_velocity_setpoint(1) = _velocity(1) + _landing_target_pose.vy_rel;
#endif // CONFIG_VTEST_MOVING

		_velocity_setpoint(2) = NAN;
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_DESCEND:
	case prec_land_status_s::PREC_LAND_NAV_STATE_FINAL:
		_velocity_setpoint(0) = 0;
		_velocity_setpoint(1) = 0;

#if defined(CONFIG_VTEST_MOVING)
		_velocity_setpoint(0) = _velocity(0) + _landing_target_pose.vx_rel;
		_velocity_setpoint(1) = _velocity(1) + _landing_target_pose.vy_rel;
#endif // CONFIG_VTEST_MOVING

		_velocity_setpoint(2) = _param_mpc_land_speed.get();
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_DONE:
		break;
	}

}

void FlightTaskPrecisionLanding::generate_acc_setpoints()
{
	_acceleration_setpoint.setNaN();
}

void FlightTaskPrecisionLanding::generate_yaw_setpoint()
{
	_yaw_setpoint = NAN;
	switch (_precland_state.nav_state) {

	case prec_land_status_s::PREC_LAND_NAV_STATE_START:
		_yaw_setpoint = NAN;
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_HORIZONTAL:
	case prec_land_status_s::PREC_LAND_NAV_STATE_DESCEND:
	case prec_land_status_s::PREC_LAND_NAV_STATE_FINAL:
#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR)
		if (_param_pld_yaw_en.get() == 1) {
			_yaw_setpoint = _vte_est_orientation.theta;
		}
		else {
			_yaw_setpoint = NAN;
		}

		break;
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	case prec_land_status_s::PREC_LAND_NAV_STATE_SEARCH:
	case prec_land_status_s::PREC_LAND_NAV_STATE_FALLBACK:
		_yaw_setpoint = NAN;
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_DONE:
		break;
	}
}

void FlightTaskPrecisionLanding::generate_yaw_rate_setpoint()
{
	_yawspeed_setpoint = NAN;
}

void FlightTaskPrecisionLanding::check_state_transitions()
{

	switch (_precland_state.nav_state) {

	case prec_land_status_s::PREC_LAND_NAV_STATE_START: {
		if (precision_target_available()){
			PX4_INFO("Transitioning to Horizontal");
			do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_HORIZONTAL);
		} else {
			PX4_INFO("Transitioning to Search");
			do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_SEARCH);
		}
		break;
	}

	case prec_land_status_s::PREC_LAND_NAV_STATE_HORIZONTAL: {
		if (inside_acceptance_radius()){
			// Only transition to PREC_LAND_NAV_STATE_DESCEND if we still see target
			if (precision_target_available()){
				PX4_INFO("Transitioning to Descend");
				do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_DESCEND);
			}
			else {
				// we've reached the position where we last saw the target, move to search.
				PX4_INFO("Transitioning to Search");
				do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_SEARCH);
			}
		}
		// TODO maybe add a break for getting stuck in this state !inside_acceptance_radius && measurement has timed out
		break;
	}

	case prec_land_status_s::PREC_LAND_NAV_STATE_DESCEND:{
		if (precision_target_available()) {
			// TODO A little horrible
			if (-_position(2) < _param_pld_fappr_alt.get()) {
				PX4_INFO("Transitioning to Final");
				do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_FINAL);
			}
		} else {
			// We have lost the target, move back to search.
			PX4_INFO("Transitioning to Search");
			do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_SEARCH);
		}
		break;
	}

	case prec_land_status_s::PREC_LAND_NAV_STATE_SEARCH: {
		const float max_search_duration = _param_pld_srch_tout.get();

		if ((hrt_absolute_time() - _state_start_time) > max_search_duration * SEC2USEC) {
			if (++_search_count > _param_pld_max_srch.get()) {
				PX4_INFO("Transitioning to Fallback");
				do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_FALLBACK);
			}
		}

		if (precision_target_available()) {
			PX4_INFO("Transitioning to Horizontal");
			do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_HORIZONTAL);
		}
		break;
	}

	case prec_land_status_s::PREC_LAND_NAV_STATE_FINAL:
	case prec_land_status_s::PREC_LAND_NAV_STATE_FALLBACK:
		if (_land_detected){
			PX4_INFO("Transitioning to Done");
			do_state_transition(prec_land_status_s::PREC_LAND_NAV_STATE_DONE);
		}
		break;

	case prec_land_status_s::PREC_LAND_NAV_STATE_DONE:
		_precland_state.state = prec_land_status_s::PREC_LAND_STATE_STOPPED;
		break;
	}
}


bool FlightTaskPrecisionLanding::update()
{
	// Get setpoints from FlightTask and later override if necessary
	bool ret = FlightTask::update();

	// Fetch uorb
	if (_landing_target_pose_sub.updated()) {
		_landing_target_pose_sub.copy(&_landing_target_pose);
	}

	if (_vehicle_land_detected_sub.update(&vehicle_land_detected) && vehicle_land_detected.landed) {
		_land_detected = true;
	}

	generate_pos_xy_setpoints();
	generate_pos_z_setpoints();
	generate_vel_setpoints();
	generate_acc_setpoints();
	generate_yaw_setpoint();
	generate_yaw_rate_setpoint();
	check_state_transitions();

	prec_land_status_s prec_land_status{};
	prec_land_status.timestamp = hrt_absolute_time();
	prec_land_status.state = _precland_state.state;
	prec_land_status.nav_state = _precland_state.nav_state;
	_prec_land_status_pub.publish(prec_land_status);

	return ret;
}
