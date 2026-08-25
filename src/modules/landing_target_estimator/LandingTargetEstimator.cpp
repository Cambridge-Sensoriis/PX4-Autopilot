/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file LandingTargetEstimator.cpp
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 * @author Mohammed Kabir <kabir@uasys.io>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "LandingTargetEstimator.h"

#define SEC2USEC 1000000.0f

namespace landing_target_estimator
{

LandingTargetEstimator::LandingTargetEstimator()
{
	_paramHandle.acc_unc = param_find("LTEST_ACC_UNC");
	_paramHandle.meas_grad = param_find("LTEST_MEAS_GRAD");
	_paramHandle.meas_base = param_find("LTEST_MEAS_BASE");
	_paramHandle.pos_unc_init = param_find("LTEST_POS_UNC_IN");
	_paramHandle.vel_unc_init = param_find("LTEST_VEL_UNC_IN");
	_paramHandle.mode = param_find("LTEST_MODE");
	_paramHandle.scale_x = param_find("LTEST_SCALE_X");
	_paramHandle.scale_y = param_find("LTEST_SCALE_Y");
	_paramHandle.sensor_yaw = param_find("LTEST_SENS_ROT");
	_paramHandle.offset_x = param_find("LTEST_SENS_POS_X");
	_paramHandle.offset_y = param_find("LTEST_SENS_POS_Y");
	_paramHandle.offset_z = param_find("LTEST_SENS_POS_Z");
	_paramHandle.lag = param_find("LTEST_LAG");
	_check_params(true);
}

void LandingTargetEstimator::update()
{
	_check_params(false);

	_update_topics();

	/* predict */
	if (_estimator_initialized) {
		if (hrt_absolute_time() - _last_update > landing_target_estimator_TIMEOUT_US) {
			PX4_INFO("Lost sight of Marker");
			_estimator_initialized = false;

		} else {
			float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

			// predict target position with the help of accel data
			matrix::Vector3f a{_vehicle_acceleration.xyz};

			if (_vehicleAttitude_valid && _vehicle_acceleration_valid) {
				matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
				_R_att = matrix::Dcm<float>(q_att);
				a = _R_att * a;

			} else {
				a.zero();
			}

			_kalman_filter_x.predict(dt, -a(0), _params.acc_unc);
			_kalman_filter_y.predict(dt, -a(1), _params.acc_unc);

			_last_predict = hrt_absolute_time();
		}
	}

	if (_new_target_measurement) {
		// mark this sensor measurement as consumed
		_new_target_measurement = false;

		if (!_estimator_initialized) {
			float vx_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vx : 0.f;
			float vy_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vy : 0.f;
			PX4_INFO("Init %.2f %.2f", (double)vx_init, (double)vy_init);
			_kalman_filter_x.init(_target_position_report.rel_pos_x, vx_init, _params.pos_unc_init, _params.vel_unc_init);
			_kalman_filter_y.init(_target_position_report.rel_pos_y, vy_init, _params.pos_unc_init, _params.vel_unc_init);

			_estimator_initialized = true;
			_target_pose.timestamp = _target_position_report.timestamp;
			_last_update = hrt_absolute_time();
			_last_predict = _last_update;

		} else {
			// update
			// The lateral error of a bearing measurement grows with the range to the target, the floor
			// keeps the modelled noise from collapsing to zero as we approach it.
			const float meas_stddev = _dist_z * _params.meas_grad + _params.meas_base;
			const float measurement_uncertainty = meas_stddev * meas_stddev;
			bool update_x = _kalman_filter_x.update(_target_position_report.rel_pos_x, measurement_uncertainty);
			bool update_y = _kalman_filter_y.update(_target_position_report.rel_pos_y, measurement_uncertainty);

			if (!update_x || !update_y) {
				_consecutive_rejections++;

				if (_consecutive_rejections > MAX_CONSECUTIVE_REJECTIONS) {
					// The gate has locked out: the state has drifted far enough from the measurements
					// that nothing gets back in, and with no update it only drifts further until the
					// target times out. Fuse past the gate to recover. Deliberately fusing rather than
					// re-initialising, because init() would zero the velocity state and the moving
					// target feedforward is built on it. Only the axis that was rejected is forced,
					// so an axis that already fused is not fused twice.
					if (!update_x) {
						_kalman_filter_x.update(_target_position_report.rel_pos_x, measurement_uncertainty, true);
					}

					if (!update_y) {
						_kalman_filter_y.update(_target_position_report.rel_pos_y, measurement_uncertainty, true);
					}

					_consecutive_rejections = 0;
					_faulty = false;

				} else if (!_faulty) {
					_faulty = true;
					PX4_INFO("Landing target measurement rejected:%s%s", update_x ? "" : " x", update_y ? "" : " y");
				}

			} else {
				_consecutive_rejections = 0;
				_faulty = false;
			}

			if (!_faulty) {
				// Only a fused measurement advances the timestamp and the loss watchdog. A rejected
				// one leaves both where they were, so a run of rejections still times the target out.
				_target_pose.timestamp = _target_position_report.timestamp;
				_last_update = hrt_absolute_time();
				_last_predict = _last_update;
			}

			float innov_x, innov_cov_x, innov_y, innov_cov_y;
			_kalman_filter_x.getInnovations(innov_x, innov_cov_x);
			_kalman_filter_y.getInnovations(innov_y, innov_cov_y);

			_target_innovations.timestamp = _target_position_report.timestamp;
			_target_innovations.meas_lag = _meas_lag_us / SEC2USEC;
			_target_innovations.innov_x = innov_x;
			_target_innovations.innov_cov_x = innov_cov_x;
			_target_innovations.innov_y = innov_y;
			_target_innovations.innov_cov_y = innov_cov_y;

			_targetInnovationsPub.publish(_target_innovations);
		}
	}

	// Publish every cycle the filter is running, not only when a measurement lands. The predict step
	// above already advances the state at the module rate, and withholding it until the next
	// measurement left consumers holding a position up to a full sensor interval stale, which shows
	// up as the vehicle trailing a moving target.
	if (_estimator_initialized) {
		_publish_target_pose();
	}
}

void LandingTargetEstimator::_publish_target_pose()
{
	float x, xvel, y, yvel, covx, covx_v, covy, covy_v;
	_kalman_filter_x.getState(x, xvel);
	_kalman_filter_x.getCovariance(covx, covx_v);

	_kalman_filter_y.getState(y, yvel);
	_kalman_filter_y.getCovariance(covy, covy_v);

	_target_pose.is_static = (_params.mode == TargetMode::Stationary);

	_target_pose.rel_pos_valid = true;
	_target_pose.rel_vel_valid = true;
	_target_pose.x_rel = x;
	_target_pose.y_rel = y;
	_target_pose.z_rel = _target_position_report.rel_pos_z;
	_target_pose.vx_rel = xvel;
	_target_pose.vy_rel = yvel;

	_target_pose.cov_x_rel = covx;
	_target_pose.cov_y_rel = covy;

	_target_pose.cov_vx_rel = covx_v;
	_target_pose.cov_vy_rel = covy_v;

	if (_vehicleLocalPosition_valid && _vehicleLocalPosition.xy_valid) {
		_target_pose.x_abs = x + _vehicleLocalPosition.x;
		_target_pose.y_abs = y + _vehicleLocalPosition.y;
		_target_pose.z_abs = _target_position_report.rel_pos_z + _vehicleLocalPosition.z;
		_target_pose.abs_pos_valid = true;

	} else {
		_target_pose.abs_pos_valid = false;
	}

	_targetPosePub.publish(_target_pose);
}

void LandingTargetEstimator::_check_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		_update_params();
	}
}

void LandingTargetEstimator::_update_topics()
{
	_vehicleLocalPosition_valid = _vehicleLocalPositionSub.update(&_vehicleLocalPosition);
	_vehicleAttitude_valid = _attitudeSub.update(&_vehicleAttitude);
	_vehicle_acceleration_valid = _vehicle_acceleration_sub.update(&_vehicle_acceleration);

	// The two sources are mutually exclusive in practice (serial IRLock driver vs MAVLink
	// LANDING_TARGET), so consume at most one measurement per cycle: the filter must never be
	// updated twice with the same prediction step.
	if (_landingTargetReportSub.update(&_landingTargetReport)) {
		_new_target_measurement = _process_landing_target_report(_landingTargetReport);

	} else if (_irlockReportSub.update(&_irlockReport)) {
		// IRLock reports tangents of the angular offsets
		_new_target_measurement = _process_angle_measurement(_irlockReport.pos_x, _irlockReport.pos_y,
					  _irlockReport.timestamp);
	}
}

bool LandingTargetEstimator::_process_angle_measurement(float tan_x, float tan_y, hrt_abstime timestamp)
{
	if (!_vehicleAttitude_valid || !_vehicleLocalPosition_valid || !_vehicleLocalPosition.dist_bottom_valid) {
		// don't have the data needed for an update
		return false;
	}

	if (!PX4_ISFINITE(tan_x) || !PX4_ISFINITE(tan_y)) {
		return false;
	}

	matrix::Vector<float, 3> sensor_ray; // ray pointing towards target in body frame
	sensor_ray(0) = tan_x * _params.scale_x; // forward
	sensor_ray(1) = tan_y * _params.scale_y; // right
	sensor_ray(2) = 1.0f;

	// rotate unit ray according to sensor orientation
	_S_att = get_rot_matrix(_params.sensor_yaw);
	sensor_ray = _S_att * sensor_ray;

	// rotate the unit ray into the navigation frame
	matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
	_R_att = matrix::Dcm<float>(q_att);
	sensor_ray = _R_att * sensor_ray;

	if (fabsf(sensor_ray(2)) < 1e-6f) {
		// z component of measurement unsafe, don't use this measurement
		return false;
	}

	_dist_z = _vehicleLocalPosition.dist_bottom - _params.offset_z;

	// scale the ray s.t. the z component has length of _uncertainty_scale
	_target_position_report.timestamp = timestamp;
	_target_position_report.rel_pos_x = sensor_ray(0) / sensor_ray(2) * _dist_z;
	_target_position_report.rel_pos_y = sensor_ray(1) / sensor_ray(2) * _dist_z;
	_target_position_report.rel_pos_z = _dist_z;

	// Adjust relative position according to sensor offset
	_target_position_report.rel_pos_x += _params.offset_x;
	_target_position_report.rel_pos_y += _params.offset_y;

	return true;
}

hrt_abstime LandingTargetEstimator::_measurement_lag(hrt_abstime measurement_timestamp)
{
	// A sender that leaves the time empty, or a timesync that has not converged, leaves the
	// timestamp at the moment of arrival. Then the configured lag is all we know.
	hrt_abstime lag = (hrt_abstime)(_params.lag * SEC2USEC);
	const hrt_abstime now = hrt_absolute_time();

	if ((measurement_timestamp > 0) && (measurement_timestamp < now)) {
		lag += now - measurement_timestamp;
	}

	_meas_lag_us = math::min(lag, MAX_MEASUREMENT_LAG_US);

	return _meas_lag_us;
}

bool LandingTargetEstimator::_process_landing_target_report(const landing_target_report_s &report)
{
	if (!report.position_valid) {
		// Angle measurements carry tangents, matching the IRLock convention
		return _process_angle_measurement(report.angle_x, report.angle_y, report.timestamp);
	}

	if (report.frame != landing_target_report_s::MAV_FRAME_LOCAL_NED) {
		// Only local NED is supported for position measurements
		return false;
	}

	if (!_vehicleLocalPosition_valid || !_vehicleLocalPosition.xy_valid || !_vehicleLocalPosition.z_valid) {
		// the measurement is absolute, we need our own position to make it relative
		return false;
	}

	if (!PX4_ISFINITE(report.pos_x) || !PX4_ISFINITE(report.pos_y) || !PX4_ISFINITE(report.pos_z)) {
		return false;
	}

	_target_position_report.timestamp = report.timestamp;
	_target_position_report.rel_pos_x = report.pos_x - _vehicleLocalPosition.x + _params.offset_x;
	_target_position_report.rel_pos_y = report.pos_y - _vehicleLocalPosition.y + _params.offset_y;
	_target_position_report.rel_pos_z = report.pos_z - _vehicleLocalPosition.z;

	// The report says where the target was when the sensor saw it, not where it is now. Our own
	// position is measured in the same inertial frame at both instants, so the vehicle's motion in
	// between needs no correction: what is missing is the distance the target itself covered while
	// the measurement was in flight.
	if ((_params.mode == TargetMode::Moving) && _estimator_initialized && _vehicleLocalPosition.v_xy_valid) {
		float rel_pos_x, rel_vel_x, rel_pos_y, rel_vel_y;
		_kalman_filter_x.getState(rel_pos_x, rel_vel_x);
		_kalman_filter_y.getState(rel_pos_y, rel_vel_y);

		// the filter tracks the target relative to the vehicle, so the target's own velocity is
		// that relative velocity plus the vehicle's
		const float lag_s = _measurement_lag(report.timestamp) / SEC2USEC;
		_target_position_report.rel_pos_x += (rel_vel_x + _vehicleLocalPosition.vx) * lag_s;
		_target_position_report.rel_pos_y += (rel_vel_y + _vehicleLocalPosition.vy) * lag_s;
	}

	// Vertical separation drives the measurement noise model. Unlike the angle path this does not
	// need a distance sensor, the report gives us the target altitude directly.
	_dist_z = fabsf(_target_position_report.rel_pos_z);

	return true;
}

void LandingTargetEstimator::_update_params()
{
	param_get(_paramHandle.acc_unc, &_params.acc_unc);
	param_get(_paramHandle.meas_grad, &_params.meas_grad);
	param_get(_paramHandle.meas_base, &_params.meas_base);
	param_get(_paramHandle.pos_unc_init, &_params.pos_unc_init);
	param_get(_paramHandle.vel_unc_init, &_params.vel_unc_init);

	int32_t mode = 0;
	param_get(_paramHandle.mode, &mode);
	_params.mode = (TargetMode)mode;

	param_get(_paramHandle.scale_x, &_params.scale_x);
	param_get(_paramHandle.scale_y, &_params.scale_y);

	int32_t sensor_yaw = 0;
	param_get(_paramHandle.sensor_yaw, &sensor_yaw);
	_params.sensor_yaw = static_cast<enum Rotation>(sensor_yaw);

	param_get(_paramHandle.offset_x, &_params.offset_x);
	param_get(_paramHandle.offset_y, &_params.offset_y);
	param_get(_paramHandle.offset_z, &_params.offset_z);
	param_get(_paramHandle.lag, &_params.lag);
}


} // namespace landing_target_estimator
