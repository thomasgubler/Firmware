/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *   	     @author Anton Babushkin <anton.babushkin@me.com>
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
 * @file failsafe_handler.cpp
 * Provides an update function and calls to failsafe state machine given the current state of the system
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "failsafe_handler.h"

#include <sys/types.h>
#include <float.h>


FailsafeHandler::FailsafeHandler() :
SuperBlock(NULL, "FAIL"),
_position_setpoint_triplet(&getSubscriptions(), ORB_ID(position_setpoint_triplet), 500),
_telemetry_status(&getSubscriptions(), ORB_ID(telemetry_status), 500),
_rc_loss_threshold_seconds(this, "RC_TIME"),
_failsafe_rc_auto_enabled(this, "AUTO_RC"),
_data_loss_threshold_seconds(this, "DL_TIME"),
_data_loss_threshold_counter(this, "DL_COUN"),
_data_loss_wp_lat(this, "DL_LAT"),
_data_loss_wp_lon(this, "DL_LON"),
_data_loss_wp_alt(this, "DL_ALT"),
_gps_loss_loiter_time(this, "GPS_WAIT"),
_gps_loss_action(this, "GPS_ACT"),
_last_timestamp(hrt_absolute_time()),
_rc_loss_timer(0.0f),
_counter_gps_losses(0),
_gps_loss_wait_timer(0.0f),
_counter_comm_losses(0)
{
	updateSubscriptions();
	updateParams();
}

FailsafeHandler::~FailsafeHandler()
{

}

transition_result_t FailsafeHandler::update(vehicle_status_s* status, const actuator_armed_s& armed)
{
	/* Update parameters */
	updateParams();

	/* Update subscriptions which provide additional data */
	updateSubscriptions();


	/* Get time and update last_timestamp*/
	float dt = (float)hrt_elapsed_time(&_last_timestamp) * 1e-6f;
	_last_timestamp = hrt_absolute_time();

	/* First handle everything that leads to flight termination:
	 * - flight termination request
	 * - geofence violation
	 * */
	if (_position_setpoint_triplet.geofence_violated)
	{
		transition_result_t failsafe_res = failsafe_state_transition(status, FAILSAFE_STATE_TERMINATION);
		return failsafe_res;
	}

	/* Update data link loss timer */
	bool data_link_loss_threshold_reached = false;
	float dt_last_heartbeat = (float)hrt_elapsed_time(&_telemetry_status.timestamp_last_gcs_heartbeat) * 1e-6f;
	if (_data_loss_threshold_seconds.get() >= 0 && dt_last_heartbeat >= _data_loss_threshold_seconds.get()) {
		data_link_loss_threshold_reached = true;
	}


	/* Update RC loss timer */
	bool rc_loss_threshold_reached = false;
	if (status->rc_signal_lost) {
		_rc_loss_timer += dt;
		if (_rc_loss_threshold_seconds.get() >= 0 && _rc_loss_timer >= _rc_loss_threshold_seconds.get()) {
			rc_loss_threshold_reached = true;
		}
	} else {
		_rc_loss_timer = 0.0f;
	}


	/* Handle engine failure */
	//XXX
//	if (status->engine_failure) {
//		ransition_result_t failsafe_res = failsafe_state_transition(status, FAILSAFE_STATE_ENGINEFAILURE);
//		return OK
//	}

	/* Handle all failures depending on main state */
	if (status->main_state == MAIN_STATE_AUTO) {

		/* GPS loss, data link loss and combinations */

		/* Increase gps loss counter if this is new occurrence */
		if(!status->condition_global_position_valid && status->failsafe_state != FAILSAFE_STATE_GPS_LOSS_WAIT) {
			_counter_gps_losses++;
		}

		/* Increase comm loss counter if this is new occurrence */
		if(data_link_loss_threshold_reached &&
				status->failsafe_state != FAILSAFE_STATE_COMM_LOSS &&
				status->failsafe_state != FAILSAFE_STATE_COMM_LOSS_RTL) {
			_counter_comm_losses++;
		}

		/* Change states */
		if (!status->condition_global_position_valid && data_link_loss_threshold_reached) {
			transition_result_t failsafe_res = failsafe_state_transition(status, FAILSAFE_STATE_TERMINATION);
			return failsafe_res;
		} else if (status->condition_global_position_valid && !data_link_loss_threshold_reached) {
			transition_result_t failsafe_res = failsafe_state_transition(status, FAILSAFE_STATE_GPS_LOSS_WAIT);
			return failsafe_res;
		} else if (status->condition_global_position_valid && data_link_loss_threshold_reached) {

			transition_result_t failsafe_res;
			/* Fly to comms loss wp if it is available and if we are below the comms loss counter threshold */

			if ( ( _data_loss_threshold_counter.get() > 0 && _counter_comm_losses > _data_loss_threshold_counter.get() ) ||
					_data_loss_wp_lat.get() < FLT_EPSILON || _data_loss_wp_lon.get() <= FLT_EPSILON || _data_loss_wp_alt.get() <= FLT_EPSILON) {
				failsafe_res = failsafe_state_transition(status, FAILSAFE_STATE_COMM_LOSS_RTL);
			} else {
				failsafe_res = failsafe_state_transition(status, FAILSAFE_STATE_COMM_LOSS);
			}

			return failsafe_res;
		}
		/* END gps and data link loss handling */

		/* RC loss, only if failsafe on rc loss is enabled via a param */
		if (rc_loss_threshold_reached && _failsafe_rc_auto_enabled.get() > 0) {
			return handle_rc_loss_auto(status);
		}

	} else if (status->main_state == MAIN_STATE_MANUAL || status->main_state == MAIN_STATE_SEATBELT || status->main_state == MAIN_STATE_EASY) {

		/* RC loss */
		if (rc_loss_threshold_reached)
		{

			return handle_rc_loss_manual(status);
		}

		//XXX handle FPV comms loss
	}


	/* recover or progress failsafe state */
	bool recovered = false;
	if ( status->failsafe_state == FAILSAFE_STATE_RC_LOSS_RTL) {
		if (!armed.armed || !rc_loss_threshold_reached)
			recovered = true;
	} else if ( status->failsafe_state == FAILSAFE_STATE_RC_LOSS_LAND) {
		if (!armed.armed || !rc_loss_threshold_reached)
			recovered = true;
	} else if ( status->failsafe_state == FAILSAFE_STATE_COMM_LOSS) {
		//XXX check if the issue is resolved
	} else if ( status->failsafe_state == FAILSAFE_STATE_COMM_LOSS_RTL) {
		//XXX check if the issue is resolved
	} else if ( status->failsafe_state == FAILSAFE_STATE_GPS_LOSS_WAIT) {
		update_gps_wait(status, dt);
	} else if ( status->failsafe_state == FAILSAFE_STATE_SOFT_GEOFENCE_VIOLATION) {
		//XXX check if the issue is resolved
	}

	if (recovered) {
		transition_result_t failsafe_res = failsafe_state_transition(status, FAILSAFE_STATE_NORMAL);
		return failsafe_res;
	}

	return TRANSITION_NOT_CHANGED;
}

transition_result_t FailsafeHandler::handle_rc_loss_manual(vehicle_status_s* status) {
	transition_result_t res = TRANSITION_NOT_CHANGED;

	/* failsafe for manual modes */
	res = failsafe_state_transition(status, FAILSAFE_STATE_RC_LOSS_RTL);

	if (res == TRANSITION_DENIED) {
		/* RTL not allowed (no global position estimate), try LAND */
		res = failsafe_state_transition(status, FAILSAFE_STATE_RC_LOSS_LAND);

		if (res == TRANSITION_DENIED) {
			/* LAND not allowed, set TERMINATION state */
			res = failsafe_state_transition(status, FAILSAFE_STATE_TERMINATION);
		}
	}

	return res;
}

transition_result_t FailsafeHandler::handle_rc_loss_auto(vehicle_status_s* status) {
	transition_result_t res = TRANSITION_NOT_CHANGED;

	/* check if AUTO mode still allowed */
	res = main_state_transition(status, MAIN_STATE_AUTO);

	if (res == TRANSITION_DENIED) {
		/* AUTO mode denied, don't try RTL, switch to failsafe state LAND */
		res = failsafe_state_transition(status, FAILSAFE_STATE_RC_LOSS_LAND);

		if (res == TRANSITION_DENIED) {
			/* LAND not allowed, set TERMINATION state */
			res = failsafe_state_transition(status, FAILSAFE_STATE_TERMINATION);
		}
	}

	return res;
}

transition_result_t FailsafeHandler::update_gps_wait(vehicle_status_s* status, float dt)
{
	transition_result_t res = TRANSITION_NOT_CHANGED;

	if (status->condition_global_position_valid) {
		/* Have a valid position, reset if this is the first gps loss */
		if (_counter_gps_losses <= 1) {

			res = failsafe_state_transition(status, FAILSAFE_STATE_NORMAL);
			if (res == TRANSITION_CHANGED){
				_gps_loss_wait_timer = 0.0f;
			}
		}
	} else {
		_gps_loss_wait_timer += dt;
	}

	if (_gps_loss_wait_timer >= _gps_loss_loiter_time.get()) {
		/* Did not get GPS lock in gps_loss_loiter_time seconds, depending on the param settings land or switch to manual */
		switch (_gps_loss_action.get()) {
			case 1: //Switch to manual
				if (!status->rc_signal_lost) {
					res = main_state_transition(status, MAIN_STATE_MANUAL);
				}
				break;
			default: // Try to land
				res = failsafe_state_transition(status, FAILSAFE_STATE_GPS_LOSS_LAND);
			break;
		}
	}

	return res;
}
