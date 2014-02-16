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
 * @file failsafe_handler.h
 * Provides an update function and calls to failsafe state machine given the current state of the system
 */

#ifndef FAILSAFEHANDLER_H_
#define FAILSAFEHANDLER_H_

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <controllib/uorb/UOrbSubscription.hpp>
#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>

#include "state_machine_helper.h"

class FailsafeHandler {
public:
	FailsafeHandler();
	virtual ~FailsafeHandler();

	transition_result_t update(vehicle_status_s* status, const actuator_armed_s& armed);
protected:
private:
	/* Subscriptions */
	void updateSubscriptions();
	control::UOrbSubscription<position_setpoint_triplet_s> _position_setpoint_triplet;          /**< position_setpoint_triplet_s sub from navigator */

	/* Params */
	void updateParams();
	control::BlockParamFloat rc_loss_threshold_seconds;
	control::BlockParamFloat data_loss_threshold_seconds;
	control::BlockParamInt failsafe_rc_auto_enabled;

	hrt_abstime last_timestamp;		/**< Timestamp of last update */

	float rc_loss_timer;			/**< Counts the time of RC loss in seconds */
	float data_link_loss_timer;		/**< Counts the time of data link loss in seconds */

	transition_result_t handle_rc_loss_manual(vehicle_status_s* status);
	transition_result_t handle_rc_loss_auto(vehicle_status_s* status);
};

#endif /* FAILSAFEHANDLER_H_ */
