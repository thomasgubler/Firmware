/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@gmail.com>
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
 * @file failsafe_navigator.cpp
 * Navigator functions for operation in failsafe mode
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "failsafe_navigator.h"

#include <sys/types.h>
#include <string.h>

FailsafeNavigator::FailsafeNavigator() :
SuperBlock(NULL, "NAV"),
_data_loss_wp_lat(this, "FAIL_DL_LAT", false),
_data_loss_wp_lon(this, "FAIL_DL_LON", false),
_data_loss_wp_alt(this, "FAIL_DL_ALT", false)
{

}

FailsafeNavigator::~FailsafeNavigator()
{

}

int FailsafeNavigator::navigate_failsafe_commloss(mission_item_s * mission_item, float loiter_radius, float acceptance_radius)
{
	updateParams();

	mission_item->lat = _data_loss_wp_lat.get();
	mission_item->lon = _data_loss_wp_lon.get();
	mission_item->altitude_is_relative = false;
	mission_item->altitude = _data_loss_wp_alt.get();
	mission_item->yaw = NAN;
	mission_item->loiter_radius = loiter_radius;
	mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	mission_item->acceptance_radius = acceptance_radius;
	mission_item->time_inside = 0.0f;
	mission_item->pitch_min = 0.0f;
	mission_item->autocontinue = false;
	mission_item->origin = ORIGIN_ONBOARD;

	return OK;
}

