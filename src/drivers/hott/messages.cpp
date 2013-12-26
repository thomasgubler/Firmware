/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Simon Wilks <sjwilks@gmail.com>
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
 * @file messages.cpp
 *
 */

#include "messages.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <geo/geo.h>
#include <unistd.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <systemlib/err.h>

void
build_gam_request(uint8_t *buffer, size_t *size)
{
	struct gam_module_poll_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.mode = BINARY_MODE_REQUEST_ID;
	msg.id = GAM_SENSOR_ID;

	memcpy(buffer, &msg, *size);
}

void
build_vario_request(uint8_t *buffer, size_t *size)
{
	struct vario_module_poll_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.mode = BINARY_MODE_REQUEST_ID;
	msg.id = VARIO_SENSOR_ID;

	memcpy(buffer, &msg, *size);
}

void
convert_to_degrees_minutes_seconds(double val, int *deg, int *min, int *sec)
{
	*deg = (int)val;

	double delta = val - *deg;
	const double min_d = delta * 60.0d;
	*min = (int)min_d;
	delta = min_d - *min;
	*sec = (int)(delta * 10000.0d);
}
