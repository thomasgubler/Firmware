/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file commander_params.c
 *
 * Parameters defined by the sensors task.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include <nuttx/config.h>
#include <systemlib/param/param.h>

PARAM_DEFINE_FLOAT(TRIM_ROLL, 0.0f);
PARAM_DEFINE_FLOAT(TRIM_PITCH, 0.0f);
PARAM_DEFINE_FLOAT(TRIM_YAW, 0.0f);

/**
 * Empty cell voltage.
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_EMPTY, 3.4f);

/**
 * Full cell voltage.
 *
 * Defines the voltage where a single cell of the battery is considered full.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_FULL, 3.9f);

/**
 * Number of cells.
 *
 * Defines the number of cells the attached battery consists of.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT_N_CELLS, 3);

/**
 * Battery capacity.
 *
 * Defines the capacity of the attached battery.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_CAPACITY, -1.0f);


/* Failsafe Handler params */
/**
 * RC loss threshold time
 *
 * After FAIL_RC_TIME seconds of RC loss the failsafe system will kick in, set to -1 to disable
 *
 * @group Failsafe
 */
PARAM_DEFINE_FLOAT(FAIL_RC_TIME, 1.5f);

/**
 * Data link loss threshold time
 *
 * After FAIL_DL_TIME seconds of data link loss the failsafe system will kick in, set to -1 to disable
 *
 * @group Failsafe
 */
PARAM_DEFINE_FLOAT(FAIL_DL_TIME, -1.0f);
/**
 * Failsafe on RC loss in Manual
 *
 * If set to 1 the system will perform failsafe actions (RTL) when in auto mode and RC signal is lost
 *
 * @group Failsafe
 */
PARAM_DEFINE_INT32(FAIL_AUTO_RC, 0);

/**
 * Failsafe wait time on GPS loss in AUTO
 *
 * On a gps loss in auto mode the system tries to loiter (open loop) at the last position for this amount of seconds
 *
 * @group Failsafe
 */
PARAM_DEFINE_FLOAT(FAIL_GPS_WAIT, 0);

/**
 * GPS loss action
 *
 * This sets the action the system performs after a gps loss and after it has waited unsuccessfully for FAIL_GPS_WAIT seconds
 * 0: perform landing without position control
 * 1: switch to manual
 *
 * @group Failsafe
 */
PARAM_DEFINE_FLOAT(FAIL_GPS_ACT, 1);

/**
 * Datalink counter threshold
 *
 * if FAIL_DL_COUN > 0:
 * 	system will RTL after FAIL_DL_COUN data link losses
 * else:
 * 	system will not RTL based on the number of data link losses
 *
 * @group Failsafe

 */
PARAM_DEFINE_INT32(FAIL_DL_COUN, 2);

/**
 * Action on gps loss
 *
 * Determines which action is performed when gps is lost in auto mode:
 * default (0): try to land
 * 1: switch to manual
 *
 * @group Failsafe

 */

PARAM_DEFINE_FLOAT(FAIL_DL_LAT, -1.0f);

/**
 * Comm Loss / Data Link Loss Waypoint Longitude
 *
 * In case of a data link loss the airplane will fly to this point in order to re-establish the datalink)
 *
 * @group Failsafe

 */
PARAM_DEFINE_FLOAT(FAIL_DL_LON, -1.0f);
