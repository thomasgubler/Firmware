/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *
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
 * @file fixedwing_tecs.h
 * Implementation of a fixed wing total energy control system
 * following roughly the proposed TECS in the paper "Analysis and tuning of a 'Total Energy Control System' control low using eigenstructure assignment" by L.F Faleiro & A.A. Lambregts, Aerospace Science and Technology 1999, no. 3, 127-140
 */

#ifndef FIXEDWING_TECS_H_
#define FIXEDWING_TECS_H_

typedef struct {
	float kti;
	float ktp;
	float kep;
	float kei;

	float integral_t;
	float integral_e;

	float ti_max;
	float ei_max;

	float limit_t;
	float limit_e;

} TECS_t;

int tecs_calculate(TECS_t * tecs, float * thrust_sp, float * pitch_sp,
		float flight_path_angle_sp, float flight_path_angle,
		float acceleration_sp, float acceleration,
		float dt);

void tecs_init(TECS_t *tecs, float ktp, float kti, float kep, float kei,
		       float ti_max, float ei_max, float limit_t, float limit_e);

int tecs_set_parameters(TECS_t *tecs, float ktp, float kti, float kep, float kei,
	       float ti_max, float ei_max, float limit_t, float limit_e);

#endif /* FIXEDWING_TECS_H_ */
