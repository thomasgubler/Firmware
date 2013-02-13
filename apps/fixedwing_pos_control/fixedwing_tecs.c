/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: 	@author Thomas Gubler <thomasgubler@student.ethz.ch> *
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
 * @file fixedwing_tecs.c
 * Implementation of a fixed wing total energy control system
 * following roughly the proposed TECS in the paper "Analysis and tuning of a 'Total Energy Control System' control low using eigenstructure assignment" by L.F Faleiro & A.A. Lambregts, Aerospace Science and Technology 1999, no. 3, 127-140
 */
 
 #include <fixedwing_tecs.h>

#include <unistd.h>
#include <math.h>
#include <stdio.h>

void tecs_init(TECS_t *tecs, float ktp, float kti, float kep, float kei,
		       float ti_max, float ei_max, float limit_t, float limit_e)
{
	tecs->ktp = ktp;
	tecs->kti = kti;

	tecs->kep = kep;
	tecs->kei = kei;

	tecs->ti_max = ti_max;
	tecs->ei_max = ei_max;

	tecs->limit_t = limit_t;
	tecs->limit_e = limit_e;

	tecs->integral_t = 0.0f;
	tecs->integral_e = 0.0f;
}

int tecs_set_parameters(TECS_t *tecs, float ktp, float kti, float kep, float kei,
	       float ti_max, float ei_max, float limit_t, float limit_e)
{
	tecs->ktp = ktp;
	tecs->kti = kti;

	tecs->kep = kep;
	tecs->kei = kei;

	tecs->ti_max = ti_max;
	tecs->ei_max = ei_max;

	tecs->limit_t = limit_t;
	tecs->limit_e = limit_e;

	return OK;
}

int tecs_calculate(TECS_t * tecs, float * thrust_sp, float * pitch_sp,
		float flight_path_angle_sp, float flight_path_angle,
		float acceleration_sp, float acceleration,
		float dt)
{
//	printf("flight_path_angle_sp %.4f, flight_path_angle %.4f, acceleration_sp %.4f, acceleration: %.4f \n", (double)flight_path_angle_sp, (double) flight_path_angle, (double)acceleration_sp, (double)acceleration);

	float flight_path_angle_error = flight_path_angle_sp - flight_path_angle;
	float acceleration_error = acceleration_sp - acceleration;

//	printf("flight_path_angle_error: %.4f, acceleration_error: %.4f\n", flight_path_angle_error, acceleration_error);

	float specific_total_energy_rate_error = flight_path_angle_error + acceleration_error;
	float specific_total_energy_rate = -flight_path_angle - acceleration;
	float specific_energy_distribution_rate = flight_path_angle - acceleration;
	float specific_energy_distribution_rate_error = -flight_path_angle_error + acceleration_error;

//	printf("specific_total_energy_rate_error: %.4f, specific_total_energy_rate: %.4f\nspecific_energy_distribution_rate: %.4f, specific_energy_distribution_rate_error: %.4f\n", specific_total_energy_rate_error, specific_total_energy_rate, specific_energy_distribution_rate, specific_energy_distribution_rate_error);

	/* Thrust */
	float i = tecs->integral_t + (specific_total_energy_rate_error * dt);
//	printf("tecs->integral_t: %.4f, i: %.4f; dt: %.4f\n", (double)tecs->integral_t, (double)i, (double)dt);
	if (fabsf(i) > tecs->ti_max) {
		i = tecs->integral_t;		// If saturated then do not update integral value

	} else {
		if (!isfinite(i)) {
			i = 0;
		}

		tecs->integral_t = i;
	}

//	printf("tecs->integral_t %.4f\n", tecs->integral_t);

//	printf("tecs->ktp: %.4f, tecs->kti: %.4f, tecs->limit_t: %.4f\n", (double)tecs->ktp, (double)tecs->kti, (double)tecs->limit_t);
	float output = specific_total_energy_rate * tecs->ktp + i * tecs->kti;
	if (output > tecs->limit_t) {
		output = tecs->limit_t;
	}
	if (output < -tecs->limit_t) {
		output = -tecs->limit_t;
	}

//	printf("thrust_sp:output %.4f\n", (double)output);

	if (isfinite(output)) {
		*thrust_sp = output;
	}


	/* Pitch */
	i = tecs->integral_e + (specific_energy_distribution_rate_error * dt);
	if (fabsf(i) > tecs->ei_max) {
		i = tecs->integral_e;		// If saturated then do not update integral value

	} else {
		if (!isfinite(i)) {
			i = 0;
		}

		tecs->integral_e = i;
	}

//	printf("tecs->integral_e %.4f\n", tecs->integral_e);

	output = -(specific_energy_distribution_rate * tecs->kep + i * tecs->kei); // additional -, because of definition of elevator deflection

	if (output > tecs->limit_e) {
		output = tecs->limit_e;
	}
	if (output < -tecs->limit_e) {
		output = -tecs->limit_e;
	}


	if (isfinite(output)) {
		*pitch_sp = output;
	}


	return OK;
}
