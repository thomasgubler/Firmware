/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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
 * @file geo.h
 *
 * Definition of geo / math functions to perform geodesic calculations
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * Additional functions - @author Doug Weibel <douglas.weibel@colorado.edu>
 */


#include <stdbool.h>

struct crosstrack_error_s {
	bool past_end;		// Flag indicating we are past the end of the line/arc segment
	float distance;		// Distance in meters to closest point on line/arc
	float bearing;		// Bearing in radians to closest point on line/arc
};

struct planned_path_segments_s {
	bool segment_type;
	double start_lat;			// Start of line or center of arc
	double start_lon;
	double end_lat;
	double end_lon;
	float radius;				// Radius of arc
	float arc_start_bearing;		// Bearing from center to start of arc
	float arc_sweep;				// Angle (radians) swept out by arc around center.
								// Positive for clockwise, negative for counter-clockwise
};

/**
 * Initializes the map transformation.
 *
 * Initializes the transformation between the geographic coordinate system and the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
__EXPORT void map_projection_init(double lat_0, double lon_0);

/**
 * Transforms a point in the geographic coordinate system to the local azimuthal equidistant plane
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
__EXPORT void map_projection_project(double lat, double lon, float *x, float *y);

/**
 * Transforms a point in the local azimuthal equidistant plane to the geographic coordinate system
 *
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
__EXPORT void map_projection_reproject(float x, float y, double *lat, double *lon);

__EXPORT float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

__EXPORT float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

__EXPORT int get_distance_to_line(struct crosstrack_error_s * crosstrack_error, double lat_now, double lon_now, double lat_start, double lon_start, double lat_end, double lon_end);

__EXPORT int get_distance_to_arc(struct crosstrack_error_s * crosstrack_error, double lat_now, double lon_now, double lat_center, double lon_center,
		float radius, float arc_start_bearing, float arc_sweep);

__EXPORT float _wrap_180(float bearing);
__EXPORT float _wrap_360(float bearing);
__EXPORT float _wrap_pi(float bearing);
__EXPORT float _wrap_2pi(float bearing);

__EXPORT void get_guide_WP_center_of_circle(float last_lat_x, float last_lon_y,
		float current_lat_x, float current_lon_y,
		float next_lat_x, float next_lon_y,
		float * center_lat_x, float * center_lon_y);

__EXPORT float get_circle_bearing(float phi);


/**
 * Calculates arc segment based on 3 points, expects points projected on a plane
 *
 */
__EXPORT void calculate_arc(struct planned_path_segments_s * arc,
		float p1[2], float p2[2], float p3[2],
		float r_min);
