/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: 	@author Thomas Gubler <thomasgubler@student.ethz.ch>
 *   			@author Doug Weibel <douglas.weibel@colorado.edu>
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
 * @file fixedwing_pos_control.c
 * Implementation of a fixed wing attitude controller.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/debug_key_value.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */
PARAM_DEFINE_FLOAT(FW_HEAD_P, 0.1f);
PARAM_DEFINE_FLOAT(FW_HEADR_I, 0.1f);
PARAM_DEFINE_FLOAT(FW_HEADR_LIM, 1.5f); //TODO: think about reasonable value
PARAM_DEFINE_FLOAT(FW_HEADR_AWU, 1.0f);
PARAM_DEFINE_FLOAT(FW_XTRACK_P, 0.01745f);
PARAM_DEFINE_FLOAT(FW_XTRACK_D, 0.01745f);
PARAM_DEFINE_FLOAT(FW_XTRACK_I, 0.01745f);
PARAM_DEFINE_FLOAT(FW_XTRACK_AWU, 0.01);
PARAM_DEFINE_FLOAT(FW_ALT_P, 0.1f);
PARAM_DEFINE_FLOAT(FW_ROLL_LIM, 0.7f);	// Roll angle limit in radians
PARAM_DEFINE_FLOAT(FW_HEADR_P, 0.1f);
PARAM_DEFINE_FLOAT(FW_PITCH_LIM, 0.35f);	/**< Pitch angle limit in radians per second */

struct fw_pos_control_params {
	float heading_p;
	float headingr_p;
	float headingr_i;
	float headingr_awu;
	float headingr_lim;
	float xtrack_p;
	float xtrack_d;
	float xtrack_i;
	float xtrack_awu;
	float altitude_p;
	float roll_lim;
	float pitch_lim;
};

struct fw_pos_control_param_handles {
	param_t heading_p;
	param_t headingr_p;
	param_t headingr_i;
	param_t headingr_awu;
	param_t headingr_lim;
	param_t xtrack_p;
	param_t xtrack_d;
	param_t xtrack_i;
	param_t xtrack_awu;
	param_t altitude_p;
	param_t roll_lim;
	param_t pitch_lim;
};

/* Prototypes */
/* Internal Prototypes */
static int parameters_init(struct fw_pos_control_param_handles *h);
static int parameters_update(const struct fw_pos_control_param_handles *h, struct fw_pos_control_params *p);

/**
 * Deamon management function.
 */
__EXPORT int fixedwing_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int fixedwing_pos_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Variables */
static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */


/**
 * Parameter management
 */
static int parameters_init(struct fw_pos_control_param_handles *h)
{
	/* PID parameters */
	h->heading_p 		=	param_find("FW_HEAD_P");
	h->headingr_p 		=	param_find("FW_HEADR_P");
	h->headingr_i 		=	param_find("FW_HEADR_I");
	h->headingr_awu 		=	param_find("FW_HEADR_AWU");
	h->headingr_lim 	=	param_find("FW_HEADR_LIM");
	h->xtrack_p 		=	param_find("FW_XTRACK_P");
	h->xtrack_d 		=	param_find("FW_XTRACK_D");
	h->xtrack_i 		=	param_find("FW_XTRACK_I");
	h->xtrack_awu 		=	param_find("FW_XTRACK_AWU");
	h->altitude_p 		=	param_find("FW_ALT_P");
	h->roll_lim 		=	param_find("FW_ROLL_LIM");
	h->pitch_lim 		=	param_find("FW_PITCH_LIM");

	return OK;
}

static int parameters_update(const struct fw_pos_control_param_handles *h, struct fw_pos_control_params *p)
{
	param_get(h->heading_p, &(p->heading_p));
	param_get(h->headingr_p, &(p->headingr_p));
	param_get(h->headingr_i, &(p->headingr_i));
	param_get(h->headingr_awu, &(p->headingr_awu));
	param_get(h->headingr_lim, &(p->headingr_lim));
	param_get(h->xtrack_p, &(p->xtrack_p));
	param_get(h->xtrack_d, &(p->xtrack_d));
	param_get(h->xtrack_i, &(p->xtrack_i));
	param_get(h->xtrack_awu, &(p->xtrack_awu));
	param_get(h->altitude_p, &(p->altitude_p));
	param_get(h->roll_lim, &(p->roll_lim));
	param_get(h->pitch_lim, &(p->pitch_lim));

	return OK;
}


/* Main Thread */
int fixedwing_pos_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
		bool verbose = false;

		for (int i = 1; i < argc; i++) {
			if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
				verbose = false;
			}
		}

		/* welcome user */
		printf("[fixedwing pos control] started\n");

//		//XXX testing
//		struct planned_path_segments_s  arc;
//		float p1[2] = {-5.0f, -15.0f};
//		float p2[2] = {0.0f, 0.0f};
//		float p3[2] = {5.0f, -5.0f};
//		float r_min = 3;
//		calculate_arc(&arc,
//				p1, p2, p3,
//				r_min);

		/* declare and safely initialize all structs */
		struct vehicle_global_position_s global_pos;
		memset(&global_pos, 0, sizeof(global_pos));
		struct vehicle_global_position_s start_pos;		// Temporary variable, replace with
		memset(&start_pos, 0, sizeof(start_pos));		// previous waypoint when available
		struct vehicle_global_position_setpoint_s global_setpoint;
		memset(&global_setpoint, 0, sizeof(global_setpoint));
		struct vehicle_attitude_s att;
		memset(&att, 0, sizeof(att));
		struct crosstrack_error_s xtrack_err;
		memset(&xtrack_err, 0, sizeof(xtrack_err));
		struct parameter_update_s param_update;
		memset(&param_update, 0, sizeof(param_update));
		struct vehicle_status_s vehicle_status;
		memset(&vehicle_status, 0, sizeof(vehicle_status));

		/* output structs */
		struct vehicle_attitude_setpoint_s attitude_setpoint;
		memset(&attitude_setpoint, 0, sizeof(attitude_setpoint));

		/* publish attitude setpoint */
		attitude_setpoint.roll_body = 0.0f;
		attitude_setpoint.pitch_body = 0.0f;
		attitude_setpoint.yaw_body = 0.0f;
		attitude_setpoint.thrust = 0.0f;
		orb_advert_t attitude_setpoint_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &attitude_setpoint);

		/* subscribe */
		int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
		int global_setpoint_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
		int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
		int param_sub = orb_subscribe(ORB_ID(parameter_update));
		int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

		/* Setup of loop */
		struct pollfd fds[2] = {
			{ .fd = param_sub, .events = POLLIN },
			{ .fd = att_sub, .events = POLLIN }
		};
		bool global_sp_updated_set_once = false;
		bool global_pos_set_once = true;

		float psi_track = 0.0f;

		int counter = 0;

		struct fw_pos_control_params p;
		struct fw_pos_control_param_handles h;

		PID_t heading_controller;
		PID_t heading_rate_controller;
		PID_t offtrack_controller;
		PID_t altitude_controller;

		parameters_init(&h);
		parameters_update(&h, &p);
		pid_init(&heading_controller, p.heading_p, 0.0f, 0.0f, 0.0f, 10000.0f, PID_MODE_DERIVATIV_NONE); //arbitrary high limit
		pid_init(&heading_rate_controller, p.headingr_p, p.headingr_i, 0.0f, p.headingr_awu, p.roll_lim, PID_MODE_DERIVATIV_NONE);
		pid_init(&altitude_controller, p.altitude_p, 0.0f, 0.0f, 0.0f, p.pitch_lim, PID_MODE_DERIVATIV_NONE);
		pid_init(&offtrack_controller, p.xtrack_p, p.xtrack_i, p.xtrack_d, p.xtrack_awu , 30.0f*M_DEG_TO_RAD_F, PID_MODE_DERIVATIV_CALC); //TODO: remove hardcoded value

		float r_min = fabs(30.0f*30.0f/(9.81*tanf(p.roll_lim))) +10; //V^2/(9.81*tan(roll_max) //XXX: remove hardcoded value

		/* Horizontal Navigation State */
		typedef enum {
			HNAV_LINE = 0,
			HNAV_ARC
		}
		horizontal_navigation_state_t;
		horizontal_navigation_state_t horizontal_navigation_state = HNAV_LINE;
		struct vehicle_global_position_setpoint_s current_navigation_setpoint;
		memset(&current_navigation_setpoint, 0, sizeof(current_navigation_setpoint));

		struct planned_path_segments_s arc;
		memset(&arc, 0, sizeof(arc));

		/* error and performance monitoring */
		perf_counter_t fw_interval_perf = perf_alloc(PC_INTERVAL, "fixedwing_pos_control_interval");
		perf_counter_t fw_err_perf = perf_alloc(PC_COUNT, "fixedwing_pos_control_err");

		bool wp_reached = false;

		/* advertise debug values */
		struct debug_key_value_s dbg_xte = { .key = "xte", .value = 0.0f };
		orb_advert_t pub_dbg_xte = orb_advertise(ORB_ID(debug_key_value), &dbg_xte);
		struct debug_key_value_s dbg_delta_psi_c = { .key = "dpc", .value = 0.0f };
		orb_advert_t pub_dbg_delta_psi_c = orb_advertise(ORB_ID(debug_key_value), &dbg_delta_psi_c);

		while(!thread_should_exit)
		{
			/* wait for a sensor update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			static int counter = 0;

			static uint64_t last_run = 0;
			last_run = hrt_absolute_time();

			if (ret < 0) {
				/* poll error, count it in perf */
				perf_count(fw_err_perf);
			} else if (ret == 0) {
				/* no return value, ignore */
			} else {

				/* only update parameters if they changed */
				if (fds[0].revents & POLLIN) {
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), param_sub, &update);

					/* update parameters from storage */
					parameters_update(&h, &p);
					pid_set_parameters(&heading_controller, p.heading_p, 0, 0, 0, 10000.0f); //arbitrary high limit
					pid_set_parameters(&heading_rate_controller, p.headingr_p, p.headingr_i, 0, p.headingr_awu, p.roll_lim);
					pid_set_parameters(&altitude_controller, p.altitude_p, 0, 0, 0, p.pitch_lim);
					pid_set_parameters(&offtrack_controller, p.xtrack_p, p.xtrack_i, p.xtrack_d, p.xtrack_awu, 30.0f*M_DEG_TO_RAD); //TODO: remove hardcoded value
					r_min = fabs(30.0f*30.0f/(9.81*tanf(p.roll_lim))) + 10; //V^2/(9.81*tan(roll_max) + margin //XXX: remove hardcoded value

				}

				/* only run controller if attitude changed */
				if (fds[1].revents & POLLIN) {

					static uint64_t last_run = 0;
					const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
					last_run = hrt_absolute_time();

					/* check if there is a new position or setpoint */
					bool pos_updated;
					orb_check(global_pos_sub, &pos_updated);
					bool global_sp_updated;
					orb_check(global_setpoint_sub, &global_sp_updated);

					/* update vehicle status if necessary */
					bool vehicle_status_updated;
					orb_check(vehicle_status_sub, &vehicle_status_updated);
					orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);

					/* load local copies */
					orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

					if (pos_updated) {
						orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
						global_pos_set_once = true;
					}

					if (global_sp_updated && global_pos_set_once && vehicle_status.flag_global_position_valid) {

						/* waypoint reached */

						orb_copy(ORB_ID(vehicle_global_position_setpoint), global_setpoint_sub, &global_setpoint);
						wp_reached = true;
						if(!global_sp_updated_set_once || global_setpoint.index == current_navigation_setpoint.index) { //only update the navigation setpoint here if it's empty or if the setpoint is the same as the old one (same index) but has been moved around
							/* init navigation */
							/* load next wp to current_navigation_setpoint */
							start_pos = global_pos; //for now using the current position as the startpoint (= approx. last waypoint because the setpoint switch occurs at the waypoint)
							current_navigation_setpoint = global_setpoint;

							/* check if next wp is a navigation wp -> if yes calculate arc */
							if(global_setpoint.waypoint_navigation == WP_NAV_GUIDE) {
								printf("calculating arc");
								calculate_arc(&arc,
										(double)start_pos.lat / (double)1e7d, (double)start_pos.lon / (double)1e7d, (double)global_setpoint.lat / (double)1e7d, (double)global_setpoint.lon / (double)1e7d, (double)global_setpoint.lat_next / (double)1e7d, (double)global_setpoint.lon_next / (double)1e7d,
										r_min);

								current_navigation_setpoint.lat = arc.navpoint1_lat * 1e7d;
								current_navigation_setpoint.lon = arc.navpoint1_lon * 1e7d;

								printf("center latlon: %0.4f, %0.4f \n", arc.start_lat, arc.start_lon);
								printf("arc.navpoint1_latlon: %0.4f, %0.4f \n", arc.navpoint1_lat, arc.navpoint1_lon);
								printf("arc.navpoint2_latlon: %0.4f, %0.4f \n", arc.navpoint2_lat, arc.navpoint2_lon);
								printf("arc.arc_start_bearing: %0.4f, arc.arc_sweep: %0.4f \n", (double)arc.arc_start_bearing*180.0/M_PI, (double)arc.arc_sweep*180.0/M_PI);
								printf("arc.radius: %0.4f\n", (double)arc.radius);

								wp_reached = false;
							}

							psi_track = get_bearing_to_next_waypoint((double)global_pos.lat / (double)1e7d, (double)global_pos.lon / (double)1e7d,
																						(double)current_navigation_setpoint.lat / (double)1e7d, (double)current_navigation_setpoint.lon / (double)1e7d);
							printf("next navigation point direction: %0.4f deg\n", (double)psi_track * 180.0 / M_PI);

							horizontal_navigation_state = HNAV_LINE;


						}

						global_sp_updated_set_once = true;


					}

					/* Simple Horizontal Control (very simple path planning) */
					if(global_sp_updated_set_once)
					{
		//				if (counter % 100 == 0)
		//					printf("lat_sp %d, ln_sp %d, lat: %d, lon: %d\n", global_setpoint.lat, global_setpoint.lon, global_pos.lat, global_pos.lon);

						/* calculate crosstrack error */
						// Only the case of a straight line track following handled so far

						int distance_res;
						/* if current waypoint is a guide wp --> check if we need to switch to the horizontal navigation state (eg. change from line to arc and vice versa */
						if (horizontal_navigation_state == HNAV_LINE ) {

							 distance_res = get_distance_to_line(&xtrack_err, (double)global_pos.lat / (double)1e7d, (double)global_pos.lon / (double)1e7d,
															(double)start_pos.lat / (double)1e7d, (double)start_pos.lon / (double)1e7d,
															(double)current_navigation_setpoint.lat / (double)1e7d, (double)current_navigation_setpoint.lon / (double)1e7d);


							if (current_navigation_setpoint.waypoint_navigation == WP_NAV_GUIDE) {

								if(xtrack_err.past_end && arc.valid) {
									/* switch to arc */
									printf("switch to arc\n");
									horizontal_navigation_state = HNAV_ARC; //the arc is already calculated!
								} else if(xtrack_err.past_end && !arc.valid) {
									if(wp_reached) {


										/* load next wp to current_navigation_setpoint */ //XXX: move this code block into function
		//								start_pos.lat = current_navigation_setpoint.lat_next; //current start position is old goal/waypoint
		//								start_pos.lon = current_navigation_setpoint.lon_next;
		//								start_pos.alt = current_navigation_setpoint.altitude;
										start_pos = global_pos; //for now using the current position as the startpoint (= approx. last waypoint because the setpoint switch occurs at the waypoint)
										current_navigation_setpoint = global_setpoint;

										/* check if next wp is a navigation wp -> if yes calculate arc */
										if(global_setpoint.waypoint_navigation == WP_NAV_GUIDE) {
											printf("calculating arc\n");
											calculate_arc(&arc,
													(double)start_pos.lat / (double)1e7d, (double)start_pos.lon / (double)1e7d, (double)global_setpoint.lat / (double)1e7d, (double)global_setpoint.lon / (double)1e7d, (double)global_setpoint.lat_next / (double)1e7d, (double)global_setpoint.lon_next / (double)1e7d,
													r_min);

											current_navigation_setpoint.lat = arc.navpoint1_lat * 1e7d;
											current_navigation_setpoint.lon = arc.navpoint1_lon * 1e7d;

											printf("center latlon: %0.4f, %0.4f \n", arc.start_lat, arc.start_lon);
											printf("arc.navpoint1_latlon: %0.4f, %0.4f \n", arc.navpoint1_lat, arc.navpoint1_lon);
											printf("arc.navpoint2_latlon: %0.4f, %0.4f \n", arc.navpoint2_lat, arc.navpoint2_lon);
											printf("arc.arc_start_bearing: %0.4f, arc.arc_sweep: %0.4f \n", arc.arc_start_bearing*180/M_PI, arc.arc_sweep*180/M_PI);
										}

										psi_track = get_bearing_to_next_waypoint((double)global_pos.lat / (double)1e7d, (double)global_pos.lon / (double)1e7d,
																									(double)current_navigation_setpoint.lat / (double)1e7d, (double)current_navigation_setpoint.lon / (double)1e7d);
										wp_reached = false;

									}
									else
									{
										/*past the setpoint but missed it, have no valid arcs --> try to turn back to the setpoint */
										start_pos = global_pos;
										psi_track = get_bearing_to_next_waypoint((double)global_pos.lat / (double)1e7d, (double)global_pos.lon / (double)1e7d,
														(double)current_navigation_setpoint.lat / (double)1e7d, (double)current_navigation_setpoint.lon / (double)1e7d);
										printf("Missed wp, new try (line mode, no valid arc)\n");


									}
								}
							}
							else {

								if(xtrack_err.past_end && !wp_reached) {

									/* try again */
									start_pos = global_pos;
									psi_track = get_bearing_to_next_waypoint((double)global_pos.lat / (double)1e7d, (double)global_pos.lon / (double)1e7d,
											(double)current_navigation_setpoint.lat / (double)1e7d, (double)current_navigation_setpoint.lon / (double)1e7d);
									printf("Missed wp, new try (line mode)\n");

								}
							}

						} else if (horizontal_navigation_state == HNAV_ARC) {

							distance_res = get_distance_to_arc(&xtrack_err, (double)global_pos.lat / (double)1e7d, (double)global_pos.lon / (double)1e7d,
										arc.start_lat, arc.start_lon,
										arc.radius, arc.arc_start_bearing, arc.arc_sweep, wp_reached);

							if(xtrack_err.past_end) {

								/* load next wp to current_navigation_setpoint */  //XXX: move this code block into function
//								start_pos.lat = current_navigation_setpoint.lat_next; //current start position is old goal/waypoint
//								start_pos.lon = current_navigation_setpoint.lon_next;
//								start_pos.alt = current_navigation_setpoint.altitude;
								start_pos = global_pos; //for now using the current position as the startpoint (= approx. last waypoint because the setpoint switch occurs at the waypoint)
								current_navigation_setpoint = global_setpoint;

								/* check if next wp is a navigation wp -> if yes calculate arc */
								if(global_setpoint.waypoint_navigation == WP_NAV_GUIDE) {
									printf("calculating arc\n");
									calculate_arc(&arc,
											(double)start_pos.lat / (double)1e7d, (double)start_pos.lon / (double)1e7d, (double)global_setpoint.lat / (double)1e7d, (double)global_setpoint.lon / (double)1e7d, (double)global_setpoint.lat_next / (double)1e7d, (double)global_setpoint.lon_next / (double)1e7d,
											r_min);

									current_navigation_setpoint.lat = arc.navpoint1_lat * 1e7d;
									current_navigation_setpoint.lon = arc.navpoint1_lon * 1e7d;

									printf("center latlon: %0.4f, %0.4f \n", arc.start_lat, arc.start_lon);
									printf("arc.navpoint1_latlon: %0.4f, %0.4f \n", arc.navpoint1_lat, arc.navpoint1_lon);
									printf("arc.navpoint2_latlon: %0.4f, %0.4f \n", arc.navpoint2_lat, arc.navpoint2_lon);
									printf("arc.arc_start_bearing: %0.4f, arc.arc_sweep: %0.4f \n", arc.arc_start_bearing*180/M_PI, arc.arc_sweep*180/M_PI);
								}

								psi_track = get_bearing_to_next_waypoint((double)global_pos.lat / (double)1e7d, (double)global_pos.lon / (double)1e7d,
																							(double)current_navigation_setpoint.lat / (double)1e7d, (double)current_navigation_setpoint.lon / (double)1e7d);
								wp_reached = false;

								/* switch to line */
								printf("switch to line\n");
								horizontal_navigation_state = HNAV_LINE;
							}
						}

						dbg_xte.value = xtrack_err.distance;
						orb_publish(ORB_ID(debug_key_value), pub_dbg_xte, &dbg_xte);

						if(distance_res == OK /*&& !xtrack_err.past_end*/) {
//							if (counter % 10 == 0) {
//								printf("distance = %0.4f\n",  xtrack_err.distance);
//							}

							float delta_psi_c = pid_calculate(&offtrack_controller, 0, xtrack_err.distance, 0.0f, deltaT); //p.xtrack_p * xtrack_err.distanc
							//note: delta_psi_c is limited by the limit of the offtrack_controller

//							printf("xtrack_err.distance %.4f , delta_psi_c %.4f, radius : %.4f\n", (double)xtrack_err.distance, (double)delta_psi_c, (double)arc.radius);

//							dbg_delta_psi_c.value = delta_psi_c;
//							orb_publish(ORB_ID(debug_key_value), pub_dbg_delta_psi_c, &dbg_delta_psi_c);

							float psi_c;
							if (horizontal_navigation_state == HNAV_ARC) {
								psi_c = xtrack_err.bearing + delta_psi_c; //follow the circle
							} else {
								psi_c = psi_track + delta_psi_c; //following the track bearing
							}

							float psi_e = psi_c - att.yaw;

							/* wrap difference back onto -pi..pi range */
							psi_e = _wrap_pi(psi_e);

							if (verbose) {
								printf("xtrack_err.distance %.4f ", (double)xtrack_err.distance);
								printf("xtrack_err.bearing %.4f ", (double)xtrack_err.bearing);
								printf("psi_track %.4f ", (double)psi_track);
								printf("delta_psi_c %.4f ", (double)delta_psi_c);
								printf("psi_c %.4f ", (double)psi_c);
								printf("att.yaw %.4f ", (double)att.yaw);
								printf("psi_e %.4f ", (double)psi_e);
							}

							/* calculate roll setpoint, do this artificially around zero */
							float delta_psi_rate_c = pid_calculate(&heading_controller, psi_e, 0.0f, 0.0f, 0.0f);
							float psi_rate_track = 0;
							if (horizontal_navigation_state == HNAV_ARC) {
								psi_rate_track = sqrtf(global_pos.vx * global_pos.vx +  global_pos.vy * global_pos.vy) / arc.radius;//=V_gr/r_track

								if (arc.arc_sweep < 0) { //counter clockwise
									psi_rate_track *= -1;
								}
							}
							float psi_rate_c = delta_psi_rate_c + psi_rate_track;

							/* limit turn rate */
							if(psi_rate_c > p.headingr_lim) {
								psi_rate_c = p.headingr_lim;
							} else if(psi_rate_c < -p.headingr_lim) {
								psi_rate_c = -p.headingr_lim;
							}

							float psi_rate_e = psi_rate_c - att.yawspeed;

							// XXX sanity check: Assume 10 m/s stall speed and no stall condition
							float ground_speed = sqrtf(global_pos.vx * global_pos.vx + global_pos.vy * global_pos.vy);

							if (ground_speed < 10.0f) {
								ground_speed = 10.0f;
							}

							float psi_rate_e_scaled = psi_rate_e * ground_speed / 9.81f; //* V_gr / g

							attitude_setpoint.roll_body = pid_calculate(&heading_rate_controller, psi_rate_e_scaled, 0.0f, 0.0f, deltaT);

							if (verbose) {
								printf("psi_rate_c %.4f ", (double)psi_rate_c);
								printf("psi_rate_e_scaled %.4f ", (double)psi_rate_e_scaled);
								printf("rollbody %.4f\n", (double)attitude_setpoint.roll_body);
							}

							if (verbose && counter % 100 == 0)
								printf("xtrack_err.distance: %0.4f, delta_psi_c: %0.4f\n",xtrack_err.distance, delta_psi_c);
						} else {
							if (verbose && counter % 100 == 0)
								printf("distance_res: %d, past_end %d\n", distance_res, xtrack_err.past_end);
						}

						/* Very simple Altitude Control */
						if(pos_updated)
						{

							//TODO: take care of relative vs. ab. altitude
							attitude_setpoint.pitch_body = pid_calculate(&altitude_controller, current_navigation_setpoint.altitude, global_pos.alt, 0.0f, 0.0f);

						}

						// XXX need speed control
						attitude_setpoint.thrust = 0.7f;

						/* publish the attitude setpoint */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), attitude_setpoint_pub, &attitude_setpoint);

						/* measure in what intervals the controller runs */
						perf_count(fw_interval_perf);

						counter++;

					} else {
						// XXX no setpoint, decent default needed (loiter?)

						attitude_setpoint.roll_body = 0.0f;
						attitude_setpoint.pitch_body = 0.0f;
						attitude_setpoint.yaw_body = 0.0f;
						attitude_setpoint.thrust = 0.7f;

						/* publish the attitude setpoint */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), attitude_setpoint_pub, &attitude_setpoint);
					}
				}
			}

			counter++;
		}

		printf("[fixedwing_pos_control] exiting.\n");
		thread_running = false;


		close(attitude_setpoint_pub);

		fflush(stdout);
		exit(0);

		return 0;

}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: fixedwing_pos_control {start|stop|status}\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int fixedwing_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("fixedwing_pos_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("fixedwing_pos_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 4000,
					 fixedwing_pos_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tfixedwing_pos_control is running\n");
		} else {
			printf("\tfixedwing_pos_control not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
