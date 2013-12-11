/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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
 * @file hott_sensors.cpp
 * @author Simon Wilks <sjwilks@gmail.com>
 *
 * Graupner HoTT sensor driver implementation.
 *
 * Poll any sensors connected to the PX4 via the telemetry wire.
 */

#include <fcntl.h>
#include <nuttx/config.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include "../comms.h"
#include "../messages.h"
#include "../uorb_interface.h"

#define DEFAULT_UART "/dev/ttyS0";		/**< DSM input (USART1) */

/* Oddly, ERROR is not defined for C++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static int thread_should_exit = false;		/**< Deamon exit flag */
static int thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const char daemon_name[] = "hott_sensors";
static const char commandline_usage[] = "usage: hott_sensors start|status|stop [-d <device>] [-t <type>]";

/**
 * Deamon management function.
 */
extern "C" __EXPORT int hott_sensors_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int hott_sensors_thread_main(int argc, char *argv[]);

int
hott_sensors_thread_main(int argc, char *argv[])
{
	warnx("starting");

	thread_running = true;

	const char *device = DEFAULT_UART;
	hott_sensor_type_t hott_sensor_type = HOTT_SENSOR_TYPE_GAM;

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				thread_running = false;
				errx(1, "missing parameter to -d\n%s", commandline_usage);
			}
		} else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--type") == 0) { //type
			if (argc > i + 1) {
				if ( strcmp( argv[i + 1], "gam") == 0 ) {
					hott_sensor_type = HOTT_SENSOR_TYPE_GAM;
				} else if ( strcmp( argv[i + 1], "vario") == 0 ) {
					hott_sensor_type = HOTT_SENSOR_TYPE_VARIO;
				} else {
					errx(1, "unsupported sensor type %s",  argv[i + 1]);
				}

			} else {
				thread_running = false;
				errx(1, "missing parameter to -t\n%s", commandline_usage);
			}

		}
	}

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	const int uart = open_uart(device);
	if (uart < 0) {
		errx(1, "Failed opening HoTT UART, exiting.");
		thread_running = false;
	}

	init_pub_messages();

	uint8_t buffer[MAX_MESSAGE_BUFFER_SIZE];
	size_t size = 0;
	uint8_t id = 0;
	while (!thread_should_exit) {
		/* Send poll request */
		switch (hott_sensor_type) {
		case HOTT_SENSOR_TYPE_GAM:
			build_gam_request(&buffer[0], &size);
			break;
		case HOTT_SENSOR_TYPE_VARIO:
			build_vario_request(&buffer[0], &size);
			break;
		default:
			warnx("Unknown sensor type");
			break;
		}

		send_poll(uart, buffer, size);

		// The sensor will need a little time before it starts sending.
		usleep(5000);

		recv_data(uart, &buffer[0], &size, &id);

		// Determine which module sent it and process accordingly.
		if (id == GAM_SENSOR_ID) {
			publish_gam_message(buffer);
		} else if (id == VARIO_SENSOR_ID) {
			publish_vario_message(buffer);
		}
		else {
			warnx("Unknown sensor ID: %d", id);
		}
	}

	warnx("exiting");
	close(uart);
	thread_running = false;

	return 0;
}

/**
 * Process command line arguments and start the daemon.
 */
int
hott_sensors_main(int argc, char *argv[])
{
	if (argc < 1) {
		errx(1, "missing command\n%s", commandline_usage);
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("deamon already running");
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd(daemon_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 40,
					     1024,
					     hott_sensors_thread_main,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("daemon is running");

		} else {
			warnx("daemon not started");
		}

		exit(0);
	}

	errx(1, "unrecognized command\n%s", commandline_usage);
}
