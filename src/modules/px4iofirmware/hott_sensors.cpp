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
 * @author Thomas Gubler <thoamsgubler@gmail.com>
 * heavily based on work done by
 * @author Simon Wilks <sjwilks@gmail.com>
 *
 * Graupner HoTT sensor driver implementation for the PX4IO
 *
 * Poll any sensors connected to the PX4 via the telemetry wire. This can be used for altitude tracking independent of the FMU
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

#include <drivers/hott/comms.h>
#include <drivers/hott/messages.h>


extern "C" {
//#define DEBUG
#include "px4io.h"
}

#define DEFAULT_UART "/dev/ttyS0"		/**< USART1 */

static int uart;

void
hott_vario_init(void) {

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	uart = open_uart(DEFAULT_UART);

}

void
hott_vario_tick(void)
{
	static uint8_t buffer[MAX_MESSAGE_BUFFER_SIZE];
	static size_t size = 0;
	static uint8_t id = 0;

	/* Send poll request */
	build_vario_request(&buffer[0], &size);

	// The sensor will need a little time before it starts sending.
	usleep(5000);

	recv_data(uart, &buffer[0], &size, &id);

	/* Get altitude */
	struct vario_module_msg msg;
	size_t size_msg = sizeof(msg);
	memset(&msg, 0, size_msg);
	memcpy(&msg, buffer, size_msg);
	uint16_t altitude = (uint16_t) ((msg.alt_H << 8) | (msg.alt_L));

	/* Correct offset from hott protocol and save to register */
	r_page_hott[PX4IO_P_HOTT_VARIO_ALT] = SIGNED_TO_REG( (int16_t)altitude - 500 );

}
