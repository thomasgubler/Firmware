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
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include <drivers/hott/comms_io.h>
#include <drivers/hott/messages.h>
#include <drivers/drv_hrt.h>

extern "C" {
//#define DEBUG
#include "px4io.h"
}

#define DEFAULT_UART "/dev/ttyS0"		/**< USART1*/

static int uart;

enum {
	HOTT_STATE_RECEIVED,
	HOTT_STATE_POLLED,
	HOTT_STATE_POLLED_CONFIRMED
} hott_state;

void hott_vario_init(void)
{

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	uart = open_uart(DEFAULT_UART);

}

void hott_vario_tick(void)
{
//	static uint8_t buffer[MAX_MESSAGE_BUFFER_SIZE];
//	static size_t size = 0;
//	static uint8_t id = 0;
//	static int16_t debug_output_receive = 99;
//	static int16_t debug_output_poll = 99;
//	static int16_t debug_output_poll_read = 99;
//	static hrt_abstime poll_time;
//
//	if (hott_state == HOTT_STATE_RECEIVED) {
//		/* Send poll request */
//		build_vario_request(&buffer[0], &size);
//
//		if (OK == send_poll(uart, buffer, size, &debug_output_poll)) {
//			hott_state = HOTT_STATE_POLLED;
//			poll_time = hrt_absolute_time();
//		}
//
//	}
//	if (hott_state == HOTT_STATE_POLLED) {
//		hrt_abstime now = hrt_absolute_time();
//
//		/* A hack the reads out what was written so the next read from the receiver doesn't get it. */
//		/* TODO: Fix this!! */
//		uint8_t dummy[size];
//		int res_read = read(uart, &dummy, size);
//		if (res_read > 0) {
//			hott_state = HOTT_STATE_POLLED_CONFIRMED;
//		} else {
//			debug_output_poll_read = errno;
//
//
//		}
//
//		if (now - poll_time > 2e6) {
//			/* Timeout: reset state machine and poll again in next iteration */
//			hott_state = HOTT_STATE_RECEIVED;
//		}
//
//	}
//	if (hott_state == HOTT_STATE_POLLED_CONFIRMED) {
//		hrt_abstime now = hrt_absolute_time();
//		if (now - poll_time > 5000) {  //The sensor will need a little time before it starts sending.
//			if (OK == recv_data(uart, &buffer[0], &size, &id, &debug_output_receive)) {
//
//				/* Possibly received a valid frame, decode */
//
//
//				/* Get altitude */
//				struct vario_module_msg msg;
//				size_t size_msg = sizeof(msg);
//				memset(&msg, 0, size_msg);
//				memcpy(&msg, buffer, size_msg);
//				uint16_t altitude = (uint16_t) ((msg.alt_H << 8) | (msg.alt_L));
//
//				/* Correct offset from hott protocol and save to register */
//				r_page_hott[PX4IO_P_HOTT_VARIO_ALT] = SIGNED_TO_REG( (int16_t)altitude );
//		//		r_page_hott[PX4IO_P_HOTT_VARIO_ALT] = SIGNED_TO_REG(
//		//				(int16_t )id);
//				hott_state = HOTT_STATE_RECEIVED;
//			} else {
//				r_page_hott[PX4IO_P_HOTT_VARIO_ALT] = SIGNED_TO_REG((int16_t)debug_output_poll);
//			}
//		} else if (now - poll_time > 2e6) {
//			/* Timeout: reset state machine and poll again in next iteration */
//			hott_state = HOTT_STATE_RECEIVED;
//		}
//	}


	uint8_t buffer_out = 1;
	uint8_t buffer_in = 0;
	int write_res = write(uart, &buffer_out, 1);
	int read_res = read(uart, &buffer_in, 1);
	write_res = write(uart, &write_res, sizeof(write_res));
	write_res = write(uart, &read_res, sizeof(read_res));


}
