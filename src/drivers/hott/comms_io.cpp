/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file comms_io.cpp
 * @author Simon Wilks <sjwilks@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 */

#include "comms_io.h"

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <systemlib/err.h>
#include <termios.h>
#include <poll.h>
#include <unistd.h>

#include "messages.h"

/* Oddly, ERROR is not defined for C++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;


int
send_poll(int uart, uint8_t *buffer, size_t size)
{
//	for (size_t i = 0; i < size; i++) {
//		write(uart, &buffer[i], sizeof(buffer[i]));
//
//		/* Sleep before sending the next byte. */
//		usleep(POST_WRITE_DELAY_IN_USECS);
//	}
//
//	/* A hack the reads out what was written so the next read from the receiver doesn't get it. */
//	/* TODO: Fix this!! */
//	uint8_t dummy[size];
//	read(uart, &dummy, size);

	return OK;
}

int
recv_data(int uart, uint8_t *buffer, size_t *size, uint8_t *id)
{
//	static const int timeout_ms = 1000;
//
//	struct pollfd fds;
//	fds.fd = uart;
//	fds.events = POLLIN;
//
//	// XXX should this poll be inside the while loop???
//	int ret = poll(&fds, 1, timeout_ms);
//	if (ret > 0) {
//		int i = 0;
//		bool stop_byte_read = false;
//		while (true)  {
//			read(uart, &buffer[i], sizeof(buffer[i]));
////			warnx("byte %u = %x", i + 1, buffer[i]);
//
//			if (stop_byte_read) {
//				// XXX process checksum
//				*size = ++i;
//				return OK;
//			}
//			// XXX can some other field not have the STOP BYTE value?
//			if (buffer[i] == STOP_BYTE) {
//				*id = buffer[1];
//				stop_byte_read = true;
//			}
//			i++;
//		}
//	}
////	else {
////		warnx("ret = %d", ret);
////	}
	return ERROR;
}
