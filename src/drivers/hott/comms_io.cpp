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
#include <termios.h>
#include <unistd.h>
#include <errno.h>


#include <drivers/drv_hrt.h>

#include "messages.h"

/* Oddly, ERROR is not defined for C++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

hrt_abstime hott_last_rx_time = 0;		/**< Timestamp when we last received */
hrt_abstime hott_last_tx_time = 0;		/**< Timestamp when we last sent data */
hrt_abstime hott_last_tx_byte_time = 0;	/**< Timestamp when we last sent a byte */
size_t poll_index = 0;

int
open_uart(const char *device)
{
	/* baud rate */
	static const speed_t speed = B19200;

	/* open uart */
	int uart = open(device, O_RDWR |  O_NOCTTY | O_NONBLOCK);

//	if (uart < 0) {
//		err(1, "Error opening port: %s", device);
//	}

	/* Back up the original uart configuration to restore it after exit */
	int termios_state;
	struct termios uart_config_original;
	if ((termios_state = tcgetattr(uart, &uart_config_original)) < 0) {
		close(uart);
//		err(1, "Error getting baudrate / termios config for %s: %d", device, termios_state);
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config;
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		close(uart);
//		err(1, "Error setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)",
//			 device, termios_state);
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		close(uart);
//		err(1, "Error setting baudrate / termios config for %s (tcsetattr)", device);
	}

	/* Activate single wire mode */
	if (ioctl(uart, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED) < 0) {
		int error_number = errno;
		uart = error_number;
		close(uart);
	}

	return uart;
}


int
send_poll(int uart, uint8_t *buffer, size_t size)
{

	hrt_abstime	now;
	now = hrt_absolute_time();

	if ( poll_index < size) {

		if (poll_index < size) {
			if (now - hott_last_tx_byte_time > (hrt_abstime)POST_WRITE_DELAY_IN_USECS) {
				int write_res = write(uart, &buffer[poll_index], sizeof(buffer[poll_index]));

				/* Sleep before sending the next byte. */
	//			usleep(POST_WRITE_DELAY_IN_USECS);
				hott_last_tx_byte_time = now;
				poll_index++;


				if ( poll_index >= size) {
					hott_last_tx_time = now;
					poll_index = 0;
					return OK;
				}

			}
		} else {
			hott_last_tx_time = now;
			poll_index = 0;
			return OK;
		}

	} else if (poll_index >= size && now > 5000){
		poll_index = 0;

	}

	return ERROR;

}

int
recv_data(int uart, uint8_t *buffer, size_t *size, unsigned *hott_partial_frame_count)
{
	if (hott_input(uart, buffer, size, hott_partial_frame_count) ) {
		return OK;
	}
	return ERROR;
}

bool hott_input(int uart, uint8_t *buffer, size_t *size, unsigned *hott_partial_frame_count)
{
	ssize_t		ret;
	hrt_abstime	now;

	now = hrt_absolute_time();

	if (*hott_partial_frame_count >= MAX_MESSAGE_BUFFER_SIZE) {
		*hott_partial_frame_count = 0;
	}

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * the current hott frame.
	 */
	ret = read(uart, &buffer[*hott_partial_frame_count], sizeof(struct vario_module_msg) - *hott_partial_frame_count);

	/* if the read failed for any reason, just give up here */
	if (ret < 1)
		return false;

	hott_last_rx_time = now;

	/*
	 * Add bytes to the current hott frame
	 */
	*hott_partial_frame_count += ret;

	/*
	 * If we don't have a full hott  frame, return
	 */
	if ( buffer[*hott_partial_frame_count - 1] != STOP_BYTE) //(hott_partial_frame_count < sizeof(struct vario_module_msg)) ||
		return false;
//	if ( *hott_partial_frame_count < sizeof(struct vario_module_msg))
//		return false;

	/*
	 * Great, it looks like we might have a hott frame.  Go ahead and
	 * decode it (done in hott_vario_tick).
	 */
	*hott_partial_frame_count = 0;
	return true;
}

int get_usec_since_poll() {

	return hrt_absolute_time() - hott_last_tx_time;
}
