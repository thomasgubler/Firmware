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
 * @file comms_io.h
 * @author Simon Wilks <sjwilks@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Communication with Graupner HoTT module, compatible with px4iofirmware
 *
 */


#ifndef COMMS_IO_H_
#define COMMS_IO_H_

#include <stdbool.h>
#include <unistd.h>

int open_uart(const char *device);

/**
 *  Receive data from the hott sensor, on the px4io this is a wrapper function for hott_input
 */
int recv_data(int uart, uint8_t *buffer, size_t *size, uint8_t *id, int16_t *debug);
int send_poll(int uart, uint8_t *buffer, size_t size, int16_t *debug);

/**
 *  Px4io specific read without using poll (inspired by the dsm and sbus input for px4iofirmware)
 */
bool hott_input(int uart, uint8_t *buffer, size_t *size, uint8_t *id, int16_t *debug);

int get_usec_since_poll();

#endif /* COMMS_IO_H_ */
