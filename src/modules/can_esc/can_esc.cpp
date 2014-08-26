/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier
 *              Jean Cyr
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
 * @file dataman.c
 * DATAMANAGER driver.
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
#include <sys/ioctl.h>
#include <systemlib/systemlib.h>
#include <systemlib/visibility.h>
#include <systemlib/err.h>
#include <nuttx/can.h>
#include <queue.h>

extern "C" { __EXPORT int can_esc_main(int argc, char *argv[]); }
extern "C" { __EXPORT int can_devinit(void); }

static int
start(void)
{
	can_devinit();
	
	int fd = open("/dev/can0", O_RDWR);
	struct can_msg_s txmsg;
	struct can_msg_s rxmsg;
	txmsg.cm_hdr.ch_id    = 1;
    txmsg.cm_hdr.ch_rtr   = false;
    txmsg.cm_hdr.ch_dlc   = 2;
    txmsg.cm_data[0] = 100;
    txmsg.cm_data[1] = 101;

	size_t msgsize;
  	ssize_t nbytes;
	msgsize = CAN_MSGLEN(2);
    nbytes = write(fd, &txmsg, msgsize);    
	
	msgsize = sizeof(struct can_msg_s);
    nbytes = read(fd, &rxmsg, msgsize);
    if (nbytes < CAN_MSGLEN(0) || nbytes > msgsize)
      {
        printf("ERROR: read(%d) returned %d\n", msgsize, nbytes);
       }
       else
       {
       	 printf("OK: read(%d) returned %d %d %d\n", msgsize, nbytes, rxmsg.cm_data[0], rxmsg.cm_data[1]);
       }
	
	return 0;
}

static void
status(void)
{
	/* display usage statistics */
}

static void
stop(void)
{
	/* Tell the worker task to shut down */
	
}

static void
usage(void)
{
	errx(1, "usage: can_esc {start|stop|status}");
}

int
can_esc_main(int argc, char *argv[])
{
	if (argc < 2)
		usage();

	if (!strcmp(argv[1], "start")) {

		/*if (g_fd >= 0)
			errx(1, "already running");*/

		start();

		/*if (g_fd < 0)
			errx(1, "start failed");*/

		exit(0);
	}

	/* Worker thread should be running for all other commands */
	/*if (g_fd < 0)
		errx(1, "not running");*/

	if (!strcmp(argv[1], "stop"))
		stop();
	else if (!strcmp(argv[1], "status"))
		status();
	else
		usage();

	exit(1);
}

