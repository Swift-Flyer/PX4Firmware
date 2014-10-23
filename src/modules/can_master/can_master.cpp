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
#include "canfestival.h"
#include "config.h"
#include "od.h"
#include "callbacks.h"
#include <drivers/drv_hrt.h>
#include "stm32_can.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32.h"

extern "C" { __EXPORT int can_master_main(int argc, char *argv[]); }
extern "C" { __EXPORT int can_devinit(void); }

//struct hrt_call		_call;
//unsigned		_call_interval;
//hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&BMA180::measure_trampoline, this);
//hrt_call_after();

TIMEVAL getElapsedTime(void)
{
	return 0;
}

#define TRANSMIT 1
#define RECEIVE 1

hrt_call call_;
hrt_call task_;

sem_t sem;

void CanTimeDispatch(void *arg)
{
	sem_post(&sem);
}

void setTimer(TIMEVAL value)
{
	hrt_cancel(&call_);
	hrt_call_after(&call_, value, &CanTimeDispatch, 0);
}



int task_receiver(int argc, char *argv[])
{
	struct can_msg_s rxmsg;
	int fd2 = open("/dev/can0", O_RDWR);
	while(1)
    {
#if RECEIVE
		
		size_t msgsize = CAN_MSGLEN(8);;
		ssize_t nbytes;
		nbytes = read(fd2, &rxmsg, msgsize);
		if (nbytes > 0)
		{
			Message m;
			m.cob_id = rxmsg.cm_hdr.ch_id;
			m.rtr = rxmsg.cm_hdr.ch_rtr;
			m.len = rxmsg.cm_hdr.ch_dlc;
			m.data[0] = rxmsg.cm_data[0];
			m.data[1] = rxmsg.cm_data[1];
			m.data[2] = rxmsg.cm_data[2];
			m.data[3] = rxmsg.cm_data[3];
			m.data[4] = rxmsg.cm_data[4];
			m.data[5] = rxmsg.cm_data[5];
			m.data[6] = rxmsg.cm_data[6];
			m.data[7] = rxmsg.cm_data[7];
			canDispatch(&esc_Data, &m);
			printf("%d r\n", int(hrt_absolute_time()/1000), msgsize, m.cob_id & 0x7F, rxmsg.cm_data[0], rxmsg.cm_data[1], rxmsg.cm_data[2] );
			
		}
		else
		{
			//usleep(1000);
		}
#endif		
	}
	close(fd2);
	return 0;
}

int fd1 = 0;

int task_main_trampoline(int argc, char *argv[])
{
	fd1 = open("/dev/can0", O_RDWR);
	while(1)
	{
		sem_wait(&sem);
#if TRANSMIT		
		TimeDispatch();
#endif		
	}
	close(fd1);
	return 0;
}

extern "C" {
	unsigned char canSend(CAN_PORT notused, Message *m)
	{
		struct can_msg_s txmsg;
		txmsg.cm_hdr.ch_id    = m->cob_id;
		txmsg.cm_hdr.ch_rtr   = m->rtr;
		txmsg.cm_hdr.ch_dlc   = m->len;
		txmsg.cm_data[0] = m->data[0];
		txmsg.cm_data[1] = m->data[1];
		txmsg.cm_data[2] = m->data[2];
		txmsg.cm_data[3] = m->data[3];
		txmsg.cm_data[4] = m->data[4];
		txmsg.cm_data[5] = m->data[5];
		txmsg.cm_data[6] = m->data[6];
		txmsg.cm_data[7] = m->data[7];

		size_t msgsize = CAN_MSGLEN(txmsg.cm_hdr.ch_dlc);
		ssize_t nbytes;
		if(fd1)
		{
			//printf("BW\n");
    		nbytes = write(fd1, &txmsg, msgsize);  
    		//printf("AW\n");
			if(msgsize == nbytes)
			{
				printf("CSD OK, %d %d 0x%08x\n", msgsize, nbytes, getreg32(STM32_CAN1_ESR));
				fflush(stdout);
			}
			else
			{
				printf("CSD F, %d %d %d\n", msgsize, nbytes, errno);
			}
    	}
    	else
    	{
    		printf("no fd\n");
    	}
		return msgsize == nbytes;
	}
}

void InitNode(int a)
{
	UNS8 nodeID = a;
	setNodeId (&esc_Data, nodeID);
	InitCallbacks(&esc_Data);
	setState(&esc_Data, Initialisation);	// Init the state
}

static int
start(int a)
{
	sem_init(&sem, 0, 0);
	hrt_call_init(&call_);
	can_devinit();
	//fd = open("/dev/can0", O_RDWR);
	//size_t msgsize;
  	//ssize_t nbytes;
	//msgsize = CAN_MSGLEN(2);
    //nbytes = write(fd, &txmsg, msgsize);    
	
	//msgsize = sizeof(struct can_msg_s);
    
    task_create("task_receiver", 100, 2048, task_receiver, 0);
    task_create("task", 100, 2048, task_main_trampoline, 0);
    //hrt_call_every(&task_, 0, 1000000, &task, 0);
    
    InitNode(a);
    setState(&esc_Data, Operational);
    
    
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
can_master_main(int argc, char *argv[])
{
	if (argc < 2)
		usage();

	if (!strcmp(argv[1], "start")) {

		/*if (g_fd >= 0)
			errx(1, "already running");*/
		int a = atoi(argv[2]);
		start(a);

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

