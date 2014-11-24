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

#define NEED_PRINT_MESSAGE
#include <can_driver.h>


#include <nuttx/config.h>
#include <drivers/device/device.h>
#include <drivers/drv_canopennode.h>
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


class CanOpenNode : public device::CDev
{
public:
	CanOpenNode();
	~CanOpenNode();

	int init();

	static int start();
	static int stop();
	static int task_main_trampoline(int argc, char *argv[]);

	void initNode(UNS8 nodeID);

	void run();
	void receiveLoop();

	int	ioctl(struct file *filp, int cmd, unsigned long arg);
	ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

private:
	volatile bool _task_should_exit;
	int		_task;
	int 	_can_dev_fd;
};

namespace
{
	CanOpenNode* g_Node = nullptr;
	hrt_call g_TimerCall;
}

CanOpenNode::CanOpenNode() :
	CDev("canopennode", CANOPEN_NODE_PATH),
	_task_should_exit(false),
	_task(-1),
	_can_dev_fd(-1)
{

}

CanOpenNode::~CanOpenNode()
{
}

extern "C" { __EXPORT int can_master_main(int argc, char *argv[]); }
extern "C" { __EXPORT int can_devinit(void); }


TIMEVAL getElapsedTime(void)
{
	return 0;
}

#define TRANSMIT 1
#define RECEIVE 1



sem_t sem;
void CanTimeDispatch(void *arg);
void CanTimeDispatch(void *arg)
{
	sem_post(&sem);
}

void setTimer(TIMEVAL value)
{
	hrt_cancel(&g_TimerCall);
	hrt_call_after(&g_TimerCall, value, &CanTimeDispatch, 0);
}

sem_t sendSem;
struct can_msg_s txmsg;
void* senderThread(void* arg);
void* senderThread(void* arg)
{
	int fd = *((int*)arg);
	sem_init(&sendSem, 0, 0);
	while(1)
	{
		sem_wait(&sendSem);
		ssize_t msgsize = CAN_MSGLEN(txmsg.cm_hdr.ch_dlc);
		ssize_t nbytes = msgsize;
		if(fd)
		{
			nbytes = write(fd, &txmsg, msgsize);
			if(msgsize == nbytes)
			{
				printf("CSD OK, %d %d 0x%08x\n", msgsize, nbytes, getreg32(STM32_CAN1_ESR));
				fflush(stdout);
			}
			else
			{
				printf("CSD F, %d %d %d\n", msgsize, nbytes, errno);
				fflush(stdout);
			}

		}
		else
		{
			printf("no fd\n");
		}
	}
	return 0;
}

void InitNode(UNS8 nodeID);

void* receiverThread(void* arg)
{
	CanOpenNode* canNode = reinterpret_cast<CanOpenNode*>(arg);
	if(canNode)
		canNode->receiveLoop();
}

void CanOpenNode::receiveLoop()
{
	struct can_msg_s rxmsg;
	while(!_task_should_exit)
	{
		size_t msgsize = CAN_MSGLEN(8);;
		ssize_t nbytes;
		nbytes = ::read(_can_dev_fd, &rxmsg, msgsize);
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
			print_message(&m);
			canDispatch(&F4BY_Data, &m);
		}
	}
}

void CanOpenNode::run()
{
	_can_dev_fd = ::open("/dev/can0", O_RDWR);
	pthread_t sender_thread_id;
	pthread_t receiver_thread_id;
	pthread_create(&sender_thread_id, 0, senderThread, &_can_dev_fd);
	pthread_create(&receiver_thread_id, 0, receiverThread, this);

	initNode(0x01);

	while(!_task_should_exit)
	{
		sem_wait(&sem);
#if TRANSMIT
		TimeDispatch();
#endif
	}
	::close(_can_dev_fd);
}

int CanOpenNode::task_main_trampoline(int argc, char *argv[])
{
	g_Node->run();
	return 0;
}

extern "C" {
	unsigned char canSend(CAN_PORT notused, Message *m)
	{
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
		print_message(m);
		//sem_post(&sendSem);
		int fddd = ::open("/dev/can0", O_RDWR);
		ssize_t msgsize = CAN_MSGLEN(txmsg.cm_hdr.ch_dlc);
		ssize_t nbytes = msgsize;
		if(fddd)
		{
			nbytes = write(fddd, &txmsg, msgsize);
			printf("WriteTO\n");
		}
		close(fddd);
		return 0;
	}
}


void CanOpenNode::initNode(UNS8 nodeID)
{
	setNodeId (&F4BY_Data, nodeID);
	InitCallbacks(&F4BY_Data);
	setState(&F4BY_Data, Initialisation);	// Init the state
}

static void status(void)
{
	/* display usage statistics */
}

static void usage(void)
{
	errx(1, "usage: can_master {start|stop|status}");
}

inline int CanOpenNode::start()
{
	int ret = OK;
	if (g_Node == nullptr) {
		g_Node = new CanOpenNode();

		if (g_Node == nullptr) {
			ret = -ENOMEM;
		} else {
			ret = g_Node->init();

			if (ret != OK) {
				delete g_Node;
				g_Node = nullptr;
			}
		}
	}
	return ret;
}

int CanOpenNode::stop()
{
	int ret = OK;
	if (g_Node != nullptr) {
		delete g_Node;
		g_Node = nullptr;
	}
	return ret;
}

int CanOpenNode::init()
{
	int ret = can_devinit();

	if(ret == OK){
		ret = CDev::init();
		if(ret == OK){
			sem_init(&sem, 0, 0);
			hrt_call_init(&g_TimerCall);

			_task = task_spawn_cmd("canopennode",
							   SCHED_DEFAULT,
							   SCHED_PRIORITY_DEFAULT,
							   2048,
							   (main_t)&CanOpenNode::task_main_trampoline,
							   nullptr);

			if (_task < 0) {
				debug("task start failed: %d", errno);
				return -errno;
			}
		}
	}

	return ret;
}

void test()
{
	g_Node->write(0, 0, 0);
}

int	CanOpenNode::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = -ENOTTY;
	switch(cmd)
	{
	case CANOPEN_ARM:
		printf("Oper\n");
		setState(&F4BY_Data, Operational);
		ret = OK;
		break;
	case CANOPEN_ARM_ESC:
		masterSendNMTstateChange(&F4BY_Data, 0, arg ? Operational : Pre_operational);
		break;
	default:
		break;
	}
	if(ret == -ENOTTY)
	{
		ret = CDev::ioctl(filp, cmd, arg);
	}
	return ret;
}

ssize_t	CanOpenNode::write(struct file *filp, const char *buffer, size_t buflen)
{
	const uint16_t* data = (const uint16_t*)buffer;
	for(int i = 0; i < 4; ++i)
	{
		speed[i] = (data[i] << 5) | (i+1);
	}
	sendOnePDOevent(&F4BY_Data, 0);
	return buflen;
}

int
can_master_main(int argc, char *argv[])
{
	if (argc < 2)
		usage();

	if (!strcmp(argv[1], "start")) {
		if (CanOpenNode::start() != OK)
			errx(1, "failed to start the FMU driver");
		else
			exit(0);
	}
	else if (!strcmp(argv[1], "stop"))
		CanOpenNode::stop();
	else if (!strcmp(argv[1], "status"))
		status();
	else if (!strcmp(argv[1], "test"))
		test();
	else
		usage();

	exit(1);
}


