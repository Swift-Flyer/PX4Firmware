/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file vrinput.cpp
 * Driver for the VRBRAIN board.
 *
 *
 */
#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <crc32.h>
#include <arch/board/board.h>
#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
//#include <drivers/drv_sbus.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
//#include <drivers/vrbrain/vrinput/controls/controls.h>
#include <uORB/topics/rc_channels.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_rc.h>
#include <systemlib/ppm_decode.h>

#ifndef ARDUPILOT_BUILD
# define RC_HANDLING_DEFAULT false
#else
// APM uses raw PWM output
# define RC_HANDLING_DEFAULT true
#endif
/**
 * The VRBRAIN class.
 *
 *
 */
class F4BY_INPUT : public device::CDev
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	F4BY_INPUT();
	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	virtual ~F4BY_INPUT();
	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 */
	virtual int		init();
	/**
	 * Push failsafe values to IO.
	 *
	 * @param[in] vals	Failsafe control inputs: in us PPM (900 for zero, 1500 for centered, 2100 for full)
	 * @param[in] len	Number of channels, could up to 8
	 */
	int			set_failsafe_values(const uint16_t *vals, unsigned len);
	/**
	 * Disable RC input handling
	 */
private:
	// XXX
	enum InputType
	{
		ePPM = 1,
		ePPMSUM,
		eSBUS,
		eDSM
	};

	unsigned		_max_rc_input;		///< Maximum receiver channels supported by PX4IO

	unsigned		_rc_chan_count;		///< Internal copy of the last seen number of RC channels
	uint64_t		_rc_last_valid;		///< last valid timestamp
	volatile int		_task;			///< worker task id
	volatile bool		_task_should_exit;	///< worker terminate flag
	perf_counter_t		_perf_chan_count;	///<local performance counter for channel number changes
	/* advertised topics */
	orb_advert_t 		_to_input_rc;		///< rc inputs from io
	/**
	 * Trampoline to the worker task
	 */
	static void		task_main_trampoline(int argc, char *argv[]);
	/**
	 * worker task
	 */
	void			task_main();

	void		controls_tick();
	void		controls_init();
	/**
	 * Fetch RC inputs from IO.
	 *
	 * @param input_rc	Input structure to populate.
	 * @return		OK if data was returned.
	 */
	int			io_get_raw_rc_input(rc_input_values &input_rc);
	/**
	 * Fetch and publish raw RC input data.
	 */
	int			io_publish_raw_rc();
};
namespace
{
F4BY_INPUT	*g_dev = nullptr;
}
F4BY_INPUT::F4BY_INPUT() :
	CDev("f4by_input", F4BY_INPUT_DEVICE_PATH),
	_max_rc_input(0),
	_rc_chan_count(0),
	_rc_last_valid(0),
	_task(-1),
	_task_should_exit(false),
	_perf_chan_count(perf_alloc(PC_COUNT, "io rc #")),
	_to_input_rc(0)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	_debug_enabled = false;
}
F4BY_INPUT::~F4BY_INPUT()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;
	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}
	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);
	g_dev = nullptr;
}

#if 0
uint8_t RC_Init()
{
 GPIO_InitTypeDef GPIO_InitStructure;
 
 uint8_t i;
 uint8_t test = 0x5a; 
 uint8_t sbus = 0;
 uint8_t sppm = 0; 
 
 /* ïîäàäèì òàêò íà ïîðò */
  RCC_APB1PeriphClockCmd(RC_SelectGPIO_RCC, ENABLE);
  /* íàñòðîéêà âõîäîâ ïîðòà */
  GPIO_InitStructure.GPIO_Pin  = RC_SelectGPIO_SBUSPin|RC_SelectGPIO_PPMPin;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(RC_SelectGPIO, &GPIO_InitStructure);
 /* âûõîä */
 GPIO_InitStructure.GPIO_Pin  = RC_SelectGPIO_OutPin; 
 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;  
 GPIO_Init(RC_SelectGPIO, &GPIO_InitStructure);
 
 for(i = 0;i<32;i++)
 {  
  if ((test>>(i&0x07))&0x01) {
   GPIO_SetBits(RC_SelectGPIO,RC_SelectGPIO_OutPin);
   while (!GPIO_ReadOutputDataBit(RC_SelectGPIO,RC_SelectGPIO_OutPin));
   if (!GPIO_ReadInputDataBit(RC_SelectGPIO,RC_SelectGPIO_SBUSPin)) sbus++;  
   if (!GPIO_ReadInputDataBit(RC_SelectGPIO,RC_SelectGPIO_PPMPin)) sppm++;
   
  } else {
   GPIO_ResetBits(RC_SelectGPIO,RC_SelectGPIO_OutPin);
   while (GPIO_ReadOutputDataBit(RC_SelectGPIO,RC_SelectGPIO_OutPin));
   if (GPIO_ReadInputDataBit(RC_SelectGPIO,RC_SelectGPIO_SBUSPin)) sbus++;  
   if (GPIO_ReadInputDataBit(RC_SelectGPIO,RC_SelectGPIO_PPMPin)) sppm++;
  }
 }

#ifdef _F4BY_V2_
 if (sbus==0) {
  USART_SBUS_Config();
  RCSource = RC_SBUS;
  return RCSource;
 }
#endif /* _F4BY_V2_*/  
 
 if (sppm==0) {
  RC_SPPM_Config();
  RCSource =  RC_SPPM;
  return RCSource;
 }
 
 #endif
 
 Init_PPM(1000000,1000000);
 RCSource =  RC_PWM;
 return RCSource;
}

int
F4BY_INPUT::init()
{

	int ret;
	ASSERT(_task == -1);
	/* do regular cdev init */
	ret = CDev::init();
	if (ret != OK)
		return ret;

	/* start the IO interface task */
	_task = task_create("f4by_input", SCHED_PRIORITY_ACTUATOR_OUTPUTS, 2048, (main_t)&F4BY_INPUT::task_main_trampoline, nullptr);
	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}
void
F4BY_INPUT::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}
void
F4BY_INPUT::task_main()
{
	log("controls_init");
	controls_init();
	/* lock against the ioctl handler */
	lock();
	/* loop talking to IO */
	while (!_task_should_exit) {
		controls_tick();
		usleep(1000);
		/* get raw R/C input from IO */
		io_publish_raw_rc();
	}
	unlock();
out:
	debug("exiting");
	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

extern "C" int    dsm_init(const char *device);
extern "C" bool   dsm_input(uint16_t *values, uint16_t *num_values);

extern "C" int    sbus_init(const char *device);
extern "C" bool   sbus_input(uint16_t *values, uint16_t *num_values, uint16_t *rssi, uint16_t max_channels);

static bool
ppm_input(uint16_t *values, uint16_t *num_values)
{
	bool result = false;

	/* avoid racing with PPM updates */
	irqstate_t state = irqsave();

	/*
	 * If we have received a new PPM frame within the last 200ms, accept it
	 * and then invalidate it.
	 */
	if (hrt_elapsed_time(&ppm_last_valid_decode) < 200000) {

		/* PPM data exists, copy it */
		*num_values = ppm_decoded_channels;
		if (*num_values > 16)
			*num_values = 16;

		for (unsigned i = 0; i < *num_values; i++)
			values[i] = ppm_buffer[i];

		/* clear validity */
		ppm_last_valid_decode = 0;

		/* good if we got any channels */
		result = (*num_values > 0);
	}

	irqrestore(state);

	return result;
}

void F4BY_INPUT::controls_tick()
{
	uint16_t r_raw_rc_values[18] = {0};
	uint16_t r_raw_rc_count = 18;
	uint16_t rssi;
	
	struct rc_input_values rc_in;

	memset(&rc_in, 0, sizeof(rc_in));
	rc_in.input_source = RC_INPUT_SOURCE_PX4FMU_PPM;
	
	//perf_begin(c_gather_dsm);
	uint16_t temp_count = r_raw_rc_count;
	bool dsm_updated = dsm_input(r_raw_rc_values, &temp_count);
	if (dsm_updated) {

		r_raw_rc_count = temp_count & 0x7fff;
		rc_in.channel_count = r_raw_rc_count;
		if (rc_in.channel_count > RC_INPUT_MAX_CHANNELS) {
			rc_in.channel_count = RC_INPUT_MAX_CHANNELS;
		}

		for (uint8_t i = 0; i < rc_in.channel_count; i++) {
			rc_in.values[i] = r_raw_rc_values[i];
		}

		rc_in.timestamp = hrt_absolute_time();
		
		rssi = 255;
	}
	//perf_end(c_gather_dsm);
	
	
	
	
	//perf_begin(c_gather_sbus);
	bool sbus_updated = sbus_input(r_raw_rc_values, &r_raw_rc_count, &rssi, 18 /* XXX this should be INPUT channels, once untangled */);
	if(sbus_updated)
	{
		rc_in.channel_count = r_raw_rc_count;
		if (rc_in.channel_count > RC_INPUT_MAX_CHANNELS) {
			rc_in.channel_count = RC_INPUT_MAX_CHANNELS;
		}

		for (uint8_t i = 0; i < rc_in.channel_count; i++) {
			rc_in.values[i] = r_raw_rc_values[i];
		}

		rc_in.timestamp = hrt_absolute_time();
	}
	//perf_end(c_gather_sbus);
	
	
	//perf_begin(c_gather_ppm);
	bool ppm_updated = ppm_input(r_raw_rc_values, &r_raw_rc_count);
	ppm_updated = 1;
	if (ppm_updated) {

		/* XXX sample RSSI properly here */
		rssi = 255;

		rc_in.channel_count = r_raw_rc_count;
		if (rc_in.channel_count > RC_INPUT_MAX_CHANNELS) {
			rc_in.channel_count = RC_INPUT_MAX_CHANNELS;
		}

		for (uint8_t i = 0; i < rc_in.channel_count; i++) {
			rc_in.values[i] = r_raw_rc_values[i];
			rc_in.values[i] = 1400;
		}

		rc_in.timestamp = hrt_absolute_time();
	}
	//perf_end(c_gather_ppm);
	
	if(sbus_updated || dsm_updated || ppm_updated)
	{
		/* lazily advertise on first publication */
		if (_to_input_rc == 0) {
			_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_in);

		} else {
			orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_in);
		}
	}
}

void F4BY_INPUT::controls_init()
{
	InputType inputType = ePPM;
	
	if(inputType == eDSM)
	{
		/* DSM input (USART6) */
		dsm_init("/dev/ttyS4");
	}
	else
	{
		unregister_driver("/dev/ttyS4");
		if(inputType == eSBUS)
		{
			/* S.bus input (USART4) */
			sbus_init("/dev/ttyS3");
		}
		else if(inputType == ePPMSUM)
		{
			rc_init();
		}
		else if(inputType == ePPM)
		{
			rc_init();
		}
		else
		{
			//error. unknown input source
		}
	}
	

	
}

int
F4BY_INPUT::io_get_raw_rc_input(rc_input_values &input_rc)
{
	uint32_t channel_count;
	int	ret = OK;
#if 0	
	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = RC_INPUT_SOURCE_UNKNOWN;
	static const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t regs[RC_INPUT_MAX_CHANNELS + prolog];
	/*
	 * Read the channel count and the first 9 channels.
	 *
	 * This should be the common case (9 channel R/C control being a reasonable upper bound).
	 */
	input_rc.timestamp_publication = hrt_absolute_time();
//	ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &regs[0], prolog + 9);
	for (unsigned i = 0; i < prolog + 9; i++) {
		regs[i] = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT + i);
//		printf("[F4BY_INPUT] %u - %u\n", i, regs[i]);
	}
//	if (ret != OK)
//		return ret;
	/*
	 * Get the channel count any any extra channels. This is no more expensive than reading the
	 * channel count once.
	 */
	channel_count = regs[PX4IO_P_RAW_RC_COUNT];
	if (channel_count != _rc_chan_count)
		perf_count(_perf_chan_count);
	_rc_chan_count = channel_count;
	input_rc.rc_ppm_frame_length = regs[PX4IO_P_RAW_RC_DATA];
	input_rc.rssi = regs[PX4IO_P_RAW_RC_NRSSI];
	input_rc.rc_failsafe = (regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost_frame_count = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
	input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];
	/* rc_lost has to be set before the call to this function */
	if (!input_rc.rc_lost && !input_rc.rc_failsafe)
		_rc_last_valid = input_rc.timestamp_publication;
	input_rc.timestamp_last_signal = _rc_last_valid;
	if (channel_count > 9) {
//		ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + 9, &regs[prolog + 9], channel_count - 9);
		for (unsigned int i = 0; i < channel_count - 9; i++) {
			regs[prolog + 9 + i] = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + 9 + i);
		}
//		if (ret != OK)
//			return ret;
	}
	input_rc.channel_count = channel_count;
	memcpy(input_rc.values, &regs[prolog], channel_count * 2);
#endif	
	return ret;
}
int
F4BY_INPUT::io_publish_raw_rc()
{
#if 0
	/* fetch values from IO */
	rc_input_values	rc_val;
	/* set the RC status flag ORDER MATTERS! */
	rc_val.rc_lost = !(_status & PX4IO_P_STATUS_FLAGS_RC_OK);
	int ret = io_get_raw_rc_input(rc_val);
	if (ret != OK)
		return ret;
	/* sort out the source of the values */
	if (_status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_PPM;
	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_SPEKTRUM;
	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_SBUS;
	} else {
		rc_val.input_source = RC_INPUT_SOURCE_UNKNOWN;
		/* we do not know the RC input, only publish if RC OK flag is set */
		/* if no raw RC, just don't publish */
		if (!(_status & PX4IO_P_STATUS_FLAGS_RC_OK))
			return OK;
	}
	/* lazily advertise on first publication */
	if (_to_input_rc == 0) {
		_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_val);
	} else {
		orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_val);
	}
#endif	
	return OK;
}
extern "C" __EXPORT int f4by_input_main(int argc, char *argv[]);
namespace
{
void
start(int argc, char *argv[])
{
	if (g_dev != nullptr)
		errx(0, "already loaded");
	/* create the driver - it will set g_dev */
	(void)new F4BY_INPUT();
	if (g_dev == nullptr) {
		errx(1, "driver alloc failed");
	}
	if (OK != g_dev->init()) {
		delete g_dev;
		g_dev = nullptr;
		errx(1, "driver init failed");
	}
	exit(0);
}
} /* namespace */

int
f4by_input_main(int argc, char *argv[])
{
	/* check for sufficient number of arguments */
	if (argc < 2)
		goto out;
	if (!strcmp(argv[1], "start"))
		start(argc - 1, argv + 1);
	/* commands below here require a started driver */
	if (g_dev == nullptr)
		errx(1, "not started");
	if (!strcmp(argv[1], "stop")) {
		/* stop the driver */
		delete g_dev;
		g_dev = nullptr;
		exit(0);
	}

out:
	errx(1, "need a command, try 'start', 'stop'");
}
