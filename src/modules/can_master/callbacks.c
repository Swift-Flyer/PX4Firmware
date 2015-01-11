/*
 *  callbacks.c
 *  ESC
 *
 *  Created by Maxim G. Strinzha on 10/17/14.
 *  Copyright 2014 __MyCompanyName__. All rights reserved.
 *
 */
#include "canfestival.h"
#include "od.h"
#include "callbacks.h"
#include <stdio.h>


UNS32 speedChanged(CO_Data* d, const indextable *test , UNS8 bSubindex)
{
	printf("sc\n");
	fflush(stdout);

	return 0;
}

void GoToOperationalState(CO_Data* d)
{
	printf("OpState\n");
}

void GoToStoppedState(CO_Data* d)
{
	printf("StopState\n");
}

void CheckSDOAndContinue(CO_Data* d, UNS8 nodeId)
{
	printf("CaC\n");
}



void GoToPreOperationalState(CO_Data* d)
{
	printf("PreOpState\n");
	UNS16 Heartbeat_Producer_Time = 5000;
	writeNetworkDictCallBackAI (d, /*CO_Data* d*/
			/**TestSlave_Data.bDeviceNodeId, UNS8 nodeId*/
			0x11, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS8 subindex*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&Heartbeat_Producer_Time,/*void *data*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0,
			0); /* use block mode */
	//_preOperational(d);
}

void GoToInitialisationState(CO_Data* d)
{
	printf("InitState\n");
}

void InitCallbacks(CO_Data* d)
{
	d->operational = GoToOperationalState;
	d->stopped = GoToStoppedState;
	d->preOperational = GoToPreOperationalState;
	d->initialisation = GoToInitialisationState;
	
	
	
	UNS32 errorCode;
    ODCallback_t *CallbackList = 0;
    
    for(UNS16 index = 0x2001; index <= 0x2004; ++index)
    {
		F4BY_scanIndexOD (index, &errorCode, &CallbackList);
		if(errorCode == OD_SUCCESSFUL && CallbackList)
		{
			for(int i = 1; i <= 8; ++i)
			{
				CallbackList[i] = speedChanged;
			}
		}
	}
}
