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

UNS32 speedChanged(CO_Data* d, const indextable *test , UNS8 bSubindex)
{
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

void GoToPreOperationalState(CO_Data* d)
{
	printf("PreOpState\n");
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
    
    esc_scanIndexOD (0x2000, &errorCode, &CallbackList);
	if(errorCode == OD_SUCCESSFUL && CallbackList)
	{
		CallbackList[0] = speedChanged;
	}
}
