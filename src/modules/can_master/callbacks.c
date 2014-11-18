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
