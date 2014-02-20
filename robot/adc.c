//    joystick.c - reads the joystick and calls appropraite events
//    Copyright (C) 2007  Illinois Institute of Technology Robotics
//	  <robotics@iit.edu>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with this program; if not, write to the Free Software Foundation, Inc.,
//    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

// adc.c
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdio.h>
#include "robot_log.h"
#include "robot_queue.h"
#include "robot_comm.h"
#include "adc.h"
#include "events.h"
#include "timer.h"
#include "mod_i2c-io.h"

#ifndef NOADC
//---------------------------------------------------------------------------//
// Function Prototypes
//
static void *adc_thread_main(void *arg);

//---------------------------------------------------------------------------//
// Private Globals
//
static pthread_t tid = 0; // Thread ID of the adc thread

//---------------------------------------------------------------------------//
// Public Function Implementations
//

int adc_thread_create(robot_queue *q) {
	// create the thread
	if(pthread_create(&tid, NULL, adc_thread_main, q) != 0) {
		return 0;
	}
	return 1; // exit true

}

int adc_thread_destroy() {
	if (tid <= 0) {
		return 0;
	}

	// kill the thread
	if (pthread_cancel(tid) != 0) {
		return 0;
	}
	// reap the thread
	if (pthread_join(tid, NULL) != 0) {
		return 0;
	}
	return 1;

}

//---------------------------------------------------------------------------//
// Private Function Implementations
//

void *adc_thread_main(void *arg) {
	unsigned char ADCVals[8] = {127,127,127,127,127,127,127,127};
	int inval;
	robot_queue *q = (robot_queue *)arg;
	robot_event ev;
	int i;
	ev.command = ROBOT_EVENT_ADC;

	//Waits the polling interval, then polls all ADC's in use
	//Via ADC_COUNT (in adc.h) 
	while(1) {
		usleep(POLL_INTERVAL);	//Wait Polling Interval
		for(i = 0; i < ADC_COUNT; i++) {
			inval = getADC(i);
			inval = inval >> 2;
			if(inval != ADCVals[i]){
				ADCVals[i] = (unsigned char)inval;
				ev.index = i;
				ev.value = ADCVals[i];
				robot_queue_enqueue(q, &ev);
			}
		}
	}
	return NULL;
}
#endif
