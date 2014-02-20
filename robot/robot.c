//    robot.c - receives commands, and moves motors.
//    Copyright (C) 2007 Illinois Institute of Technology Robotics
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

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "robot_comm.h"
#include "robot_log.h"
#include "events.h"
#include "timer.h"
#include "mod_i2c-io.h"
#include "robot_queue.h"
#include "profile.c"
#include "adc.h"

void term_handler(int signal);

// usage information
void usage(char *progname);

void failsafe_mode(robot_queue *q);

const unsigned char dvmaxlut[] = {
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 62, 62, 62, 62, 62, 61, 61, 61,
    61, 60, 60, 60, 59, 59, 58, 58, 58, 57,
    56, 56, 55, 55, 54, 53, 53, 52, 51, 50,
    49, 48, 47, 47, 46, 44, 43, 42, 41, 40,
    39, 38, 36, 35, 34, 33, 31, 30, 29, 27,
    26, 25, 23, 22, 21, 20, 18, 17, 16, 15,
    14, 13, 12, 11, 10,  9,  8,  7,  7,  6,
     5,  5,  5,  4,  4,  4,  4,  4,  4,  4,
     4,  4,  5,  5,  5,  6,  7,  7,  8,  9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 20,
    21, 22, 23, 25, 26, 27, 29, 30, 31, 33,
    34, 35, 36, 38, 39, 40, 41, 42, 43, 44,
    46, 47, 47, 48, 49, 50, 51, 52, 53, 53,
    54, 55, 55, 56, 56, 57, 58, 58, 58, 59,
    59, 60, 60, 60, 61, 61, 61, 61, 62, 62,
    62, 62, 62, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64
};

int main(int argc, char *argv[])
{
	unsigned int server_port;
	robot_queue q;
    robot_event ev;
	unsigned int failcount;
	int opt;

	log_level = 0;
	setProfile('p');
	server_port = 0;
	 while((opt = getopt(argc, argv, "p:v:j:")) != -1)
		 switch (opt)
		 {
			 case 'p':
				 server_port = atoi(optarg);
			 	 break;
			 case 'v':
				 log_level = atoi(optarg);
			 	 break;
			 case 'j':
				 setProfile(optarg[0]);
			 case '?':
				 usage(argv[0]);
				 exit(1);
				 break;
				
		 }

	 if(server_port == 0){
		 server_port = 31337;
	 }
	log_string(-10, "Creating Queue");
	robot_queue_create(&q);

	// Initialize the I2C and servos
	log_string(-10, "Init");
	init(0x0b);
	log_string(-10, "servoInit");
	servoInit();

	// Open up the socket
	if(!net_thread_server_create(&q, server_port)) {
		log_string(2, "Error running the network thread");
		exit(1);
	}

	if(!timer_thread_create(&q)){
		log_string(2, "Error running the timer thread");
		exit(1);
	}
#ifndef NO_ADC
	if(!adc_thread_create(&q)){
		log_string(2, "Error running the adc thread");
		exit(1);
	}
#endif


	// init event
	on_init();

	// Install the signal handlers
	if(signal(SIGHUP, term_handler) == SIG_ERR)
		log_errno(0, "Error setting the SIGHUP handler");
	if(signal(SIGINT, term_handler) == SIG_ERR)
		log_errno(0, "Error setting the SIGINT handler");
	if(signal(SIGTERM, term_handler) == SIG_ERR)
		log_errno(0, "Error setting the SIGTERM handler");


	while(1) {
		robot_queue_wait_event(&q, &ev);
		switch (ev.command & 0xF0) {
			case ROBOT_EVENT_CMD:
				failcount = 0;
				on_command_code(&ev);
				break;
			case ROBOT_EVENT_NET:
				failcount = 0;
				on_status_code(&ev);
				break;
			case ROBOT_EVENT_JOY_AXIS:
				failcount = 0;
				on_axis_change(&ev);
				break;
			case ROBOT_EVENT_MOTOR:
				failcount = 0;

		/*
                // update the state
                if (ev.index < 4) {
                    last_motors[ev.index] = ev.value;
                    vf = last_motors + ev.index;
                    *vf = ev.value;
                    vi = cur_motors + ev.index;
                    dv = (int)*vf - (int)*vi; 
                    // if the motor tries to accelerate too fast limit it
                    if ((dv > dvmaxlut[*vi]) || (-dv > dvmaxlut[*vi])) {
                        log_string(0, "Over accelerated");
                        dv = dvmaxlut[*vi] * ( dv > 0 ? 1 : -1 );
                        log_string(0, "vi= %d, vf=%d, dv=%d", *vi, *vf, dv);
                    }
                    *vi += dv;
                    ev.value = *vi;

                }
		*/
				on_motor(&ev);

				break;
			case ROBOT_EVENT_JOY_BUTTON:
				failcount = 0;
				if(ev.value)	
					on_button_down(&ev);
				else
					on_button_up(&ev);
				break;
			case ROBOT_EVENT_ADC:
				failcount = 0;
				on_adc_change(&ev);
				break;
			case ROBOT_EVENT_SET_VAR:
				failcount = 0;
				on_set_variable(&ev);
				log_string(-1,"Set var: %d to %d. robot.c", ev.index, ev.value);
				break;
			case ROBOT_EVENT_READ_VAR:
				failcount = 0;
				on_read_variable(&ev);
				break;
			case ROBOT_EVENT_TIMER:
				if(ev.index == 1){
					on_10hz_timer(&ev);
					failcount++;
					if(failcount >= 5)
						failsafe_mode(&q);

		 /*
                    // resubmit the motor commands
                    new_ev.command = ROBOT_EVENT_MOTOR;
                    for(i = 0; i < 4; ++i) {
                        new_ev.index = i;
                        new_ev.value = last_motors[i];
                        robot_queue_enqueue(&q, &new_ev);
                    }
		    */

				}
                else if(ev.index == 2)
					on_1hz_timer(&ev);
		}
	}

	on_shutdown();
	net_thread_destroy();

	return 0;
}

// handles signals that tell the controller to shutdown
void term_handler(int signal) { // Signal handler
	on_shutdown();

	net_thread_destroy();

	exit(0);
}


void usage(char *progname) {
	log_string(3, "%s: [-p port (31337)] [-v verbosity (0)]\n", progname);
}

void failsafe_mode(robot_queue *q) {
	robot_event ev;
	ev.command = ROBOT_EVENT_SET_VAR;
	ev.index = 12;
	ev.value = 0;
	robot_queue_enqueue(q, &ev);

	ev.command = ROBOT_EVENT_MOTOR;
	ev.index = 0;
	ev.value = 127;
	int i;
	for(i = 0; i<=5; i++){
		ev.index = i;
		robot_queue_enqueue(q, &ev);
	}
}



