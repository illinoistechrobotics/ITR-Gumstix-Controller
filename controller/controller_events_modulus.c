//
// controller_events.c
//
// This file contains all the user defined code for events triggered by
// a change of state. On the controller side, a change of state occurs
// when the joystick state changes. Or when the program starts and shuts
// down. See common/events.h for more complete information.
//

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "robot_comm.h"
#include "robot_log.h"
#include "joystick.h"
#include "events.h"
#include "profile.h"

void on_init() {
    robot_event ev;
    ev.command = ROBOT_EVENT_CMD_START;
    ev.index = 0;
    ev.value = 0;

	log_string(-1, "Controller is initializing");
	send_event(&ev);
}

void on_shutdown() {
    robot_event ev;
    ev.command = ROBOT_EVENT_CMD_STOP;
    ev.index = 0;
    ev.value = 0;

	log_string(-1, "Controller is shutting down");
	send_event(&ev);
}

void on_button_up(robot_event *ev) {
    printf("I PUSHED Button %i \n", ev->index);
}

void on_button_down(robot_event *ev) {
}

void on_axis_change(robot_event *ev) {
    if(ev->index == 0){
        robot_event new_ev;
        new_ev.command = ROBOT_EVENT_MOTOR;
        new_ev.index = 0; 
        new_ev.value = ev->value;
        send_event(&new_ev);
        printf("I MOVED Axis %i to %i \n",ev->index, ev->value);
    }
}

void on_1hz_timer(robot_event *ev) {

}

void on_10hz_timer(robot_event *ev) {
	 robot_event ev1;
	 ev1.command = ROBOT_EVENT_CMD_NOOP;
	 ev1.index = 0;
	 ev1.value = 0;
	 send_event(&ev1);
}


void on_status_code(robot_event *ev) {
	switch(ev->command) {
		case ROBOT_EVENT_NET_STATUS_OK:
			log_string(-1, "Controller received STATUS_OK:%02X\n", ev->value);
			break;

		case ROBOT_EVENT_NET_STATUS_ERR:
			log_string(-1, "Controller received STATUS_ERR:%02X\n", ev->value);
			break;

		case ROBOT_EVENT_NET_STATUS_NOTICE:
			log_string(-1, "Controller received STATUS_NOTICE:%02X\n", ev->value);
			break;
		default:
			// unknown status datagram
			log_string(-1, "Controller reveived STATUS_%02X:%02X\n", ev->command, ev->value);
			break;
	}

}


void on_adc_change(robot_event *ev){

}

void on_command_code(robot_event *ev) {
    robot_event send_ev;
	switch(ev->command) {
		case ROBOT_EVENT_CMD_NOOP:
            send_ev.command = ROBOT_EVENT_NET_STATUS_OK;
            send_ev.index = 0;
            send_ev.value = 0;

            send_event(&send_ev);
			break;

		case ROBOT_EVENT_CMD_START:
			log_string(-1, "Controller received CMD_START:%02X\n", ev->value);
			break;

		case ROBOT_EVENT_CMD_STOP:
			log_string(-1, "Controller received CMD_STOP:%02X\n", ev->value);
			break;

		case ROBOT_EVENT_CMD_REBOOT:
			
			log_string(-1, "Controller received CMD_REBOOT:%02X\n", ev->value);
			break;
		default:
			// unknown command code datagram
			log_string(-1, "Controller received CMD_%02X:%02X\n", ev->command, ev->value);
			break;
	}
}

void on_read_variable(robot_event* ev){
}

void on_set_variable(robot_event* ev){
}
