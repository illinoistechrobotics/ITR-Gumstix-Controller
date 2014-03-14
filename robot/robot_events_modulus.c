//    robot_events.c - User implementation of robot events
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

// robot_events.c
//
// This file contains all the user defined code for events triggered by
// a change of state. On the robot side, a change of state occurs
// when a datagram is received that communicates an event on the
// controller's side.
//
// Local events are triggered on initialization, and shutdown
//
// See common/events.h for complete information on events.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "robot_comm.h"
#include "robot_log.h"
#include "events.h"
#include "mod_i2c-io.h"
#include "profile.h"
#include "libjaguar.h"

int flasher = 0;
JaguarConnection conn;
int rotating = 0;

void on_init() {
	robot_event ev;
	ev.command = ROBOT_EVENT_CMD_START;
	ev.index = 0;
	ev.value = 0;

    open_jaguar_connection(&conn, "/dev/ttyS2");

    uint8_t m;
    for (m = 1; m < 5; m++) {
        config_encoder_lines(&conn, m, 720);
        position_ref_encoder(&conn, m);
        position_pid(&conn, m, 0x4e200000, 0x00000000, 0x00000000);
        position_enable(&conn, m, 0);
    }

	log_string(-1, "Robot is initializing");
	send_event(&ev);
}

void on_shutdown() {
	robot_event ev;
	ev.command = ROBOT_EVENT_CMD_STOP;
	ev.index = 0;
	ev.value = 0;

	log_string(-1, "Robot is shutting down");
	send_event(&ev);
}

void on_button_up(robot_event *ev) {
    if(ev->index == 0) setPin(2,0,0);
    if(ev->index == 1) setPin(2,1,0);
    if(ev->index == 2) setPin(2,2,0);
    if(ev->index == 3) setPin(2,3,0);
    if(ev->index == 4) setPin(2,4,0);
    if(ev->index == 5) setPin(2,5,0);
}

void on_button_down(robot_event *ev) {	
    if(ev->index == 0) setPin(2,0,1);
    if(ev->index == 1) setPin(2,1,1);
    if(ev->index == 2) setPin(2,2,1);
    if(ev->index == 3) setPin(2,3,1);
    if(ev->index == 4) setPin(2,4,1);
    if(ev->index == 5) setPin(2,5,1);
}

void on_axis_change(robot_event *ev){
}

void on_adc_change(robot_event *ev){
	send_event(ev);
}

void on_motor(robot_event *ev) {
	if(ev->index == 1) {
        setMotor(0, ev->value);
        setMotor(1, ev->value);
        setMotor(2, ev->value);
        setMotor(3, ev->value);
    }
	if(ev->index == 3) {
        int32_t pos;
        //position_get(&conn, 1, &pos);

        if (ev->value < 255){
            pos = ((int32_t) (ev->value - 127)) * 0x00000100;
        } else {
            pos = ((int32_t) 127) * 0x00000100;
        }

        //position_set(&conn, 1, pos);
        //position_set(&conn, 2, pos);
        //position_set(&conn, 3, pos);
        //position_set(&conn, 4, pos);
        position_set_sync(&conn, 1, pos, 1);
        position_set_sync(&conn, 2, pos, 1);
        position_set_sync(&conn, 3, pos, 1);
        position_set_sync(&conn, 4, pos, 1);
        sys_sync_update(&conn, 1);
    }
}

void on_status_code(robot_event *ev) {
	switch(ev->command) {
		case ROBOT_EVENT_NET_STATUS_OK:
			break;
		case ROBOT_EVENT_NET_STATUS_ERR:
			break;
		case ROBOT_EVENT_NET_STATUS_NOTICE:
			break;
		default:
			// unknown status datagram
			break;
	}

}

void on_1hz_timer(robot_event *ev){
}

void on_10hz_timer(robot_event *ev){
    sys_heartbeat(&conn, 1);
    sys_heartbeat(&conn, 2);
    sys_heartbeat(&conn, 3);
    sys_heartbeat(&conn, 4);
}

void on_command_code(robot_event *ev) {
	robot_event send_ev;
	switch(ev->command) {
		case ROBOT_EVENT_CMD_NOOP:
			send_ev.command = ROBOT_EVENT_NET_STATUS_OK;
			send_ev.index = 0;
			send_ev.value = 0;
			send_event(&send_ev);
			setPin(6,3,flasher);
			flasher = 1 - flasher;
			break;
		case ROBOT_EVENT_CMD_START:
           
			break;
		case ROBOT_EVENT_CMD_STOP:
			setPin(6,3,1);
			//setPin(1,4,1);

			break;
		case ROBOT_EVENT_CMD_REBOOT:
			break;
		default:
			// unknown command code datagram
			break;
	}

}
void on_set_variable(robot_event* ev){
}
void on_read_variable(robot_event* ev){
}

