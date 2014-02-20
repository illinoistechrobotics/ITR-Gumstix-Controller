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
#include <math.h>
#include "robot_comm.h"
#include "robot_log.h"
#include "joystick.h"
#include "events.h"
#include "profile.h"

#define CTRL_DIRECT_DRIVE 0x01
#define CTRL_DIRECT_ANGLE 0x02
#define CTRL_VELOCITY 0x03

#define MAX 126 //So 0=>1 and 255=>254
#define M1POLARITY 1
#define M2POLARITY -1

#define TARGET_ANGLE 		0x00
#define TARGET_VELOCITY		0x01
#define TARGET_TURN_DIFF	0x02
#define TARGET_YAW_RATE		0x03
#define TARGET_YAW		0x04
#define KPROP			0x05
#define KRATE			0x06
#define KINT			0x07
#define KTERM			0x08
#define MAXPWM			0x09
#define FRAME_ANGLE_RESET	0x0A
#define VELOCITY_RESET		0x0B
#define DRIVE_MODE		0x0C

#define KPROP_VALUE		8
#define KRATE_VALUE		2
#define KINT_VALUE		0

#define DRIVE_NORMAL		6
#define DRIVE_TURBO		4
#define DRIVE_SUPER		1

#define TURN_NORMAL		8
#define TURN_TURBO		6
#define TURN_SUPER		1

unsigned char ctrl_mode = CTRL_DIRECT_DRIVE;
int xin = 0;
int yin = 0;
int drive_scale = DRIVE_NORMAL;
int turn_scale = TURN_NORMAL;
int turbo = 0;
int superboost = 0;

void on_init() {
	robot_event ev;
	ev.command = ROBOT_EVENT_CMD_START;
	ev.index = 0;
	ev.value = 0;

	log_string(-1, "Controller is initializing");
	send_event(&ev);

	ev.command = ROBOT_EVENT_SET_VAR;
	ev.index = KPROP;
	ev.value = KPROP_VALUE;
	send_event(&ev);

	ev.index = KRATE;
	ev.value = KRATE_VALUE;
	send_event(&ev);

	ev.index = KINT;
	ev.value = KINT_VALUE;
	send_event(&ev);

	ev.index = DRIVE_MODE;
	ev.value = ctrl_mode;
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
	if(ev->index == 5){
		turbo = 0;
		drive_scale = DRIVE_NORMAL;
		turn_scale = TURN_NORMAL;
	} else if (ev->index == 7) {
		superboost = 0;
		if(turbo){
			drive_scale = DRIVE_TURBO;
			turn_scale = TURN_TURBO;
		}
	} else if (ev->index == 4) {
		drive_scale = DRIVE_NORMAL;
		turn_scale = TURN_NORMAL;
	}
	send_event(ev);
}

void on_button_down(robot_event *ev) {
	if(ev->index == 5){
		turbo = 1;
		if(superboost){
			drive_scale = DRIVE_SUPER;
			turn_scale = TURN_SUPER;
		} else {
			drive_scale = DRIVE_TURBO;
			turn_scale = TURN_TURBO;
		}
	} else if (ev->index == 7){
		superboost = 1;
		if(turbo){
			drive_scale = DRIVE_SUPER;
			turn_scale = TURN_SUPER;
		}
	} else if (ev->index == 4) {
		drive_scale = DRIVE_SUPER;
		turn_scale = TURN_NORMAL;
	} else {
		send_event(ev);
	}
}

void on_axis_change(robot_event *ev) {
	/* +Y (up) move forward
	 * -Y (down) move backward
	 * -X (left) decrease left power, turn left
	 * +X (right) decrease right power, turn right
	 */

	int mot1 = 0;
	int mot2 = 0;
	
	robot_event new_ev;
	unsigned char axis = ev->index;
	unsigned char value = ev->value;
	if(ctrl_mode == CTRL_DIRECT_DRIVE){
		if(axis == 3 || axis == 2){

			if(axis == 3){
				yin = (int)value - 127;
				yin = ((float)yin / drive_scale);
			} else {
				xin = (int)value - 127;
				if(abs(xin) < (int)((float) abs(yin) / 32.0) + 3){ // Deadband scaled from 6 at full y, 3 at center
					xin = 0; 
				} else {
					xin = ((float)xin / turn_scale);
				}
			}

			mot1 = yin + xin;
			mot2 = yin - xin;

			mot1 *= M1POLARITY;
			mot2 *= M2POLARITY;
			
			mot1 += 128;
			mot2 += 128;

			if(mot1 > 127 + MAX) mot1 = 127 + MAX;
			if(mot1 < 127 - MAX) mot1 = 127 - MAX;
			if(mot2 > 127 + MAX) mot2 = 127 + MAX;
			if(mot2 < 127 - MAX) mot2 = 127 - MAX;

			new_ev.command = ROBOT_EVENT_MOTOR;

			new_ev.index = 0;
			new_ev.value = mot1;
			send_event(&new_ev);

			new_ev.index = 1;
			new_ev.value = mot2;
			send_event(&new_ev);
		}
	} else if (ctrl_mode == CTRL_DIRECT_ANGLE){
		if(axis == 3 || axis == 2){
			if(axis == 3){
				yin = (int)value - 127;
				yin = ((float)yin / drive_scale);
			} else if (axis == CON_XAXIS){
				xin = (int)value - 127;
				if(abs(xin) < (int)((float) abs(yin) / 32) + 3){ // Deadband scaled from 6 at full y, 3 at center
					xin = 0; 
				} else {
					xin = ((float)xin / turn_scale);
				}
			}

			if (yin > 120) yin = 120;
			if (yin < -120) yin = -120;

			new_ev.command = ROBOT_EVENT_SET_VAR;

			new_ev.index = TARGET_ANGLE;
			new_ev.value = yin;
			send_event(&new_ev);

			new_ev.index = TARGET_TURN_DIFF;
			new_ev.value = xin;
			send_event(&new_ev);
		}
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
	log_string(-1, "ADC %02X value %02X", ev->index, ev->value);
}

void on_read_variable(robot_event *ev){
	log_string(-1, "Var %02X: %d", ev->index, ev->value);
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

