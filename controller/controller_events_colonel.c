//
// controller_events.c
//
// This file contains all the user defined code for events triggered by
// a change of state. On the controller side, a change of state occurs
// when the joystick state changes. Or when the program starts and shuts
// down. See common/events.h for more complete information.
//
#define PI 3.1415926535897932384626433832

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "robot_comm.h"
#include "robot_log.h"
#include "joystick.h"
#include "events.h"
#include "profile.h"

int turbo = 0;

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
	if(ev->index == CON_TURBO1 || ev->index == CON_TURBO2)
		turbo--;
	send_event(ev);
}

int shoot = 0;
void on_button_down(robot_event *ev) {
	if(ev->index == CON_TURBO1 || ev->index == CON_TURBO2)
		turbo++;

    if(ev->index == 0x00){
        shoot = 1 - shoot;
	    robot_event new_ev;
	    new_ev.command = ROBOT_EVENT_MOTOR;
        new_ev.index = 4; new_ev.value = (shoot ? 255 : 127);
		send_event(&new_ev);
    }
	send_event(ev);
}

void on_axis_change(robot_event *ev) {
    
	 static int front = 0, rear = 0, lastfront = 0, lastrear = 0; //Motor and Steering values
	 static unsigned char xAxis = 127, yAxis = 127, rAxis = 127; //x (lateral) y(forward) and r (rotational) axis values
	 static int xNew = 0, yNew = 0, rNew =0, throttle = 0, strafe = 0; // 0 centered axis values
     static int frontswap, rearswap, tempfront, temprear;

	 robot_event new_ev;
    unsigned char axis = ev->index;
    unsigned char value = ev->value;
	 if(value == 255) value = 254;
	 if(value == 0) value = 1;
	 send_event(ev);
    
	 if(axis == CON_XAXIS || axis == CON_YAXIS || axis == CON_RAXIS || axis == CON_ZAXIS) {
        if(axis == CON_ZAXIS)
            yAxis = value;
        if(axis == CON_RAXIS)
            xAxis = value;
        if(axis == CON_XAXIS)
            rAxis = 255 - value;
	    if(axis == CON_YAXIS)
	        throttle = 255 - value;
        xNew = (int)xAxis - 128;
        yNew = (int)yAxis - 128;
        rNew = (int)rAxis - 128;
        rNew = (turbo == 0 ? rNew : rNew >> 1);

	    if(xNew < 0)
	    	strafe = (int)(((double)250*(double)atan((double)yNew/(double)xNew)/((double).5*(double)PI))-250); //Left side.
	    else if (xNew > 0)
		    strafe = (int)(((double)(250)*(double)atan((double)yNew/(double)xNew)/((double).5*(double)PI))+250); // Right Side.
	    else if (yNew <= 0)
		    strafe = 0;
	    else if (yNew > 0)
		    strafe = 500;
	
	front = strafe - (int)(rNew*200.0/127.0);
	rear = strafe + (int)(rNew*200.0/127.0);
	if(front > 999)
		front -= 1000;
	if(front < 0)
		front += 1000;
	if(rear > 999)
		rear -= 1000;
	if(rear < 0)
		rear += 1000;
    if(throttle < 137 && throttle > 117){
        tempfront = (front > 500 ? front - 500 : front + 500);
        temprear = (rear > 500 ? rear - 500 : rear + 500);
        if(abs(lastfront - front) < abs(lastfront - tempfront)){
            frontswap = 0;
        } else {
            frontswap = 1;
            front = tempfront;
        }
        if(abs(lastrear - rear) < abs(lastrear - temprear)){
            rearswap = 0;
        } else {
            rearswap = 1;
            rear = temprear;
        }
    } else {
        if(frontswap == 1)
            front = (front > 500 ? front - 500 : front + 500);
        if(rearswap == 1)
            rear = (rear > 500 ? rear - 500 : rear + 500);
    }
        int frontdrive = (frontswap == 0 ? throttle : 255 - throttle);
        int reardrive = (rearswap == 0 ? 255 - throttle : throttle);
	new_ev.command = ROBOT_EVENT_MOTOR;
        
        new_ev.index = 0; new_ev.value = frontdrive;
		send_event(&new_ev);

        new_ev.index = 1; new_ev.value = (((frontdrive-127)*turbo)/2)+127;
		send_event(&new_ev);

        new_ev.index = 2; new_ev.value = reardrive;
		send_event(&new_ev);
        
        new_ev.index = 3; new_ev.value = (((reardrive-127)*turbo)/2)+127;
		send_event(&new_ev);
        
        lastfront = front;
        new_ev.index = 6; new_ev.value = front;
        send_event(&new_ev);

        lastrear = rear;
        new_ev.index = 7; new_ev.value = rear;
		send_event(&new_ev);
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
