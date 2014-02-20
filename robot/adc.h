//    adc.h - interface for joystick handling
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

#ifndef ADC_H
#define ADC_H

#include "robot_queue.h"

//#define NO_ADC   //To eliminate ADC code

#define ADC_COUNT 1 //Number of ADC's to poll, cant remember if its 0 or 1 indexed

#define POLL_INTERVAL 1000000/50  //50Hz polling interval


extern int adc_thread_create(robot_queue *q);
extern int adc_thread_destroy(); 


#endif //!ADC_H
