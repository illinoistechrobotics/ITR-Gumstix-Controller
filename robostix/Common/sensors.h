//    sensors.h - collect and process sensor data for Fenrir
//    Copyright (C) 2010  Illinois Institute of Technology Robotics
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

#ifndef SENSORS_H
#define SENSORS_H

#include <inttypes.h>
#include "../i2c-io/Hardware.h"
#include "a2d.h"

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

#define DIRECT_DRIVE		0x01
#define DIRECT_ANGLE		0x02
#define DIRECT_VELOCITY		0x03

#define M1POLARITY		1
#define M2POLARITY		-1

#define MAX			126

extern signed short global_vars [];
extern signed short frame_angle;
extern signed short error;
extern signed long isum;
extern void initSensors(void);
extern void checkArm(void);
extern void processData(void);
#endif // !SENSORS_H
