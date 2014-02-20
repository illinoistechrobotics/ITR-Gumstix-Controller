//    sensors.c - collect and process sensor data for Fenrir
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

#include <avr/io.h>
#include "sensors.h"
#define X_GYRO_AMP_ADC	0x00
#define X_GYRO_FULL_ADC	0x01
#define X_ACCEL_ADC	0x02
#define Z_ACCEL_ADC	0x03
#define Z_GYRO_AMP_ADC  0x04
#define VOLT_METER_ADC	0x05

#define GYRO_MIN	92
#define GYRO_ZERO	492
#define GYRO_MAX	892

#define GYRO_AMP_LIMIT_LOW	132
#define GYRO_AMP_LIMIT_HIGH	852

#define GYRO_FULL_LIMIT_LOW	412
#define GYRO_FULL_LIMIT_HIGH	572

signed short global_vars[13];
signed short frame_angle;
signed short error;
signed long isum;

void calculatePWM(void);

void initSensors(){
	global_vars[0] = 0;
	global_vars[1] = 0;
	global_vars[2] = 0;
	global_vars[3] = 0;
	global_vars[4] = 0;
	global_vars[5] = 4;
	global_vars[6] = 0;
	global_vars[7] = 0;
	global_vars[8] = 0;
	global_vars[9] = 255;
	global_vars[10] = 0;
	global_vars[11] = 0;
	global_vars[12] = 0;
	DDRC &= ~(1 << PORTC0);
	PORTC |= (1 << PORTC0);
}

//Called every millisecond
void processData(){

	checkArm();
	
	PORTB |= (PINB ^ (1 << 4));

	if(global_vars[DRIVE_MODE] == DIRECT_ANGLE){
		uint16_t x_gyro_amp;
		uint16_t x_gyro_full;
		signed short x_gyro;

		static long x_gyro_sum;

		//Read data from ADCs
		//
		x_gyro_amp = a2d_10(X_GYRO_AMP_ADC);
		x_gyro_full = a2d_10(X_GYRO_FULL_ADC);

		if (x_gyro_amp < GYRO_AMP_LIMIT_LOW || x_gyro_amp > GYRO_AMP_LIMIT_HIGH){
			x_gyro = (signed short)x_gyro_amp - GYRO_ZERO;
		} else {
			x_gyro = ((signed short)x_gyro_full - GYRO_ZERO) << 2;
		}

		x_gyro_sum += x_gyro; //Integrate w/ trapezoidal rule, scaled 1000x because we don't divide by 1000

		frame_angle = (x_gyro_sum - (x_gyro >> 1)) / 1000;

		calculatePWM();

	}
}

void checkArm(){
	//If the limit switch is pressed and the arm is moving up stop the arm
	if (!(PINC & ( 1 << PINC0)) && OCR3C > 1500)
		OCR3C = 1340;
}

void calculatePWM(){
	signed short last_error = error;
	isum += error;
	error = global_vars[TARGET_ANGLE] - frame_angle;
	signed short rate = error - last_error;
	signed short pwmout = ((error * global_vars[KPROP]) >> 3) + ((rate * global_vars[KRATE]) >> 3) + ((isum * global_vars[KINT]) >> 3);

	signed short mot1, mot2;
	mot1 = pwmout + global_vars[TARGET_TURN_DIFF];
	mot2 = pwmout - global_vars[TARGET_TURN_DIFF];

	mot1 *= M1POLARITY;
	mot2 *= M2POLARITY;

	mot1 += 128;
	mot2 += 128;

	if(mot1 > 127 + MAX) mot1 = 127 + MAX;
	if(mot1 < 127 - MAX) mot1 = 127 - MAX;
	if(mot2 > 127 + MAX) mot2 = 127 + MAX;
	if(mot2 < 127 - MAX) mot2 = 127 - MAX;

	//Convert pwm to ms pulsewidth
	mot1 = (((mot1 << 3) + 500)  << 1);
	mot2 = (((mot2 << 3) + 500)  << 1);

	OCR3A = mot1;
	OCR3B = mot2;

	/*
	//Set PWM registers
	uint8_t mot1L = (mot1 & 0xFF);
	uint8_t mot1H = ((mot1 >> 8) & 0xFF);

	uint8_t mot2L = (mot2 & 0xFF);
	uint8_t mot2H = ((mot2 >> 8) & 0xFF);
	volatile uint8_t *motreg = (volatile uint8_t *) 0x86;

	motreg[0] = mot1L;
	motreg[1] = mot1H;

	motreg = (volatile uint8_t *) 0x84;
	motreg[0] = mot2L;
	motreg[1] = mot2H;
	*/

	return;
}
