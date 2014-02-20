/****************************************************************************
 *
 *   Copyright (c) 2006 Dave Hylands     <dhylands@gmail.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   Alternatively, this software may be distributed under the terms of BSD
 *   license.
 *
 *   See README and COPYING for more details.
 *
 ****************************************************************************/
/**
 *
 *   @file   i2c-io.c
 *
 *   @brief  This file implements commands for performing I/O on the robostix
 *           via the i2c bus.
 *
 ****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/timeb.h>
#include <semaphore.h>

#include "AvrInfo.h"
#include "i2c-dev.h"
#include "i2c-api.h"
#include "i2c-io-api.h"
#include "BootLoader-api.h"
#include "Log.h"
#include "mod_i2c-io.h"

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------

// ---- Private Variables ---------------------------------------------------




enum
{
	PINF   =  	0x020 ,
	PINE   =	0x021 ,
	DDRE   =	0x022 ,
	PORTE  =	0x023 ,
	ADCL   =  	0x024 ,
	ADCH   =  	0x025 ,
	ADCSR  =  	0x026 ,
	ADMUX  =  	0x027 ,
	ACSR   =  	0x028 ,
	UBRR0L =  	0x029 ,
	UCSR0B =  	0x02A ,
	UCSR0A =  	0x02B ,
	UDR0   =  	0x02C ,
	SPCR   =  	0x02D ,
	SPSR   =  	0x02E ,
	SPDR   =  	0x02F ,
	PIND   =  	0x030 ,
	DDRD   =  	0x031 ,
	PORTD  =  	0x032 ,
	PINC   =  	0x033 ,
	DDRC   =  	0x034 ,
	PORTC  =  	0x035 ,
	PINB   =  	0x036 ,
	DDRB   =  	0x037 ,
	PORTB  =  	0x038 ,
	PINA   =  	0x039 ,
	DDRA   =  	0x03A ,
	PORTA  =  	0x03B ,
	SFIOR  =  	0x040 ,
	WDTCR  =  	0x041 ,
	OCDR   =  	0x042 ,
	OCR2   =  	0x043 ,
	TCNT2  =  	0x044 ,
	TCCR2  =  	0x045 ,
	ICR1L  =  	0x046 ,
	ICR1H  =  	0x047 ,
	OCR1BL =  	0x048 ,
	OCR1BH =  	0x049 ,
	OCR1AL =  	0x04A ,
	OCR1AH =  	0x04B ,
	TCNT1L =  	0x04C ,
	TCNT1H =  	0x04D ,
	TCCR1B = 	0x04E ,
	TCCR1A =  	0x04F ,
	ASSR   =  	0x050 ,
	OCR0   =  	0x051 ,
	TCNT0  =  	0x052 ,
	TCCR0  =  	0x053 ,
	MCUSR  =  	0x054 ,
	MCUCR  =  	0x055 ,
	TIFR   =  	0x056 ,
	TIMSK  =  	0x057 ,
	EIFR   =  	0x058 ,
	EIMSK  =  	0x059 ,
	EICRB  =  	0x05A ,
	RAMPZ  =  	0x05B ,
	XDIV   =  	0x05C ,

	ADC    =  	0x124 ,
	ICR1   =	0x146 ,
	OCR1B  =  	0x148 ,
	OCR1A  = 	0x14A ,
	TCNT1  = 	0x14C ,

	DDRF   =  	0x061 ,
	PORTF  = 	0x062 ,
	PING   = 	0x063 ,
	DDRG   = 	0x064 ,
	PORTG  = 	0x065 ,
	SPMCR  = 	0x068 ,
	EICRA  = 	0x06A ,
	XMCRB  = 	0x06C ,
	XMCRA  = 	0x06D ,
	OSCCAL = 	0x06F ,
	TWBR   = 	0x070 ,
	TWSR   = 	0x071 ,
	TWAR   = 	0x072 ,
	TWDR   = 	0x073 ,
	TWCR   = 	0x074 ,
	OCR1CL = 	0x078 ,
	OCR1CH = 	0x079 ,
	TCCR1C = 	0x07A ,
	ETIFR  = 	0x07C ,
	ETIMSK = 	0x07D ,
	ICR3L  = 	0x080 ,
	ICR3H  = 	0x081 ,
	OCR3CL = 	0x082 ,
	OCR3CH = 	0x083 ,
	OCR3BL = 	0x084 ,
	OCR3BH = 	0x085 ,
	OCR3AL = 	0x086 ,
	OCR3AH = 	0x087 ,
	TCNT3L = 	0x088 ,
	TCNT3H = 	0x089 ,
	TCCR3B = 	0x08A ,
	TCCR3A = 	0x08B ,
	TCCR3C = 	0x08C ,
	UBRR0H = 	0x090 ,
	UCSR0C = 	0x095 ,
	UBRR1H = 	0x098 ,
	UBRR1L = 	0x099 ,
	UCSR1B = 	0x09A ,
	UCSR1A = 	0x09B ,
	UDR1   = 	0x09C ,
	UCSR1C = 	0x09D ,

	OCR1C  = 	0x178 ,
	ICR3   =	0x180 ,
	OCR3C  =	0x182 ,
	OCR3B  =	0x184 ,
	OCR3A  =	0x186 ,
	TCNT3  =	0x188 ,
};

#define ARRAY_LEN( x )  ( sizeof( x ) / sizeof( x[0] ))




static int i2cDev = -1;
static sem_t i2clock;

// ---- Private Function Prototypes -----------------------------------------

static void writeReg(int RevNum, int data);
const char         *i2cDevName = "/dev/i2c-0";

static int lock () {
	sigset_t   signal_mask;  /* signals to block         */

	// List of signals to block
	sigemptyset (&signal_mask);
	sigaddset (&signal_mask, SIGINT);
	sigaddset (&signal_mask, SIGTERM);
	sigaddset (&signal_mask, SIGHUP);
	// Block signals and lock the queue
	if (pthread_sigmask(SIG_BLOCK, &signal_mask, NULL) == 0) {
		return !(sem_wait(&i2clock));
	} else {
		return 0;
	}
}
static int unlock () {
	sigset_t   signal_mask;  /* signals to block         */

	// List of signals to block
	sigemptyset (&signal_mask);
	sigaddset (&signal_mask, SIGINT);
	sigaddset (&signal_mask, SIGTERM);
	sigaddset (&signal_mask, SIGHUP);

	// Unlock the queue and unblock signals
	if (sem_post(&i2clock) == 0) {
		return !pthread_sigmask(SIG_UNBLOCK, &signal_mask, NULL);
	} else {
		return 0;
	}
}



//********************************************************************************
/**
 *	Initilization for i2c-io code
 */

void init(int i2cslave){
	sem_init(&i2clock,0,1);
	lock();
	if (( i2cDev = open( i2cDevName, O_RDWR )) < 0 )
	{
		LogError( "Error  opening '%s': %s\n", i2cDevName, strerror( errno ));
		exit( 1 );
	}
	I2cSetSlaveAddress( i2cDev, 0x0b, I2C_USE_CRC );
	unlock();
}
// End Init

//*********************************************************************************
/*
 *   Servo Initilization
 */


void servoInit(){
	writeReg(TCCR1A, 170); //Servo timer and PWM initialization
	writeReg(TCCR3A, 170);
	writeReg(TCCR1B, 26);
	writeReg(TCCR3B, 26);
	writeReg(ICR1, 40000);
	writeReg(ICR3, 40000);
	writeReg(TCNT1, 0);
	writeReg(TCNT3, 0);	  //Servo timer and PWM are initialized
	setMotor(0,127);
	setMotor(1,127);
	setMotor(2,127);
	setMotor(3,127);
	setMotor(4,127);
	setMotor(5,127);
	setDir(1,5,1);	  		//set Servos as outputs
	setDir(1,6,1);
	setDir(1,7,1);
	setDir(4,3,1);
	setDir(4,4,1);
	setDir(4,5,1);	 		//End Servo Output set
	setDir(1,4,1);     	// Set LED pins to output YELLOW LED
	setDir(6,3,1); 	 	// BLUE LED
	setDir(2,0,1);			//PortC output settings
	setPin(2,0,0);
	setDir(2,1,1);
	setPin(2,1,0);
	setDir(2,2,1);
	setPin(2,2,0);
	setDir(2,3,1);
	setPin(2,3,0);
	setDir(2,4,1);
	setPin(2,4,0);
	setDir(2,5,1);
	setPin(2,5,0);
	setDir(2,6,1);
	setPin(2,6,0);
	setDir(2,7,1);			//end PortC output settings
	setPin(2,7,0);


}


// End Servo initialization


//*********************************************************************************
/*
 *   Set Servo/Motor based on 255 value
 */


void setMotor(int motor, int position){ 
	if( motor <= 5  && motor >= 0) { 				//Check and make sure the motor is one we know about
		int motor_regs[] = {0x82, 0x84, 0x86, 0x4A, 0x48, 0x78}; 	//Register map, corrolates to output 3A, 3B, 3C, 1A, 1B, 1C
		if(position > 255) position = 255;
		if(position < 0) position = 0;
		uint16_t regVal16 = (((position*8u)+500)*2); 		//Equation to convert 0-255 to 500-2500 (from gumstix wiki)
		lock();
		I2C_IO_WriteReg16( i2cDev, motor_regs[motor], regVal16 ); //Send register write command for the correct motor register
		unlock();
	}
}

void setServo(int servo, int position){
	setMotor(servo, position); 	//Call setMotor function (functions have same API and function)
}



//End Set Servo/Motor based on 255



//*********************************************************************************
/*   Set Servo/Motor based on 500-2500 range (ms PWM length)
 */


void setMotorPWM(int motor, int msLength){
	if( motor <= 5  && motor >= 0) {                             //Check and make sure the motor is one we know about
		int motor_regs[] = { OCR3A , OCR3B , OCR3C , OCR1A , OCR1B , OCR1C };  //Register map
		lock();
		I2C_IO_WriteReg16( i2cDev, motor_regs[motor], msLength * 2 ); //Send register write command for the correct motor register, multiply times 2 to allow the correct value
		unlock();
	}
}

void setServoPWM(int motor, int msLength){
	setMotorPWM(motor, msLength);
}


//End Set Servo/Motor based on PWM length


//***********************************************************************************
/*    Set/get  pin direction/set pin
 */



int getPin(uint8_t portNum, uint8_t pin){
	uint8_t pinVal;
	lock();
	if ( I2C_IO_GetGPIO( i2cDev, portNum, &pinVal ))
	{
		unlock();
		return (( pinVal & ( 1 << pin )) != 0 );
	}
	else
	{
		LogError( "Failed to retrieve value for %c.%d\n",
				portNum + 'A', pin );
		unlock();
		return 0;
	}
}

int getDir(uint8_t portNum, uint8_t pin){
	uint8_t pinVal;
	lock();
	if ( I2C_IO_GetGPIODir( i2cDev, portNum, &pinVal ))
	{
		unlock();
		return ((( pinVal & ( 1 << pin )) != 0 ) ? 1 : 0 );
	}
	else
	{
		LogError( "Failed to retrieve direction for %s.%d\n",
				portNum + 'A', pin );
	}
	unlock();
	return 0;
}

//End set commands


//******************************************************************
/* getADC: Gets the value from an Analog to Digital Converter (16 bit integer)
 *  Pin must be between 0 and 7 (corrosponding to the ADC numbers on the board)
 */

uint16_t getADC(uint8_t pin){
	uint16_t    adcVal;
	lock();
	if ( I2C_IO_GetADC( i2cDev, pin, &adcVal ))
	{
		unlock();
		return adcVal;
	}
	unlock();
	return 0;
}


void setPin(uint8_t portNum, uint8_t pin, uint8_t value){
	if(value == 0 || value == 1){
		uint8_t pinMask;
		uint8_t pinVal;
		pinMask = 1 << pin;
		if(value == 0)
			pinVal = 0;
		else
			pinVal = pinMask;
		lock();
		I2C_IO_SetGPIO( i2cDev, portNum, pinMask, pinVal );
		unlock();
	}
}

void setDir(uint8_t portNum, uint8_t pin, uint8_t value){
	uint8_t pinMask;
	uint8_t pinVal;
	pinMask = 1 << pin;
	if(value == 0 || value == 1){
		if(value == 1)
			pinVal = pinMask;
		if(value == 0)
			pinVal = 0;
		lock();
		I2C_IO_SetGPIODir( i2cDev, portNum, pinMask, pinVal );
		unlock();
	}
}

void writeReg(int RegNum, int data){
	if((RegNum >> 8) == 1){ //If is16Bit is true
		uint16_t regVal16 = data; //create 16 bit unsigned it from data
		RegNum = RegNum & ~0x100; //Remove is16Bit bit from regNum
		lock();
		I2C_IO_WriteReg16( i2cDev, RegNum, regVal16 ); //Write Register
		unlock();
	}
	if((RegNum >> 8) == 0){ //If is16Bit is false
		uint8_t regVal8 = data; //Create 8 bit unsigned int from data
		lock();
		I2C_IO_WriteReg8( i2cDev, RegNum, regVal8 );//Write Register
		unlock();
	}
}

unsigned short readEnc(int encNumber){
	unsigned char addr = 0;
	switch (encNumber){
		case 0:
			addr = 0x36;
			break;
		case 1:
			addr = 0x3E;
			break;
		default:
			return 0;
	}
	short temp = 0;
	lock();
	I2cSetSlaveAddress( i2cDev, addr, 0 );
	I2cReadBytes( i2cDev, 10, &temp, 2);
	I2cSetSlaveAddress( i2cDev, 0x0b, I2C_USE_CRC );
	unlock();
	temp = ((temp & 0xFF) << 8) | ((temp & 0xFF00) >> 8);
	return temp;
}
	
void setVariable(uint8_t var, short data){
	unsigned char d [3];
	d[0] = var;
	d[1] = (data >> 8) & 0xFF;
	d[2] = data & 0xFF;
	lock();
	I2cWriteBytes( i2cDev, 11, &d, 3);
	unlock();
}

signed short readVariable(uint8_t var){
	unsigned char d [3];
	d[0] = var;
	d[1] = 0x00;
	d[2] = 0x00;
	lock();
	I2cReadBytes( i2cDev, 12, &d, 2);
	unlock();
	signed short data = (signed short)d[1];
	data |= (signed short)d[2] << 8;
	return data;
}


void steer(int encNumber, uint16_t direction){
	unsigned char addr = 0;
	switch (encNumber){
		case 1:
			addr = 0x36;
			break;
		case 0:
			addr = 0x3E;
			break;
		default:
			return;
	}
	direction = ((direction & 0xFF) << 8) | ((direction & 0xFF00) >> 8);
	lock();
	I2cSetSlaveAddress( i2cDev, addr, 0);
	I2cWriteBytes( i2cDev, 1, &direction, 2);
	I2cSetSlaveAddress( i2cDev, 0x0b, I2C_USE_CRC);
	unlock();
}
