//mod_i2c-io header for extern functions
#ifndef MOD_I2C_IO_H
#define MOD_I2C_IO_H

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/timeb.h>

#include "AvrInfo.h"
#include "i2c-dev.h"
#include "i2c-api.h"
#include "i2c-io-api.h"
#include "BootLoader-api.h"
#include "Log.h"
extern void init(int i2cSlave); //Initiization (connects gumstix to robostix) **REQUIRED BEFORE OTHER COMMANDS**
extern void servoInit(); //Intitialize the servo/motor timers *Required for servo control*
extern void setMotor(int motor, int position); //Sets a motor or servo based on a 0-255 value
extern void setServo(int servo, int position); //Sets a motor or servo based on a 0-255 value
extern void setMotorPWM(int motor, int msLength); //Sets a motor or servo based on a 500-2500 value
extern void setServoPWM(int motor, int msLength); //Sets a motor or servo based on a 500-2500 value
extern int getPin(uint8_t portNum, uint8_t pin); //Returns the value of a pin (1 or 0)
extern int getDir(uint8_t portNum, uint8_t pin); //Returns the direction of a pin (1 out or 0 in)
extern void setMotorDR(int motor, int position, int percent); //Servo control using dual rate setup
extern uint16_t getADC(uint8_t pin); //Get ADC value, returned as a 16 bit unsigned integer
extern void setPin(uint8_t port, uint8_t pin, uint8_t value); //Set pin (faster)
extern void setDir(uint8_t port, uint8_t pin, uint8_t value); //SetDir (faster) (0=off)
extern unsigned short readEnc(int encNumber);
extern void setVariable(uint8_t var, short data);
extern void steer(int encNumber, uint16_t direction);
extern signed short readVariable(uint8_t var);
#endif // !MOD_I2C_IO_H
