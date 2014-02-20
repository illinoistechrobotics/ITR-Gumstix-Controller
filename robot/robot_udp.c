/* UDP client in the internet domain */
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include "AvrInfo.h"
#include "i2c-dev.h"
#include "i2c-api.h"
#include "i2c-io-api.h"
#include "BootLoader-api.h"
#include "Log.h"


enum 
{
	BUF_HEADER,
	BUF_SEQ0,
	BUF_SEQ1,
	BUF_SEQ2,
	BUF_SEQ3,
	BUF_LEFT,
	BUF_RIGHT,
	BUF_WINCH,
	BUF_PITCH,
	BUF_RELAYS,
}; 

// each speed is an unsigned char from 0-255
typedef struct {
	char header; // one byte header
	char seq[4]; // Adam knows what this is
	char left_speed; // speed of the left set of motors
	char right_speed; // speed of the right side of the motors
	char winch_speed; // speed of the winch motor
	char pitch_speed; // speed of the pitch motor
	char relays; // bit field of relay booleans
} packet;

#define TRUE    1
#define FALSE   0

void error(char *);
void doStuff(char buffer[]);
void set_motor_speed(int motor, int speed);

int main(int argc, char *argv[])
{
   const char         *i2cDevName = "/dev/i2c-0";
   int sock, length, n;
   struct sockaddr_in server, from;
   struct hostent *hp;
   char buffer[16];
   unsigned int i2cDev;
   unsigned long high_seq[4] = {0,0,0,0};
   int flags;
   char bool = 1; 
   struct timeval last, now;

   if (argc != 2) {
	   usage(argv[0]);
	   exit(1);
   }
   sock = socket(AF_INET, SOCK_DGRAM, 0);
   if (sock < 0) error("socket");

   server.sin_family = AF_INET;
   hp = gethostbyname(argv[1]);
   if (hp==0) error("Unknown host");

   memcpy((char *)&server.sin_addr,
	 (char *)hp->h_addr, 
         hp->h_length);
   server.sin_port = htons(31337); //htons(atoi(argv[2]));
   length=sizeof(struct sockaddr_in);
   
   flags = fcntl(sock, F_GETFL); //next lines make reads not block
   if (flags == -1) error ("get flags");
   flags |= O_NONBLOCK; 
   if (fcntl(sock, F_SETFL, flags) == -1) error("set flags");
  
   sendto(sock,buffer,8,0,(struct sockaddr *)&server,length);
   //do //wait for START command
   //{
	//int a;
	//bzero(buffer,16);
  	//n=recvfrom(sock,buffer,16,0,(struct sockaddr *)&from, &length);
  	//if(n < 0 && errno != EWOULDBLOCK) error("Recvfrom");
	//write(1,"Test\n",5);
	//for(a = 0; a < 16 && buffer[a] == 0xCC; a++)
		//if(a>=15) bool = 0;
   //}while(bool);

   gettimeofday(&last, NULL);
   gettimeofday(&now, NULL);
   if (( i2cDev = open( i2cDevName, O_RDWR )) < 0 ) //Open I2C device
   {
       LogError( "Error  opening '%s': %s\n", i2cDevName, strerror( errno ));
       exit( 1 );
   }
   I2cSetSlaveAddress( i2cDev, gI2cAddr, I2C_USE_CRC ); // set I2C slave device
   
   ProcessWriteRegCommand("TCCR1A", 0xAA);
   ProcessWriteRegCommand("TCCR1B", 0x1A);
   ProcessWriteRegCommand("ICR1", 40000);
   ProcessWriteRegCommand("TCNT1", 0);
   ProcessWriteRegCommand("OCR1A", 3000);
   ProcessWriteRegCommand("OCR1B", 3000);
   ProcessWriteRegCommand("OCR1C", 3000);
   ProcessWriteRegCommand("TCCR3A", 0xAA);
   ProcessWriteRegCommand("TCCR3B", 0x1A);
   ProcessWriteRegCommand("ICR3", 40000);
   ProcessWriteRegCommand("TCNT3", 0);
   ProcessWriteRegCommand("OCR3A", 3000);
   ProcessWriteRegCommand("OCR3B", 3000);
   ProcessWriteRegCommand("OCR3C", 3000);
   ProcessSetDirCommand("b.5","out");
   ProcessSetDirCommand("b.6","out");
   ProcessSetDirCommand("b.7","out");
   ProcessSetDirCommand("e.3","out");
   ProcessSetDirCommand("e.4","out");
   ProcessSetDirCommand("e.5","out");

   
   while(1)
   {  
       //printf("done\n");
      bzero(buffer,16);
      n = recvfrom(sock,buffer,16,0,(struct sockaddr *)&from,length);
      
      if (n < 0 && errno != EWOULDBLOCK) error("recvfrom");
      
      if (buffer[0] == (char)0xA0)
      {
	unsigned long *seq = &buffer[1]; 
	//if(*seq > high_seq[0]) 
	//{
		high_seq[0] = *seq;
		printf("Recieved: Seq: %u  Kill: %d  L: %d  R: %d\n", *seq, buffer[0]&&0x01, (signed char)buffer[5], (signed char)buffer[6]);
		//doStuff(buffer);
	}
      //}
      //else if ((buffer[0] && 0x01) == 0x01)
      //{
	      //kill_outputs();
      //}
      
      /*gettimeofday(&now,NULL);
      if(now.tv_sec > last.tv_sec || now.tv_usec > last.tv_usec + 200000)  //if now is later than 200000uS since last time then we need to send an ACK
      {
	      printf("ACK Timer %d %d\n",now.tv_sec, now.tv_usec);
	      gettimeofday(&last,NULL);
	      if(high_seq[0] == high_seq[1] && high_seq[0] == high_seq[2] && high_seq[0] == high_seq[3])
	      {
		      //kill_outputs();
	      }
	      high_seq[3] = high_seq[2];
	      high_seq[2] = high_seq[1];
	      high_seq[1] = high_seq[0];

	     
	      buffer[0] = 0xB0;
	      *((unsigned long*)(&buffer[1])) = high_seq[0];
	      sendto(sock,buffer,8,0,(struct sockaddr *)&server,length);
      }*/
	//usleep(10000);      
   }
close( i2cDev );
}

void error(char *msg)
{
    fputs(msg, stderr);
    exit(1);
}

// TODO: give this a better name, possibly get rid of this altogether. and put it in the program loop
void doStuff(char buffer[])
{
	set_motor_speed(0, buffer[BUF_LEFT]);
	set_motor_speed(1, buffer[BUF_LEFT]);
	set_motor_speed(2, buffer[BUF_RIGHT]);
	set_motor_speed(3, buffer[BUF_RIGHT]);
}

// set_motor_speed
// we have four motors labeled 0-3
// the speed is an unsigned char from 0-255
//
// motor 0 is on register OCR1A
// motor 1 is on register OCR1B
// motor 2 is on register OCR3A
// motor 3 is on register OCR3B
void set_motor_speed(int motor, int speed) {
	char motor_regs[4][5] = {"OCR1A", "OCR1B", "OCR3A", "OCR3B"};
	if((motor >= 0) && (motor < 4))
		ProcessWriteRegCommand(motor_regs[motor],( (uint16_t)speed * 8u ) + 500 );
}


// usage - prints usage information
void usage(char *arg0) {
	fprintf(stderr, "Usage: %s server [options] i2c-address\n", arg0);
}
