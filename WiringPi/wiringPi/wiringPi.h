/*
 * wiringPi:
 *	Arduino compatable (ish) Wiring library for the Raspberry Pi
 *	Copyright (c) 2012 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Handy defines

// Deprecated
#define	NUM_PINS	17

#define	WPI_MODE_PINS		 0
#define	WPI_MODE_GPIO		 1
#define	WPI_MODE_GPIO_SYS	 2
#define	WPI_MODE_PHYS		 3
#define	WPI_MODE_PIFACE		 4
#define	WPI_MODE_UNINITIALISED	-1

// Pin modes

#define	INPUT			0
#define	OUTPUT			1
#define	PWM_OUTPUT		2
#define	PULLUP		 	3
#define	PULLDOWN		4
#define	PULLOFF			5
#define	CHECK		 	6
#define	GPIO_CLOCK		7

#define	LOW			 0
#define	HIGH			 1

// Pull up/down/none

#define	PUD_OFF			 0
#define	PUD_DOWN		 1
#define	PUD_UP			 2

// PWM

#define	PWM_MODE_MS		0
#define	PWM_MODE_BAL		1

// Interrupt levels

#define	INT_EDGE_SETUP		0
#define	INT_EDGE_FALLING	1
#define	INT_EDGE_RISING		2
#define	INT_EDGE_BOTH		3

// for 8bitbus
typedef struct{
unsigned char D0;
unsigned char D1;
unsigned char D2;
unsigned char D3;
unsigned char D4;
unsigned char D5;
unsigned char D6;
unsigned char D7;
}STRUCT_8BITS_BUS;

typedef struct{
unsigned char D0;
unsigned char D1;
unsigned char D2;
unsigned char D3;
unsigned char D4;
unsigned char D5;
unsigned char D6;
unsigned char D7;
unsigned char D8;
unsigned char D9;
unsigned char D10;
unsigned char D11;
unsigned char D12;
unsigned char D13;
unsigned char D14;
unsigned char D15;
}STRUCT_16BITS_BUS;



// Threads

#define	PI_THREAD(X)	void *X (void *dummy)

// wiringPiNodeStruct:
//	This describes additional device nodes in the extended wiringPi
//	2.0 scheme of things.
//	It's a simple linked list for now, but will hopefully migrate to 
//	a binary tree for efficiency reasons - but then again, the chances
//	of more than 1 or 2 devices being added are fairly slim, so who
//	knows....

struct wiringPiNodeStruct
{
  int     pinBase ;
  int     pinMax ;

  int          fd ;	// Node specific
  unsigned int data0 ;	//  ditto
  unsigned int data1 ;	//  ditto
  unsigned int data2 ;	//  ditto
  unsigned int data3 ;	//  ditto

  int   (*pinMode)         (struct wiringPiNodeStruct *node, int pin, int mode) ;
  void   (*pullUpDnControl) (struct wiringPiNodeStruct *node, int pin, int mode) ;
  int    (*digitalRead)     (struct wiringPiNodeStruct *node, int pin) ;
  int    (*digitalWrite)    (struct wiringPiNodeStruct *node, int pin, int value) ;
  void   (*pwmWrite)        (struct wiringPiNodeStruct *node, int pin, int value) ;
  int    (*analogRead)      (struct wiringPiNodeStruct *node, int pin) ;
  int   (*analogWrite)     (struct wiringPiNodeStruct *node, int pin, int value) ;

  struct wiringPiNodeStruct *next ;
} ;


// Function prototypes
//	c++ wrappers thanks to a comment by Nick Lott
//	(and others on the Raspberry Pi forums)

#ifdef __cplusplus
extern "C" {
#endif


// Core wiringPi functions

extern struct wiringPiNodeStruct *wiringPiNewNode (int pinBase, int numPins) ;

extern int  wiringPiSetup       (void) ;
extern int  wiringPiSetupSys    (void) ;
extern int  wiringPiSetupGpio   (void) ;
extern int  wiringPiSetupPhys   (void) ;

extern int pinMode             (int pin, int mode) ;
extern void pullUpDnControl     (int pin, int pud) ;
extern int  digitalRead         (int pin) ;
extern int digitalWrite        (int pin, int value) ;
extern void pwmWrite            (int pin, int value) ;
extern int  analogRead          (int pin) ;
extern int analogWrite         (int pin, int value) ;

// PiFace specifics 
//	(Deprecated)

extern int  wiringPiSetupPiFace (void) ;
extern int  wiringPiSetupPiFaceForGpioProg (void) ;	// Don't use this - for gpio program only

// On-Board Raspberry Pi hardware specific stuff

extern int  piBoardRev          (void) ;
extern int  wpiPinToGpio        (int wpiPin) ;
extern void setPadDrive         (int group, int value) ;
extern int  getAlt              (int pin) ;
extern void digitalWriteByte    (int value) ;
extern void pwmSetMode          (int mode) ;
extern void pwmSetRange         (unsigned int range) ;
extern void pwmSetClock         (int divisor) ;
extern void gpioClockSet        (int pin, int freq) ;

// Interrupts
//	(Also Pi hardware specific)

extern int  waitForInterrupt    (int pin, int mS) ;
extern int  wiringPiISR         (int pin, int mode, void (*function)(void)) ;

// Threads

extern int  piThreadCreate      (void *(*fn)(void *)) ;
extern void piLock              (int key) ;
extern void piUnlock            (int key) ;

// Schedulling priority

extern int piHiPri (int pri) ;

// Extras from arduino land

extern void         delay             (unsigned int howLong) ;
extern void         delayMicroseconds (unsigned int howLong) ;
extern unsigned int millis            (void) ;
extern unsigned int micros            (void) ;

// for 8/16 bit BUS
extern int Set8BitsBUS(STRUCT_8BITS_BUS *bus,char d0,char d1,char d2,char d3,char d4,char d5,char d6,char d7,char mode);
extern int digitalWrite8(STRUCT_8BITS_BUS *bus,char data);
extern unsigned char digitalRead8(STRUCT_8BITS_BUS *bus);
extern int Set16BitsBUS(STRUCT_16BITS_BUS *bus,char d0,char d1,char d2,char d3, char d4,char d5,char d6,char d7,char d8,char d9,char d10,char d11,char d12,char d13,char d14,char d15,char mode);
extern int digitalWrite16(STRUCT_16BITS_BUS *bus,short int data);
extern unsigned short int digitalRead16(STRUCT_16BITS_BUS *bus);

#ifdef __cplusplus
}
#endif
