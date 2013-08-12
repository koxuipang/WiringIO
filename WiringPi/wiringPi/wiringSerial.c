/*
 * wiringSerial.c:
 *	Handle a serial port
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "wiringSerial.h"

/*
 * SerialBegin:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */
int gFdArray[6]={};

int SerialBegin (int ppp, int baud)
{
  struct termios options ;
  speed_t myBaud ;
  int     status, fd ;
  char *device;

	switch(ppp)
	{
		case 10:
			device = "/dev/ttyS0";
		break;
		case 11:
			device = "/dev/ttyS1";
		break;
		case 12:
			device = "/dev/ttyS2";
		break;
		case 13:
			device = "/dev/ttyS3";
		break;
		case 14:
			device = "/dev/ttyS4";
		break;
		default:
		break;
	}

  switch (baud)
  {
    case     50:	myBaud =     B50 ; break ;
    case     75:	myBaud =     B75 ; break ;
    case    110:	myBaud =    B110 ; break ;
    case    134:	myBaud =    B134 ; break ;
    case    150:	myBaud =    B150 ; break ;
    case    200:	myBaud =    B200 ; break ;
    case    300:	myBaud =    B300 ; break ;
    case    600:	myBaud =    B600 ; break ;
    case   1200:	myBaud =   B1200 ; break ;
    case   1800:	myBaud =   B1800 ; break ;
    case   2400:	myBaud =   B2400 ; break ;
    case   4800:	myBaud =   B4800 ; break ;
    case   9600:	myBaud =   B9600 ; break ;
    case  19200:	myBaud =  B19200 ; break ;
    case  38400:	myBaud =  B38400 ; break ;
    case  57600:	myBaud =  B57600 ; break ;
    case 115200:	myBaud = B115200 ; break ;
    case 230400:	myBaud = B230400 ; break ;

    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;

	switch(ppp)
	{
		case 10:
			gFdArray[0] = fd;
		break;
		case 11:
			gFdArray[1] = fd;
		break;
		case 12:
			gFdArray[2] = fd;
		break;
		case 13:
			gFdArray[3] = fd;
		break;
		case 14:
			gFdArray[4] = fd;
		break;
		default:
		break;
	}
	// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;	// 10mS

  return fd ;
}


/*
 * SerialFlush:
 *	Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

int SerialFlush (int fd)
{
	char statue;
	int  sfd = 0;

	switch(fd)
	{
		case 10:
			sfd = gFdArray[0];
		break;
		case 11:
			sfd = gFdArray[1];
		break;
		case 12:
			sfd = gFdArray[2];
		break;
		case 13:
			sfd = gFdArray[3];
		break;
		case 14:
			sfd = gFdArray[4];
		break;
		default:
		break;
	}

 	statue = tcflush (sfd, TCIOFLUSH) ;

	if(statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}


/*
 * SerialEnd:
 *	Release the serial port
 *********************************************************************************
 */

int SerialEnd (int fd)
{
	int statue,sfd = 0;
	
	switch(fd)
	{
		case 10:
			sfd = gFdArray[0];
		break;
		case 11:
			sfd = gFdArray[1];
		break;
		case 12:
			sfd = gFdArray[2];
		break;
		case 13:
			sfd = gFdArray[3];
		break;
		case 14:
			sfd = gFdArray[4];
		break;
		default:
		break;
	}

  	statue = close (sfd) ;
	if(statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}


/*
 * SerialWrite:
 *	Send a single character to the serial port
 *********************************************************************************
 */

int SerialWrite (int fd, unsigned char c)
{
	int statue,sfd = 0;
	
	switch(fd)
	{
		case 10:
			sfd = gFdArray[0];
		break;
		case 11:
			sfd = gFdArray[1];
		break;
		case 12:
			sfd = gFdArray[2];
		break;
		case 13:
			sfd = gFdArray[3];
		break;
		case 14:
			sfd = gFdArray[4];
		break;
		default:
		break;
	}
  	statue = write (sfd, &c, 1) ;
	if(statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}


/*
 * SerialPrint:
 *	Send a string to the serial port
 *********************************************************************************
 */

int SerialPrint (int fd, char *s)
{
	int statue,sfd = 0;
	
	switch(fd)
	{
		case 10:
			sfd = gFdArray[0];
		break;
		case 11:
			sfd = gFdArray[1];
		break;
		case 12:
			sfd = gFdArray[2];
		break;
		case 13:
			sfd = gFdArray[3];
		break;
		case 14:
			sfd = gFdArray[4];
		break;
		default:
		break;
	}
  	statue = write (sfd, s, strlen (s)) ;
	if(statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
int SerialPrintln (int fd, char *s)
{
	int statue = 0;
	int sfd = 0;

	switch(fd)
	{
		case 10:
			sfd = gFdArray[0];
		break;
		case 11:
			sfd = gFdArray[1];
		break;
		case 12:
			sfd = gFdArray[2];
		break;
		case 13:
			sfd = gFdArray[3];
		break;
		case 14:
			sfd = gFdArray[4];
		break;
		default:
		break;
	}
  	write (sfd, s, strlen (s)) ;
	printf("\n");
	if(statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
/*
 * serialPrintf:
 *	Printf over Serial
 *********************************************************************************
 */

void serialPrintf (int fd, char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  SerialPrint (fd, buffer) ;
}


/*
 * serialDataAvail:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int SerialAvailable (int fd)
{
  	int result,sfd = 0;

	switch(fd)
	{
		case 10:
			sfd = gFdArray[0];
		break;
		case 11:
			sfd = gFdArray[1];
		break;
		case 12:
			sfd = gFdArray[2];
		break;
		case 13:
			sfd = gFdArray[3];
		break;
		case 14:
			sfd = gFdArray[4];
		break;
		default:
		break;
	}
  	if (ioctl (sfd, FIONREAD, &result) == -1)
    	return -1 ;

  return result ;
}


/*
 * serialGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out after
 *	10 seconds.
 *********************************************************************************
 */

int SerialRead (int fd)
{
  	uint8_t x ;
	int sfd = 0;

	switch(fd)
	{
		case 10:
			sfd = gFdArray[0];
		break;
		case 11:
			sfd = gFdArray[1];
		break;
		case 12:
			sfd = gFdArray[2];
		break;
		case 13:
			sfd = gFdArray[3];
		break;
		case 14:
			sfd = gFdArray[4];
		break;
		default:
		break;
	}

  	if (read (sfd, &x, 1) != 1)
    	return -1 ;

  return ((int)x) & 0xFF ;
}
