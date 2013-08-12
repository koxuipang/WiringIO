/*
 * wiringSerial.h:
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

#define UART0 10
#define UART1 11
#define UART2 12
#define UART3 13
#define UART4 14

#ifdef __cplusplus
extern "C" {
#endif

extern int   SerialBegin     (int ppp, int baud) ;
extern int   SerialEnd       (int fd) ;
extern int   SerialFlush     (int fd) ;
extern int   SerialWrite     (int fd, unsigned char c) ;
extern int   SerialPrint     (int fd, char *s) ;
extern int   SerialPrintln   (int fd, char *s) ;
extern void  serialPrintf    (int fd, char *message, ...) ;
extern int   SerialAvailable (int fd) ;
extern int   SerialRead      (int fd) ;

#ifdef __cplusplus
}
#endif
