import wiringIO
import sys

#wiringIO.UART0 =  10
#wiringIO.UART0 = wiringIO.SerialBegin('/dev/ttyS4',115200)
wiringIO.SerialBegin(wiringIO.UART0,115200)
wiringIO.SerialPrint(wiringIO.UART0,"\n\r")

wiringIO.SerialPrint(wiringIO.UART0,"Print:")
wiringIO.SerialPrint(wiringIO.UART0,"hello\n\r")

wiringIO.SerialPrint(wiringIO.UART0,"Write:")
wiringIO.SerialWrite(wiringIO.UART0,0x38)#sent ascii 0x38: 8
#wiringIO.SerialWrite(wiringIO.UART0,0x65)#sent ascii 0x68: A
wiringIO.SerialPrint(wiringIO.UART0,"\n\r")

wiringIO.SerialPrint(wiringIO.UART0,"Printf:")
wiringIO.serialPrintf(wiringIO.UART0,"hello\n\r")

#####get data from wiringIO.UART0
'''
while 1:
	if wiringIO.SerialAvailable(wiringIO.UART0):
		val = wiringIO.SerialRead(wiringIO.UART0)
		print chr(val)
'''
