import wiringIO
import sys

pin = 134
#pin = 121 #hard pwm
PWM_OUTPUT = 2
wiringIO.pinMode(pin,PWM_OUTPUT)
MS = 10

print "**********************"
print "delay:%s ms" %(MS)
print "**********************"

while 1:
    for i in range(0, 255):
	    wiringIO.analogWrite(pin,i)
	    wiringIO.delay(MS)
    for i in range(0,255):
	    wiringIO.analogWrite(pin,255-i)
	    wiringIO.delay(MS)

