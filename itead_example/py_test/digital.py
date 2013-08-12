import wiringIO as IO
import sys

pin =134

IO.pinMode(pin,IO.OUTPUT)

Mo = IO.pinMode(pin,IO.CHECK)
MS = 1000

print "**********************"
print "delay:%s ms,Mode:%s" %(MS,Mo)
print "**********************"

while 1:
	IO.digitalWrite(pin,1)
	val = IO.digitalRead(pin)
        print "LED on val:%d" %(val)
	IO.delay(MS)
	IO.digitalWrite(pin,0)
	val = IO.digitalRead(pin)
        print "LED off val:%d" %(val)
	IO.delay(MS)
