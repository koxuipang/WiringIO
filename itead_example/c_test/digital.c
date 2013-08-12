/**
 * softPwm test 
 * author:gootoomoon 
 * gcc -o test digital.c -lwiringPi -lpthread
 */
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <itead.h>

int pin = 134;

int main()
{
	pinMode(pin,OUTPUT);

	for(;;){
		digitalWrite(pin,HIGH);
		delay(1000);
		digitalWrite(pin,LOW);
		delay(1000);
	}
}
