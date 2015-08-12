/*
	
	File: 		motor_directions.h
	Author: 	Daniel Busan
	Created:	May 2015
	Version: 	1.7.1
	
	Source file containing the definitions of the functions handling the motor directions
*/

#include "motor_directions.h"
#include <Arduino.h>


// a basic mapping function to transform values from one range to another.
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void set_l_motor(int motor_speed)
{
	int mSpeed = motor_speed;
	if (mSpeed < 0)
		mSpeed = -mSpeed;

	if (motor_speed >= 0) // if input motor speed is greater than 0, motor is turning forwards.
	{
		// LEFT MOTOR - FORWARD DIRECTION
		PORTA |= (1<<2);  
		PORTA &= ~(1<<3); 
	}
	else // motor is reversing
	{	
		// LEFT MOTOR - REVERSE DIRECTION
		PORTA &= ~(1<<2);
		PORTA |= (1<<3); 
	
	}
	
	// if motor_speed is greater than 126, as discussed during the lecture, the motor will be turning at maximum speed
	if (motor_speed <= -126 || motor_speed >= 126) // set to max
	{
		OCR1B = 9999; 
	}	
	else // if not, transform the input value to a pwm-suitable range and set the PWM accordingly
	{
		OCR1B = map(mSpeed, 0, 125, 0, 9999);
	}
}

void set_r_motor(int motor_speed)
{
	int mSpeed = motor_speed;
	if (mSpeed < 0)
		mSpeed = -mSpeed;
	
	
	if (motor_speed >= 0) // if input motor speed is greater than 0, motor is turning forwards.
	{
		// RIGHT MOTOR - FORWARD DIRECTION
		PORTA &= ~(1<<0); 
		PORTA |= (1<<1); 
	}
	else
	{
		// RIGHT MOTOR - REVERSE  DIRECTION
		PORTA |= (1<<0); 
		PORTA &= ~(1<<1);
	}
	// if motor_speed is greater than 126, as discussed during the lecture, the motor will be turning at maximum speed
	if (mSpeed >= 126) 
	{
		OCR1A = 9999; 
	}	
	else // if not, transform the input value to a pwm-suitable range and set the PWM accordingly
	{
		OCR1A = map(mSpeed, 0, 125, 0, 9999);
	}
}
