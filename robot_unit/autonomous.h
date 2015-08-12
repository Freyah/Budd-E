/*
	
	File:		autonomous.h
	Board:		Arduino Mega (ATMega1280) 
	Author: 	Daniel Busan & Freya Heeks
	Created:	26 May 2015
	
	Version: 	1.7.1
	
	Header file that describes the autonomous library.
	
*/

#ifndef __autonomous_h__
#define __autonomous_h__

// finite state machine
typedef enum {
	stopped, forward, turning_left, turning_right, adjusting, error
} autonomous_state;


#ifdef __cplusplus // required to make external libraries work if they're not .cpp format. source: stackexchange.net
extern "C" {
#endif

// place functions here
void move_forward(void);

void turn_left(void);

void turn_right(void);

void adjust_left(void);

void adjust_right(void);

void reverse(void);

void stop_motors(void);

#ifdef __cplusplus
}
#endif

#endif
