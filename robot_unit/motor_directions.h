/*
	
	File: 		motor_directions.h
	Author: 	Daniel Busan
	Created:	May 2015
	Version: 	1.7.1
	
	Header file for the commands that handle the motor directions for the vehicle.
*/

/*
	Set motor direction forwards on backwards
*/


#ifndef _motor_directions_h
#define _motor_directions_h


#ifdef __cplusplus
extern "C" {
#endif

long map(long x, long in_min, long in_max, long out_min, long out_max);

void set_l_motor(int motor_speed);
void set_r_motor(int motor_speed);

#ifdef __cplusplus
}
#endif

#endif
