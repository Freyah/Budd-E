#include "autonomous.h"
#include "motor_directions.h"
#include "Arduino.h"


// Forward

// OCR1A = right motor
// OCR1B = left motor
void move_forward(void) 
{
	OCR1A = 7000;

	// RIGHT MOTOR - FORWARDS DIRECTION
	PORTA &= ~(1<<0);  
	PORTA |= (1<<1);  

	// LEFT MOTOR - FORWARDS DIRECTION
	OCR1B = 7000;
	PORTA |= (1<<2); 
	PORTA &= ~(1<<3);
}

void turn_left()
{
	
	// RIGHT MOTOR - FORWARDS DIRECTION
	PORTA &= ~(1<<0); 
	PORTA |= (1<<1); 
	OCR1A = 9999; // RIGHT MOTOR MAX
	
	// LEFT MOTOR - REVERSE DIRECTION
	PORTA |= (1<<3); 
	PORTA &= ~(1<<2); 
	
	OCR1B = 1700; // LEFT MOTOR AT 17% DUTY CYCLE REVERSE
	
}


void turn_right()
{
	// RIGHT MOTOR - REVERSE DIRECTION
	PORTA |= (1<<0); 
	PORTA &= ~(1<<1); 
	OCR1A = 1700;
	
	// LEFT MOTOR - FORWARD DIRECTION
	PORTA &= ~(1<<3);
	PORTA |= (1<<2); 
	OCR1B = 9999;
}

void adjust_left()
{

	// RIGHT MOTOR - FORWARD DIRECTION
	PORTA &= ~(1<<0); 
	PORTA |= (1<<1); 
	
	// LEFT MOTOR - FORWARD DIRECTION
	PORTA &= ~(1<<3); 
	PORTA |= (1<<2); 
	
	OCR1B = 0; OCR1A = 9999; // RIGHT MOTOR MAX, LEFT MOTOR OFF
}


void adjust_right()
{
	
	// RIGHT MOTOR - FORWARD DIRECTION
	PORTA &= ~(1<<0); 
	PORTA |= (1<<1); 
	// LEFT MOTOR - FORWARD DIRECTION
	PORTA &= ~(1<<3); 
	PORTA |= (1<<2); 
	
	OCR1A = 0; OCR1B = 9999;
}

// not used
void reverse(void)
{
    OCR1A = 9999; OCR1B = 9999;
	
    PORTA |= (1<<0); 
    PORTA &= ~(1<<1); 
      
    PORTA &= ~(1<<2); 
    PORTA |= (1<<3);
}

// not used
void stop_motors(void)
{
   OCR1A = 0; OCR1B = 0;
} 

