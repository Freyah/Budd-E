/*
	
	File: 		robot_unit.ino
	Board:		Arduino Mega (ATMega2560) 
	Author: 	Daniel Busan & Freya Heeks
	Created:	May 2015
	Version: 	1.7.1
	
	File containing the source code for the Robot unit 
	of the 2nd Year Mechatronics Microcontroller Final Project
	
	This unit handles receiving PWM inputs from the Controller unit's 2 joysticks
	via the Xbee modules, through the Serial2 connections. 
	It also sends the Analog range sensor data back to the controller unit via the Serial lines.
	Camera control is also included.
	
	Autonomous decision making is also handled in this source file.
	
*/

#include "motor_directions.h"
#include "autonomous.h"

#define ADJUSTING_THRESHOLD 25 
#define MAX_WALL_WIDTH 500
#define MIN_DISTANCE_FRONT 150
#define _DEG_TURN_TIME 1000
#define ADJUSTING_PERIOD 250


byte new_f_comp	= 	0;	// Variable containing the newly received value for forwards component 
byte new_r_comp	= 	0; 	// Variable containing the newly received value for right component
byte f_comp = 	0; 		// Variable that stores the forwards component value, used throughout the code
byte r_comp = 	0; 		// Ditto^, but for right_component

byte new_servo_pos = 0; 		// byte in for servo
byte servo_pos = 0; 			// byte to store which is used thru the code
byte new_autonomous_state = 0; 	// byte to store the byte telling the code if robot is in autonomous mode or not

bool autonomous = true;
bool is_adjusting_direction = false;
unsigned long int time_last_adjusted = 0L;



autonomous_state current_autonomous_state = forward;

byte in_buffer[6] = {0, 0, 0, 0, 0, 0};
byte serial_byte_out[6] = {0, 0, 0, 0, 0, 0};

int left_motor = 0;
int right_motor = 0;


bool turning = false;
bool servo_rotating = false;
unsigned long int servo_timer = 0L;

unsigned int range_sensor_value[3]; // array that stores the range sensor readings

// Variables holding times for different comparisons
// Units are milliseconds.
unsigned long int last_time_sent = 0L; 
unsigned long int last_difference_check = 0L;
unsigned long int turning_time = 0L;

int left_right_difference = 0;
int left_right_abs = 0;

void timer1_init(void);
void timer3_init(void);

int get_left_right_distance(void);

void setup()
{
	Serial1.begin(9600);
	Serial.begin(9600);
	DDRC = (1<<PC6); // Enable port c
	PORTC &= ~(1<<PC6); // turn oFf autonomous LED status 
   	
    DDRA = (1<<A0)|(1<<A1)|(1<<A2)|(1<<A3); // the 4 ports controlling the H bridge
    
	timer1_init(); 
	timer3_init();

}

void loop()
{
	
  
	if(Serial1.available() >= 6) // there's 6 bits in the buffer
	{
		Serial1.readBytes(in_buffer, 6);
		for (int i=0;i<6; i++)
		{
			switch (i)
			{
				case 0:
				if (in_buffer[i] != 255) // first byte should be 255
				{
					i = 6; 
					Serial.print("Error in packet transmission. First byte is not 255");
				}
				
				break;
				
				case 1:
				new_r_comp= in_buffer[i]; // store in new_f_comp
				break;
				
				case 2:
				new_f_comp = in_buffer[i]; // store in new_r_comp;
				break;
				
				case 3:
				new_servo_pos = in_buffer[i]; // store camera_servo position
				break;
				
				case 4:
				new_autonomous_state = in_buffer[i]; // store autonomous_state in variable new_autonomous_state
                                
				break;
				
				case 5:
				if (in_buffer[i] == 254) // if stop byte is encountered
				{
					// transfer the values received in the packets to the variable that will be used thru the rest of the code
					f_comp = new_f_comp; 
					r_comp = new_r_comp;
					servo_pos = new_servo_pos;
					
					// bit masking didn't seem to work for the following section
					if (new_autonomous_state == 1 || new_autonomous_state == 3) // if LSB of packet 4 is 1 --> autonomous is on
					{
						autonomous = true;

					}
					else
					{
						autonomous = false;
					}
					
					if (new_autonomous_state == 2 || new_autonomous_state == 3) // if 2nd LSB of packet 4 is 1 --> camera rotation is on
					{
						servo_rotating = true;

					}
					else
					{
						servo_rotating = false;
					}
					
				}
				
				if (in_buffer[i] == 255) // if it's the start/reset byte we missed some bytes
				{
					Serial.print("Start-Reset Byte encountered, values not transferred.\n");
				}
				break;	
			}
		}
	}

	
	range_sensor_value[0] = analogRead(2); // LEFT SIDE SENSOR  - SHORT RANGE
	range_sensor_value[1] = analogRead(1); // RIGhT side sensor - SHORT RANGE
	range_sensor_value[2] = analogRead(3); // Front SIde Sensor - LONG RANGE
	
	
	// SENDING THE PACKETS
	if (millis()-last_time_sent>=100) // if 100 mS have passed since last serial byte sent, do it again
	{
		serial_byte_out[0] = 255;
		// converts data to be sent; from 10bit -> 8bit-2
        serial_byte_out[1] = map(range_sensor_value[0], 0, 1023, 0, 253);
        serial_byte_out[2] = map(range_sensor_value[1], 0, 1023, 0, 253);
        serial_byte_out[3] = map(range_sensor_value[2], 0, 1023, 0, 253);
        serial_byte_out[4] = 0; // not used
        serial_byte_out[5] = 254;
                
		Serial1.write(serial_byte_out, 6); // Send the 6 byte packet
				
		last_time_sent = millis();
	}
	
	if (servo_rotating)
	{
		OCR3A = 1470; // servo slowly scanning
	} 
	// only stop the servo from rotating when the button joystick button is pressed if 
	// servo_rotating is FALSE AND if the robot is in autonomous mode only.
	// This prevents stuttering of the servo when robot is in manual mode and automatic
	// camera rotation is disabled.
    else if (autonomous && !servo_rotating) 
    {
		OCR3A = 1500; // servo stopped 
	}

	// AUTONOMOUS MODE SECTION
	if (autonomous)
	{

		int left = (30587.0/range_sensor_value[0] - 27.303); // left sensor
		int right = (35484.0/range_sensor_value[1] - 33.711); // right sensor
		int front = (70341.0/range_sensor_value[2] - 58.94); // LONG RANGE SENSOR
		
		PORTC |= (1<<PC6); // turn on status LED
		
		// calculate left-right difference only once every 50 milliseconds
		if (millis() - last_difference_check > 50) 
		{
			left_right_difference = get_left_right_distance();
			last_difference_check = millis();
		}
		
		// finite state machine for autonomy
		switch(current_autonomous_state)
		{

			
			// DEFAULT
			// state forward
			// it will only do small direction adjustments if there is room to go forward. 
			// If there is no room to go forward, it will turn fully
			case forward:
			
			// if robot wasn't previously in a turning state; Move forward
			if (!turning) move_forward(); 
            
			// store absolute value of the left_right_difference in left_right_abs
			// this is used in priority 2
			left_right_abs = left_right_difference;
			if (left_right_difference < 0)
			{
				left_right_abs = -left_right_difference;
			}
			
			/*
				PRIORITY #1: Front distance
				If the distance in front of the robot is less 
				than the minimum set distance, the robot will
				turn.
			*/
			if (front < MIN_DISTANCE_FRONT)  // if range sensor reports that the vehicle is too close to the front wall
			{	
				turning = true;
  				//Serial.print(" Need to turn "); Serial.print(left_right_difference); Serial.print("\n"); // !DEBUG
				
				if(left_right_difference < 0) // if left < right, there is a bigger gap on the right, turn right
				{
					//Serial.print(left_right_difference); Serial.print(" Turning Right\n"); // !DEBUG
					current_autonomous_state = turning_right;
					turning_time = millis(); // used when turning
				}
				else 
				{
					//Serial.print(left_right_difference); Serial.print(" Turning Left\n"); // !DEBUG
					current_autonomous_state = turning_left; 
					turning_time = millis(); // used when turning
				} 
			} 
			
			/*
				PRIORITY #2: ADJUSTMENT
				If the distance in front of the robot is greater than the minimum
				set in the MIN_DISTANCE_FRONT constant, the robot checks whether
				it needs to adjust its direction according to the distance to the 
				left and right wall
			*/
			else
			{
				// if vehicle is adjusting, it will not do a full 90 degree turn
				// therefore turning is set to FALSE;
				turning = false; 
				
				// only adjust direction every ADJUSTING_PERIOD ms
				if (millis() - time_last_adjusted > ADJUSTING_PERIOD)
				{
					// Adjustment will occur if the magnitude of the left-right difference
					// is greater than the ADJUSTING_THRESHOLD AND if the robot is between two walls
					// i.e. if the magnitude of the difference is less than the HALL WIDTH - robot width
					if ((left_right_abs > ADJUSTING_THRESHOLD) && (left_right_abs < MAX_WALL_WIDTH))
					{
						current_autonomous_state = adjusting; 
						turning_time = millis(); // used when adjusting
					    // Serial.print(" Adjusting \n"); // !DEBUG
					}
				}
			}
			
			break;
			
			

			//Turn left 90 degrees
			case turning_left:
			
			
			if(millis() - turning_time < _DEG_TURN_TIME) // if it's been less than _DEG_TURN_TIME ms since a corner was encountered, keep turning left
			{
				turn_left(); 
				turning = true;
				if (front > MIN_DISTANCE_FRONT) // unless there is room to move forwards.
				{
					current_autonomous_state = forward;
				}
			}
			else // if it's been more than _DEG_TURN_TIME ms and robot has been turning since, start moving forwards.
			{
				turning = false; // set turning to FALSE as the robot has finished turning
				current_autonomous_state = forward;
			}
			
			break;
			
			//Turn right 90 degrees
			case turning_right:
			
			if(millis() - turning_time < _DEG_TURN_TIME) // if it's been less than _DEG_TURN_TIME ms since a corner was encountered, keep turning right
			{
				turn_right(); 
				turning = true;
                if (front > MIN_DISTANCE_FRONT) // unless there is room to move forwards
				{
					current_autonomous_state = forward;
				}	
			}
			else // if it's been more than _DEG_TURN_TIME ms and robot has been turning since, start moving forwards.
		       {
				turning = false; // set turning to FALSE as the robot has finished turning
				current_autonomous_state = forward;
			}
			break;

			// Adjusting state
			
			case adjusting:
			// if, by any chance the value of left-right is greater than 450 mm when adjusting
			// set it to 450
			if (left_right_abs > 450)  
			{
				left_right_abs = 450;
			}
			// Robot adjusts if an adjustment ocurred more than ADJUSTING_PERIOD ago
			// and if the robot was already adjusting, it continues to adjust for (left_right_abs + 120) ms -- 
			// This adjustment value was decided upon after extensive testing.
			if ((millis() - time_last_adjusted > ADJUSTING_PERIOD) && (millis() - turning_time < ((left_right_abs+120)) )) 
			{
				if (left_right_difference < 0)
				{
					// Serial.print("\t Adj Right\n"); // !DEBUG
					adjust_right();
				}
				else
				{
					// Serial.print("\t Adj Left\n"); // !DEBUG                                   
					adjust_left();
				}
				current_autonomous_state = adjusting;
			}
			// if it's been adjusting for more than left_right_abs+120 ms 
			// or if it adjusted less than ADJUSTING_PERIOD ms ago it needs to go forward 
			else 
			{
				time_last_adjusted = millis();
				current_autonomous_state = forward;
			}
			break;

			
			//state stopped
			// not used
			case stopped:
			current_autonomous_state = forward;
			break;
			
			//state error
			// not used
			case error:
			current_autonomous_state = forward;
			break;
		}
		
		
	} 
	else // if the robot is NOT in an autonomous state turn the motors according to the inputs from the packets	
	{
		PORTC &= ~(1<<PC6); // turn off LED
		
		// calculate left and right motor speeds based on packets received
		left_motor = f_comp + r_comp-253; 
		right_motor = f_comp - r_comp;
		
		// set a deadzone of +/-8 for the motors to eliminate any buzzing sounds
		if (left_motor > -8 && left_motor < 8)
		{
			left_motor = 0; 
		}
		if (right_motor > -8 && right_motor < 8)
		{
			right_motor = 0;
		}
		
		set_l_motor(left_motor);
		set_r_motor(right_motor);
	
		// set servo speed based on packets received
		if (!servo_rotating) 
			OCR3A = map(servo_pos, 0, 253, 1450, 1550); // map from 0, 253 to 1450 to 1550
		
		// the 1450 --> 1550 is chosen to ensure the servo is able to be controlled smoothly enough
		// given the possible range of 1000 to 2000, 1450 to 1550 is a small fraction of this.
		
	}

}


void timer1_init(void)
{
	
	TCCR1A = (1<<COM1A1)|(1<<COM1B1); 	//Using mode 8, phase correct
	TCCR1B = (1<<WGM13); 				//normal mode
	

    OCR1A = 0;
    OCR1B = 0;
	ICR1 = 9999; //set the top value for a prescaler 8
	
	DDRB = ((1<<PB5) | (1<<PB6)); // Set PB5 and PB6 to be output
	TCCR1B |= (1<<1); //set prescaler to 8
	
}


// timer for the PWM generated for the Servos: base period 20 ms; min duty cycle period: 1 ms, max duty cycle period 2 ms
void timer3_init(void)
{
	TCCR3A = (1<<COM3A1)|(1<<COM3B1); //Using mode 8, phase correct
	TCCR3B = (1<<WGM33); //normal mode
	
	DDRE |= (1<<PE3); // Servo is connected to port E3

	OCR3A = 1500; // PORTE: PE3; DIGITAL PIN 5 - PWM
	ICR3 = 19999;
	
	TCCR3B |= (1<<1); // set prescaler to 8
	
}


// RETURNS difference between left and right sensor in MM
int get_left_right_distance(void)
{
	
	return (int)((30587.0/range_sensor_value[0] - 27.303) - (35484.0/range_sensor_value[1]-33.711)); // in MM
	
}
