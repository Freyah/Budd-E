/*
	
	File: 		control_unit.ino
	Board:		Mega (ATMega1280)
	Author: 	Daniel Busan & Freya Heeks
	Created:	May 2015
	Version: 	1.7.1
	
	File containing the source code of the Control unit 
	of the 2nd Year Mechatronics Microcontroller Final Project
	
	This module sends the values read from the joysticks to the robot unit along with the 
	autonomous status and camera rotation status. Transmission is done via XBee modules connected 
	to Serila lines
	It also reads and displays on an LCD the values of the range sensor sent back by the robot.
	
*/

/*
	LCD Setup: LiquidCrystal(rs, enable, d4, d5, d6, d7)
	rs = 46
	e = 47
	d4 = 50
	d5 = 51
	d6 = 52
	d7 = 53
*/
/*
	Protocol description
	byte[0]		255		Start byte
	byte[1]		x		Right joystick Horizontal Axis Value
	byte[2]		x		Right joystick Vertical Axis value
	byte[3]		x		Left joystick horizontal axis value
	byte[4]		x		Camera Rotn + Autonomous toggle: xxxx xx01: LSB auto toggle, 2nd LSB camera toggle
	byte[5]		254		Stop byte
*/

/*
	Autonomous Toggle Button - PD2
*/
#include <LiquidCrystal.h>

LiquidCrystal lcd(46, 47, 50, 51, 52, 53);


byte serial_byte_in[6];
byte serial_byte_out = 0;


byte joyX_pin = 15; // analog pins where joystick is connected
byte joyY_pin = 14;

byte cameraX_pin = 0; // analog pin for the right joystick

byte out_buffer[6] = {255, 0, 0, 0, 0, 254}; // 6 bytes of buffer data


unsigned long int rangeL = 0L;
unsigned long int rangeR = 0L;
unsigned long int rangeF = 0L;

unsigned long int last_sent_ms = 0L;
unsigned long int ms_last_check = 0L;
unsigned long int ms_last_check_camera = 0L;

// stores values read from the left joystick
unsigned int joyX = 0;
unsigned int joyY = 0;
// stores values read from the x-axis of the right joystick
unsigned int cameraX = 0;

unsigned int range_left = 0;
unsigned int range_right = 0;
unsigned int range_front = 0;

unsigned int new_range_left = 0;
unsigned int new_range_right = 0;
unsigned int new_range_front = 0;

byte previous_button_state = 1; // set to 1; high; button wasn't pressed therefore, previous initial state is 1;
byte previous_camera_button_state = 1; // high ; button not pressed before
// BUTTON for camera rotation set to Digital 22 PD1

// holds fsm state for the receiving protocol
unsigned int fsm_state = 5;


bool is_in_autonomous_mode = false ; //to toggle between autonomous mode
bool camera_is_rotating = false; // to toggle the camera rotation
bool updated = false;



void setup()
{
	DDRD &= ~(1<<PD2); DDRD &= ~(1<<PD1);
	PORTD |=(1<<PD2) | (1<<PD1); //turn on pull up resistor       
        
	lcd.begin(16,2);
	lcd.home();
	Serial.begin(9600);
	Serial2.begin(9600);
}

void loop()
{  
	if(Serial2.available()>= 6)
	{
        Serial2.readBytes(serial_byte_in, 6); // read byte in buffer
		if (serial_byte_in[0] == 255) // if first byte is 255 (good)
		{
			for (fsm_state = 1; fsm_state < 6; fsm++)
			{
				switch (fsm_state)
				{

					case 1:
					new_range_left = serial_byte_in[1];
					break;

					case 2:
					
					new_range_right = serial_byte_in[2];
					break;

					case 3:
					
					new_range_front = serial_byte_in[3];
					break;

					case 4:
					break;

					case 5:
					if (serial_byte_in[5] == 254) //if stop byte is encountered transfer values to variables
					{
						 range_left = new_range_left;
						 range_right = new_range_right;
						 range_front = new_range_front;
						 updated = true;
					}
					break; 
				}
			}
			
		}
		else // discard bogus data 
		{
			serial_byte_in[1] = 0;
			serial_byte_in[2] = 0;
			serial_byte_in[3] = 0;
			serial_byte_in[4] = 0;
			//Serial.print("Discarding bogus data \n"); // !DEBUG
			updated = false;
		}
	}
	
	// Read the buttons
	
	// Camera Rotation Toggle - Software Debounce
	if(!(PIND & (1<<PD1))) // read PD1; if button has been pressed PD0 will be 0; Camera rotation toggle; ROBOT CONTROL JOY BUTTON
	{		
        if(millis() - ms_last_check_camera >= 10)
		if (previous_camera_button_state == 1)
		{
			ms_last_check_camera = millis();
			//Serial.println("\tCamera toggled"); // !DEBUG!
			// camera_is_rotating = !camera_is_rotating;
			out_buffer[4] ^= (1<<1); // starts at 0; should toggle back and forth
			//Serial.println(out_buffer[4]);
		}
		previous_camera_button_state = 0;
	} 
	else 
	{
		previous_camera_button_state = 1; // if 10 ms have passed and button isn't pressed, previous state is set to UP/HIGH/NOT PRESSED
	}
	
	// Autonomous Toggle - Software Debounce
	if(!(PIND & (1<<PD2))) // read PD2; if button has been pressed PD2 will be 0; AUtonomous mode toggle
	{		
         if(millis() - ms_last_check >= 10)
		if (previous_button_state == 1)
		{
			ms_last_check = millis();
			//Serial.println("\tToggled autonomous mode"); // !DEBUG!
			out_buffer[4] ^= (1<<0); // initially at 0; should toggle back and forth
		}
		previous_button_state = 0;
	} 
	else 
	{
		previous_button_state = 1; // if 10 ms have passed and button isn't pressed, previous state is set to UP/HIGH/NOT PRESSED
	}
	
	// out_buffer[4] looks like this |_|_|_|_| |_|_|x|x| where LSB and 2nd LSB represent autonomous state and camera rotation state 
	

	cameraX = analogRead(cameraX_pin); // 
	joyX = analogRead(joyX_pin); // read x axis of right joystick
	joyY = analogRead(joyY_pin); // read y axis of right joystick
	
	
	out_buffer[1] = map(joyX, 0, 1023, 0, 253); // use arduino in built map function
	out_buffer[2] = map(joyY, 0, 1023, 253, 0);
	out_buffer[3] = map(cameraX, 0, 1023, 253, 0); // servo pwm control
    
	// In this statement, both the sending of packets and LCD printing are handled simultaneously
	if (millis() - last_sent_ms>50) 
	{
		lcd.home();

		// LCD will display "Auto Engaged C:1" or "Auto Engaged C:0" if autonomous mode is engaged 
		// and camera rotation is on or off.
		// LCD displays the x and y value of the left joystick if autonomous is off
		if (out_buffer[4] & (1<<0))
		{
			lcd.print("Auto Engaged;"); lcd.print("C:");lcd.print(out_buffer[4] & (1<<1)); // should print camera rotation status
		}
		else
		{
			lcd.print("x: "); lcd.print(out_buffer[1]); lcd.print("  ");
			lcd.print("y: "); lcd.print(out_buffer[2]); lcd.print("  ");
		}
		
		rangeF = (70341.0/map(range_front, 0, 253, 0, 1023) - 58.94)/10.0; // front LONG RANGE SENSOR
		rangeL = (30587.0/map(range_left, 0, 253, 0, 1023) - 27.303)/10.0; // left sensor
		rangeR = (35484.0/map(range_right, 0, 253, 0, 1023) - 33.711)/10.0; // right sensor
		
		if (updated) // if values have been read properly from the 6 bytes display them on the bottom row
		{
			lcd.setCursor(0,1);
			lcd.print("l "); lcd.print(rangeL); lcd.print(" ");
			lcd.print("r "); lcd.print(rangeR); lcd.print(" ");
			lcd.print("f "); lcd.print(rangeF); lcd.print("    ");
		}
			
		Serial2.write(out_buffer, 6); // send the 6-byte packet via Serial2
		last_sent_ms = millis(); // update the last_sent_ms
	}      
}
