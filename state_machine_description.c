// sensor calibration

// wall difference threshold - constant defined that is the minimum value causing the robot to adjust its direction.
// max wall width - this is the maximum difference that can be detected by the sensors; I.E. if the robot is between two walls, the maximum distance is Width of Hall - Width of Car;
//


autonomous_state current_autonomous_state;

//scan all walls all the time - every 50 ms

if (autonomous)
{
	if (millis() - last_difference_check > 50)
	{
		int left_right_difference = get_left_right_distance();
		last_difference_check = millis();
	}
		
	switch(current_autonomous_state)
	{
		//state stopped
		case stopped:
		break;
		
		//state forward
		// it will only do small direction adjustments if there is room to go forward. 
		// If there is no room to go forward, it will turn fully
		case forward:
		move_forward();
		int left_right_abs = left_right_difference;
		if (left_right_difference < 0)
		{
			left_right_abs = -left_right_difference;
		}
		
		if (front < MIN_DISTANCE_FRONT)  // if range sensor reports that the vehicle is too close to the front wall
		{
			if(left_right_difference < 0) // if left < right, there is a bigger gap on the right, turn right
			{
				current_autonomous_state = turning_right;
				degrees = 90;
			}
			else 
			{
				current_autonomous_state = turning_left; // turn left.
				degrees = 90;
			}
		}
		
		else if (millis() - time_last_adjusted > 1000) // if last time an adjustment was checked for was more than 100 ms ago, we can check again
		{
			time_last_adjusted = millis();		
			if (left_right_abs > WALL_DIFFERENCE_THRESHOLD && left_right_abs < MAX_WALL_WIDTH) // if there is enough distance in front of the robot, but it's closer to one wall than the other
			{
				// adjust direction based on how big the difference is
				if (left_right_difference < 0 ) // if range sensor reports there is a bigger gap on the right
				{
					current_autonomous_state = turning_right;
					degrees = 10;
				}
				
				else // if range sensor reports there is a larger gap on the left 
				{
					current_autonomous_state = turning_left;
					degrees = 10;
				}
			}
			
		}
		
		// if neither of these conditions are satisfied, vehicle must be going in a straight line;
		// BUT IF the front distance isn't decreasing, something's wrong; i.e. we're stuck
		// set state to error; maybe make it reverse or turn 45 degrees one way or another;
		
		break;

		//state turning_left
		case turning_left:
		turn_left(degrees);
		current_autonomous_state = forward;
		break;
		
		//state turning_right
		case turning_right:
		turn_right(degrees);
		current_autonomous_state = forward;
		break;

		//state error
		case error:
		
		current_autonomous_state = forward;
		break;
	}
} 
else // not in autonomous mode
{

}