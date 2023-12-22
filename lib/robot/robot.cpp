#include "robot.h"

// Instantiate objects for the other classes
Motor my_motors;
// Motor left_motor;
// Motor right_motor;
Sensors my_sensors;
// Map objects to map the maze
Map my_map(15, 4);


/**
 * @brief Function used to test various features. Any code implementation is often temporary
 * But may be promoted up to its own function at later use.
 */
void Robot::test()
{
	// Serial.println("\n\nLeft Encoder amount:");
	// Serial.println(my_motors.get_encoder_revolutions_left());
	// Serial.println("\nRight Encoder amount:");
	// Serial.println(my_motors.get_encoder_revolutions_right());

	// Serial.println("\n\nLeft Wheel rotations");
	// Serial.println(my_motors.get_wheel_rotations_left());
	// Serial.println("\nRight Wheel rotations");
	// Serial.println(my_motors.get_wheel_rotations_right());

	// Serial.println("\n\nDistance moved left wheel");
	// Serial.println(my_motors.calculate_distance_by_wheel_rotations_left());
	// Serial.println("\nDistance moved right wheel");
	// Serial.println(my_motors.calculate_distance_by_wheel_rotations_right());

	// Serial.println("\n\nACTUAL DISTANCE LEFT");
	// Serial.println(my_motors.get_distance_travelled_left());
	// Serial.println("\nACTUAL DISTANCE RIGHT");
	// Serial.println(my_motors.get_distance_travelled_right());

	this->drive_backwards();
	wait_us(100000);
}



/**
 * @brief Run once upon powering on the robot.
 * Sets up sections of robot which only ever need to be set once
 */
void Robot::initial_setup()
{
    // Calibrate the motors for use
	my_motors.setup();

	// Set the default direction for the robot to move in
	my_motors.set_direction(my_motors.DIR_FORWARDS);

	// Attach the interrupts for the encoders on the motors
	my_motors.attach_encoder_interrupts();

	// Setup the occupancy grid array to initialise the maze prior to the robot moving
	my_map.setup_occupancy_grid();
	
	//my_robot.calculate_starting_location();

	// Initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(LEDR, OUTPUT);
	pinMode(LEDG, OUTPUT);
	pinMode(LEDB, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, HIGH);
	digitalWrite(LEDB, HIGH);
}

/**
 * @brief Run once the robot is placed at the starting position in the maze
 * Is a state within the state machine, can be called again to restart the robots algorithms
 */
void Robot::setup()
{
	this->calculate_starting_location();
	this->initialise_starting_location_in_map();
    this->centre_on_map_grid();


	if (this->check_side_space_left(1))
	{
		digitalWrite(LEDR, LOW);
		digitalWrite(LEDG, LOW);
		digitalWrite(LEDB, LOW);

		this->rotate_robot(-90);
		while (this->check_route_ahead(-1))
		{
			this->drive_forwards();
		}
		this->rotate_robot(90);
	}

	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, HIGH);
	digitalWrite(LEDB, HIGH);

	// this->move_robot(1.0f);
	// wait_us(4000000);

	this->current_state = this->STATE_SOLVE;
	



	// this->move_robot(20.0f);
	// this->rotate_robot(90);
	// this->move_robot(20.0f);
}

/**
 * @brief Robot starts solving the maze and finding its way to the end
 * Runs all algorithms for solving and mapping from this function
 * Part of state SOLVE within the state machine
 */
void Robot::solve_maze()
{
	algorithm = DIRECT_AND_AROUND;

	bool can_move_forwards = false;
	bool reverse = false;

	if (algorithm == DIRECT_AND_AROUND)
	{
		can_move_forwards = this->check_route_ahead(-1.0f);

		if (can_move_forwards)
		{
			this->drive_forwards();
			initial_distance_moved_left = my_motors.get_distance_travelled_left();
			initial_distance_moved_right = my_motors.get_distance_travelled_right();
			digitalWrite(LEDG, LOW);
			digitalWrite(LEDR, HIGH);
			digitalWrite(LEDB, HIGH);
		}
		else
		{
			this->stop_moving();
			determine_new_distance_moved();

			if (this->check_side_space_left(1))
			{
				this->rotate_robot(-90);
				digitalWrite(LEDB, LOW);
				digitalWrite(LEDR, HIGH);
				digitalWrite(LEDG, HIGH);
			}
			else if (this->check_side_space_right(1))
			{
				this->rotate_robot(90);
				digitalWrite(LEDB, LOW);
				digitalWrite(LEDR, HIGH);
				digitalWrite(LEDG, HIGH);
			}
			else
			{
				reverse = true;
				while (reverse)
				{
					this->drive_backwards();
					digitalWrite(LEDR, LOW);
					digitalWrite(LEDG, HIGH);
					digitalWrite(LEDB, HIGH);
					wait_us(10000);
					// if (my_sensors.get_back_IR_distance() < MIN_IR_DIST_REAR)
					// {
					// 	reverse = false;
					// 	this->stop_moving();
					// 	digitalWrite(LEDR, LOW);
					// 	digitalWrite(LEDG, LOW);
					// 	digitalWrite(LEDB, LOW);
					// }

					if (this->check_side_space_left(1))
					{
						reverse = false;
						this->rotate_robot(-90);
					}
					else if (this->check_side_space_right(1))
					{
						reverse = false;
						this->rotate_robot(90);
					}
					
				}
			}
		}
	}
	else if (algorithm == FOLLOW_WALL)
	{
		bool reached_next_wall = false;

		while (!this->check_side_space_left(1))
		{	
			reached_next_wall = false;
			//this->drive_forwards();
			if (this->check_route_ahead(-1))
			{
				digitalWrite(LEDG, LOW);
				digitalWrite(LEDR, HIGH);
				digitalWrite(LEDB, HIGH);
				Serial.println("Free space ahead");
				Serial.println("Driving forwards\n\n");
				this->drive_forwards();
			}
			else
			{
				if (!this->check_side_space_left(1))
				{
					digitalWrite(LEDR, LOW);
					digitalWrite(LEDG, HIGH);
					digitalWrite(LEDB, HIGH);
					Serial.println("No space ahead and wall on left");
					Serial.println("Turning Right\n\n");
					this->rotate_robot(90);
					this->drive_forwards();
				}
			}
		}

		if (this->check_side_space_left(1))
		{
			digitalWrite(LEDB, LOW);
			digitalWrite(LEDR, HIGH);
			digitalWrite(LEDG, HIGH);
			Serial.println("Reached end of left wall");
			Serial.println("Turning left to continue along next wall\n\n");
			
			this->move_robot(11.0f);
			this->rotate_robot(-90);
			this->move_robot(11.0f);

			// while (!reached_next_wall)
			// {
			// 	this->drive_forwards();
			// 	if (!this->check_route_ahead(-1))
			// 	{
			// 		this->stop_moving();
			// 		break;
			// 	}
			// 	if (!this->check_side_space_left(1))
			// 	{
			// 		reached_next_wall = true;
			// 		this->stop_moving();
			// 		wait_us(500000);
			// 	}
			// }

			// this->move_robot(80.0f);
			// this->rotate_robot(-90);
			// this->move_robot(80.0f);
		}
	}
	else if (algorithm == NAVIGATE_MAP)
	{

	}
}

/**
 * @brief Starts the robot driving forwards at its default speed of 0.5
 */
void Robot::drive_forwards()
{
	my_motors.set_direction(my_motors.DIR_FORWARDS);
	my_motors.set_speed(DEFAULT_ROBOT_SPEED);
	my_motors.drive(0);
}

/**
 * @brief Starts the robot driving backwards at its default speed of 0.5
 */
void Robot::drive_backwards()
{
	my_motors.set_direction(my_motors.DIR_BACKWARDS);
	my_motors.set_speed(DEFAULT_ROBOT_SPEED);
	my_motors.drive(0);
}

/**
 * @brief Stops the robot from moving
 */
void Robot::stop_moving()
{
	my_motors.stop_driving();
	//this->current_state = this->STATE_STOP;
}

/**
 * @brief Ends all processes on the robot and stops the state machine
 */
void Robot::end()
{
    this->stop_moving();
}

/**
 * @brief Moves the robot a given distance, if the value given is negative the robot will move backwards
 * 
 * @param distance_to_move 	The number of mm to move the robot
 */
void Robot::move_robot(float distance_to_move)
{
    float initial_distance_moved_left = 0.0f;
	float initial_distance_moved_right = 0.0f;
	float distance_moved_left = 0.0f;
	float distance_moved_right = 0.0f;
	float difference_between_motor_distances = 0.0f;

	bool route_ahead_free = false;
	bool reached_distance = false;

	int loop_count = 0;


	// Stop robot moving
	my_motors.stop_driving();
	
	// Check that space ahead the robot is wanting to move is free
	route_ahead_free = this->check_route_ahead(distance_to_move);
    //my_map.check_route_ahead_in_map(this->bearing, distance_to_move);

	// If route ahead is not clear, return and don't move forwards.
	if (route_ahead_free == false)
	{
		//throw ErrorFlag("Could not move forwards as space was not free", true);
		return;
	}

    // Call direction change function on motors depending on the angle direction
	// positive = forwards
	// negative = backwards
	if (distance_to_move > 0)
	{
		my_motors.set_direction(my_motors.DIR_FORWARDS);
	}
	else if (distance_to_move < 0)
	{
		// Since the distance is negative, make it positive for using in the calculation later
		distance_to_move = distance_to_move * -1.0f;
		my_motors.set_direction(my_motors.DIR_BACKWARDS);
	}
	else
	{
		return;
	}

    // Calculate current distance the robot has moved before the robot starts moving the set amount
	initial_distance_moved_left = my_motors.get_distance_travelled_left();
	initial_distance_moved_right = my_motors.get_distance_travelled_right();

    // Start robot moving
	my_motors.set_speed(DEFAULT_ROBOT_SPEED);
	my_motors.drive(0);


	// Loop until robot has moved the distance specified
	// Loops until the sum of the distances moved by both motors is equal to or greater than
	// the set distance to move multiplied by two since the sum of the two wheels is taken.
    do 
	{
		// Get left wheel distance moved whilst turning robot
		distance_moved_left = my_motors.get_distance_travelled_left() - initial_distance_moved_left;

		// Get right wheel distance moved whilst turning robot
		distance_moved_right = my_motors.get_distance_travelled_right() - initial_distance_moved_right;

		// Get sum/difference between both motors
		difference_between_motor_distances = distance_moved_left + distance_moved_right;

		// Delay to ensure polling of interrupt is not backlogged
		wait_us(100);

		if (fabs(difference_between_motor_distances) >= (distance_to_move * 2.0f))
		{
			reached_distance = true;
		}
		loop_count++;
		

	} while (!reached_distance);

	Serial.println(loop_count);

	// // Loop until robot has moved the distance specified
	// // Loops until the sum of the distances moved by both motors is equal to or greater than
	// // the set distance to move multiplied by two since the sum of the two wheels is taken.
    // while (fabs(difference_between_motor_distances) < (distance_to_move * 2.0f))
	// {
	// 	// Get left wheel distance moved whilst turning robot
	// 	distance_moved_left = my_motors.get_distance_travelled_left() - initial_distance_moved_left;

	// 	// Get right wheel distance moved whilst turning robot
	// 	distance_moved_right = my_motors.get_distance_travelled_right() - initial_distance_moved_right;

	// 	// Get sum/difference between both motors
	// 	difference_between_motor_distances = distance_moved_left + distance_moved_right;

	// 	// Delay to ensure polling of interrupt is not backlogged
	// 	wait_us(100);
	// }

    // Stop robot moving and reset direction to forwards
	my_motors.stop_driving();
    my_motors.set_direction(my_motors.DIR_FORWARDS);
}

/**
 * @brief Rotates the robot about a point a set number of degrees
 *
 * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
 */
void Robot::rotate_robot(int degrees)
{
	float initial_distance_moved_left = 0.0f;
	float initial_distance_moved_right = 0.0f;
	float distance_moved_while_turning_left = 0.0f;
	float distance_moved_while_turning_right = 0.0f;
	float difference_between_motor_distances = 0.0f;

	// Stop robot moving
	my_motors.stop_driving();
    this->update_bearing(degrees);

	// Call direction change function on motors depending on the angle direction
	if (degrees > 0)
	{
		my_motors.set_direction(my_motors.DIR_CLOCKWISE);
	}
	else if (degrees < 0)
	{
		degrees = degrees * -1;
		my_motors.set_direction(my_motors.DIR_ANTICLOCKWISE);
	}
	else
	{
		return;
	}

	// Calculate current distance the robot has moved before the robot starts turning the set amount
	initial_distance_moved_left = my_motors.get_distance_travelled_left();
	initial_distance_moved_right = my_motors.get_distance_travelled_right();

	// Calculate equivalent arc of circle
	// Left wheel 80mm out from centre
	// Right wheel 80mm out
	// arc distance = (degrees/360) * 2*pi*radius
	// Distance in mm
	float arc_distance_to_turn = ((float)degrees / 360.0f) * 2.0f * 3.141f * ROBOT_WHEEL_RADIUS;

	// Start robot moving
	my_motors.set_speed(DEFAULT_ROBOT_SPEED);
	my_motors.drive(0);

	// While distance moved < arc length
	// loop
	// Get distance moved by wheels
	// (distance moved could be an average of the two wheels)
	// delay between distance moved polls
	while (fabs(difference_between_motor_distances) < (arc_distance_to_turn * 2.0f))
	{
		// Get left wheel distance moved whilst turning robot
		distance_moved_while_turning_left = my_motors.get_distance_travelled_left() - initial_distance_moved_left;

		// Get right wheel distance moved whilst turning robot
		distance_moved_while_turning_right = my_motors.get_distance_travelled_right() - initial_distance_moved_right;

		// Get difference between both motors
		difference_between_motor_distances = distance_moved_while_turning_left - distance_moved_while_turning_right;

		// Delay to ensure polling of interrupt is not backlogged
		wait_us(100);
	}

	// Stop robot moving and reset direction to forwards
	my_motors.stop_driving();
	my_motors.set_direction(my_motors.DIR_FORWARDS);
}

/**
 * @brief Calculates the starting position of the robot
 * Checks surrounding areas based on sensors
 * And sets the coordinate values based on the sensors
 */
void Robot::calculate_starting_location()
{
	float front_sensor_distance = my_sensors.read_averaged_IR_sensor_front(5);
	float back_sensor_distance = my_sensors.read_averaged_IR_sensor_back(5);
	float left_sensor_distance = my_sensors.read_averaged_usonic_sensor_left(5);
	float right_sensor_distance = my_sensors.read_averaged_usonic_sensor_right(5);

    // Calculate direction robot is facing
    // Assume the robot is facing towards finish from centre of start location

    // Set coordinate position based on sensor readings
    this->current_position.x_coordinate = left_sensor_distance/5.0f;
    this->current_position.y_coordinate = back_sensor_distance/5.0f;

    Serial.println(this->current_position.x_coordinate);
    Serial.println(this->current_position.y_coordinate);
}

/**
 * @brief Inserts the starting location into the occupancy grid
 */
void Robot::initialise_starting_location_in_map()
{
    my_map.set_robot_location(this->current_position.x_coordinate, this->current_position.y_coordinate);
}

/**
 * @brief Centres the robot in the middle of the occupancy grid square
 */
void Robot::centre_on_map_grid()
{
    float x_distance_to_move = 0.0f;
    float y_distance_to_move = 0.0f;
	float coordinate_grid_remainder_x = fmod(this->current_position.x_coordinate, 5);
	float coordinate_grid_remainder_y = fmod(this->current_position.y_coordinate, 5);

    if (coordinate_grid_remainder_x > 2.5f)
    {
        if (this->bearing == 0)
        {
            this->rotate_robot(90);
            this->move_robot(-1.0f * (coordinate_grid_remainder_x - 2.5f));
        }
        else if (this->bearing == 90)
        {
            this->move_robot(-1.0f * (coordinate_grid_remainder_x - 2.5f));
        }
    }
    else if (coordinate_grid_remainder_x < 2.5f)
    {
        if (this->bearing == 0)
        {
            this->rotate_robot(90);
            this->move_robot(coordinate_grid_remainder_x);
        }
        else if (this->bearing == 90)
        {
            this->move_robot(coordinate_grid_remainder_x);
        }
    }
	else
	{
		// In middle of horizontal coordinate, no need to move
	}

    if (coordinate_grid_remainder_y > 2.5f)
    {
        if (this->bearing == 0)
        {
            this->move_robot(-1.0f * (coordinate_grid_remainder_y - 2.5f));
        }
        else if (this->bearing == 90)
        {
            this->rotate_robot(-90);
            this->move_robot(-1.0f * (coordinate_grid_remainder_y - 2.5f));
        }
    }
    else if (coordinate_grid_remainder_y < 2.5f)
    {
        if (this->bearing == 0)
        {
            this->move_robot(coordinate_grid_remainder_y);
        }
        else if (this->bearing == 90)
        {
            this->rotate_robot(-90);
            this->move_robot(coordinate_grid_remainder_y);
        }
    }
}

/**
 * @brief Updates the robots bearing to identify the direction in which the robot is currently heading
 * 
 * @param angle_to_add 	The angle in which the robot is currently turning by
 */
void Robot::update_bearing(int angle_to_add)
{
    if ((this->bearing + angle_to_add) < 360 && (this->bearing + angle_to_add) >= 0)
    {
        this->bearing = this->bearing + angle_to_add;
    }
    else if ((this->bearing + angle_to_add) == 360)
    {
        this->bearing = 0;
    }
    else if ((this->bearing + angle_to_add) > 360)
    {
        this->bearing = (this->bearing + angle_to_add) - 360;
    }
    else if ((this->bearing + angle_to_add) < 0)
    {
        this->bearing = this->bearing + angle_to_add + 360;
    }
}

/**
 * @brief Checks the space in front of the robot to see if there is free space or not
 * Decides whether the robot is able to move forwards the specified distance
 * 
 * @param distance_to_move 	The distance to check whether the robot can move forwards
 * @return true 			True if the robot is allowed to move forwards
 * @return false 			False if there is an object in the way and the robot can't move forwards
 */
bool Robot::check_route_ahead(float distance_to_move)
{
	float distance_to_objects_front = my_sensors.read_averaged_IR_sensor_front(5);

	if (distance_to_move == -1.0f)
	{
		if (distance_to_objects_front > MIN_IR_DIST_FRONT)
		{
			return true;
		}
	}

	if (distance_to_objects_front > (distance_to_move + MIN_IR_DIST_FRONT))
	{
		return true;
	}
	else
	{
		return false;
	}
	return false;
}

/**
 * @brief Checks if there is free space to move into on the left of the robot
 * 
 * @return true 	True if the robot can turn left and move forwards
 * @return false 	False if the robot cannot move left and something is blocking it
 */
bool Robot::check_side_space_left(int num_readings)
{
	int left_sensor_distance = my_sensors.read_averaged_usonic_sensor_left(num_readings);
	
	if (left_sensor_distance > MIN_USONIC_DIST+5.0f)
	{
		return true;
	}
	return false;
}

/**
 * @brief Checks if there is free space to move into on the right of the robot
 * 
 * @return true 	True if the robot can turn right and move forwards
 * @return false 	False if the robot cannot move left and something is blocking it
 */
bool Robot::check_side_space_right(int num_readings)
{
	int right_sensor_distance = my_sensors.read_averaged_usonic_sensor_right(num_readings);

	if (right_sensor_distance > MIN_USONIC_DIST+5.0f)
	{
		return true;
	}
	return false;
}


void Robot::determine_new_distance_moved()
{
	float distance_moved_left = 0.0f;
	float distance_moved_right = 0.0f;
	float average_distance_moved = 0.0f;

	distance_moved_left = my_motors.get_distance_travelled_left() - initial_distance_moved_left;
	distance_moved_right = my_motors.get_distance_travelled_right() - initial_distance_moved_right;

	average_distance_moved = (distance_moved_left + distance_moved_right) / 2.0f;

	Serial.println("\nInitial Coordinates");
	Serial.println("Bearing");
	Serial.println(bearing);
	Serial.println("x coordinate");
	Serial.println(current_position.x_coordinate);
	Serial.println("y coordinate");
	Serial.println(current_position.y_coordinate);

	if (bearing == 0) 
	{
		current_position.y_coordinate = current_position.y_coordinate + average_distance_moved;
	}
	else if (bearing == 90)
	{
		current_position.x_coordinate = current_position.x_coordinate + average_distance_moved;
	}
	else if (bearing == 180)
	{
		current_position.y_coordinate = current_position.y_coordinate - average_distance_moved;
	}
	else if (bearing == 270)
	{
		current_position.x_coordinate = current_position.x_coordinate - average_distance_moved;
	}

	Serial.println("\nFinal Coordinates");
	Serial.println("Bearing");
	Serial.println(bearing);
	Serial.println("x coordinate");
	Serial.println(current_position.x_coordinate);
	Serial.println("y coordinate");
	Serial.println(current_position.y_coordinate);
}











// EXTRA CODE OR OLD CODE OR TEST CODE




// /**
//  * @brief Called from the main arduino loop() function
//  * The main function to run the robot and all its processes.
//  * Is continuously called whilst the robot is set to continue running
//  *
//  * @return true		The robot should continue running its program
//  * @return false	The robot stops running
//  */
// bool Robot::run()
// {
// 	// Run the functions to read values from the infrared sensors
// 	my_sensors.run_IR_sensors();

// 	// wait_us(500000);

// 	// Run the functions to read values from the ultrasonic sensors
// 	my_sensors.run_usonic_sensors();

// 	// Set the speed of the motors to half the max speed
// 	my_motors.set_speed(0.5f);

// 	// Rotate on spot by 45 degrees for two complete rotations
// 	// Detect obstacles within min distance
// 	// Add into occupancy map
// 	// Decide where to move

// 	// Drive the robot forwards
// 	// 0 seconds means continue forever until told otherwise
// 	my_motors.drive(0);

// 	wait_us(1000000);

// 	this->rotate_robot(90);

// 	// Set the speed of the motors to half the max speed
// 	my_motors.set_speed(0.5f);

// 	// Drive the robot forwards
// 	// 0 seconds means continue forever until told otherwise
// 	my_motors.drive(0);

// 	wait_us(1000000);

// 	this->rotate_robot(-90);

// 	// Detect obstacles in the nearby area
// 	// my_robot.detect_obstacle();

// 	// Avoid obstacles in the nearby area by moving away from them
// 	// my_robot.avoid_obstacle();

// 	// Display the occupancy grid in a readable format
// 	// my_robot.display_map();

// 	// Return true to continue running the loop
// 	return true;
// }




// /**
//  * @brief Checks with the sensors whether objects are within
//  * range of the robot
//  *
//  * @return objects - bitfield defining the surrounding objects
//  * 0 - No Objects
//  * 1 - Front Object
//  * 2 - Rear Object
//  * 3 - Left Object
//  * 4 - Right Object
//  * 5 - Front and Rear Object
//  * 6 - Front and Left Object
//  * 7 - Front and Right Object
//  * 8 - Rear and Left Object
//  * 9 - Rear and Right Object
//  * 10 - Left and Right Objects
//  * 11 - Front and Rear and Left Objects
//  * 12 - Front and Rear and Right Objects
//  * 13 - Front and Left and Right Objects
//  * 14 - Rear and Left and Right Objects
//  * 15 - Front and Rear and Left and Right Objects
//  */
// void Robot::detect_obstacle()
// {
// 	if (my_sensors.get_front_IR_distance() < MIN_IR_DIST)
// 	{
// 		if (my_sensors.get_left_usonic_distance() < MIN_USONIC_DIST && my_sensors.get_right_usonic_distance() < MIN_USONIC_DIST)
// 		{
// 			objects = 13;
// 		}
// 		else if (my_sensors.get_left_usonic_distance() < MIN_USONIC_DIST)
// 		{
// 			objects = 6;
// 		}
// 		else if (my_sensors.get_right_usonic_distance() < MIN_USONIC_DIST)
// 		{
// 			objects = 7;
// 		}
// 		else
// 		{
// 			objects = 1;
// 		}
// 	}
// 	else
// 	{
// 		objects = 0;
// 	}
// }

// /**
//  * @brief Tell the robot to avoid the obstacles
//  * that were detected in the detect_obstacle() function
//  *
//  */
// void Robot::avoid_obstacle()
// {
// 	switch (objects)
// 	{
// 	case 0:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		// my_motors.set_speed(1.0f);
// 		break;
// 	case 1:
// 		my_motors.set_direction(my_motors.DIR_ANTICLOCKWISE);
// 		my_motors.set_speed(0.5f);
// 		// my_robot.turn_left();
// 		break;
// 	case 2:
// 		my_motors.set_direction(my_motors.DIR_CLOCKWISE);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 3:
// 		my_motors.set_direction(my_motors.DIR_ANTICLOCKWISE);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 4:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 5:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 6:
// 		my_motors.set_direction(my_motors.DIR_CLOCKWISE);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 7:
// 		my_motors.set_direction(my_motors.DIR_ANTICLOCKWISE);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 8:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 9:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 10:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 11:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 12:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 13:
// 		my_motors.set_direction(my_motors.DIR_BACKWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 14:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	case 15:
// 		my_motors.set_direction(my_motors.DIR_FORWARDS);
// 		my_motors.set_speed(0.5f);
// 		break;
// 	}
// }





/**
 * mbed::callback for class interrupts
 * lambda function to call the class interrupt function
 */

	// switch (current_state)
	// {
	// 	case STATE_STOP:
	// 		my_robot.stop_moving();
	// 		break;
	// 	case STATE_FORWARD:
	// 		break;
	// 	case STATE_BACKWARD:
	// 		break;
	// 	case STATE_LEFT:
	// 		my_robot.turn_left();
	// 		break;
	// 	case STATE_RIGHT:
	// 		my_robot.turn_right();
	// 		break;
	// }


	// // Debug statements for calibrating the encoders to check the distance travelled
	// Serial.println("Distance Travelled Left in mm:");
	// Serial.print(get_distance_travelled_left());
	// Serial.print(" mm\n");
	// Serial.println("Distance Travelled Left in cm:");
	// Serial.print(get_distance_travelled_left() / 10);
	// Serial.print(" cm\n\n");

	// Serial.println("Distance Travelled Right in mm:");
	// Serial.print(get_distance_travelled_right());
	// Serial.print(" mm\n");
	// Serial.println("Distance Travelled Right in cm:");
	// Serial.print(get_distance_travelled_right() / 10);
	// Serial.print(" cm\n\n");

	// Serial.println("==========================================");
	// Serial.println("AVERAGE DISTANCE MOVED BETWEEN BOTH MOTORS");
	// Serial.println(((get_distance_travelled_left() + get_distance_travelled_right()) / 2));
	// Serial.println("==========================================");

	// if (get_encoder_revolutions_left() >= 1000)
	// {
	// 	Serial.println("Final Encoder Count");
	// 	Serial.println(get_encoder_revolutions_left());
	// 	return false;
	// }