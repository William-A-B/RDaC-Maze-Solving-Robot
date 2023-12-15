#include "robot.h"

// Instantiate objects for the other classes
Motor my_motors;
// Motor left_motor;
// Motor right_motor;
Sensors my_sensors;
// Map objects to map the maze
Map my_map(15, 4);



/**
 * @brief Run once upon powering on the robot.
 * Sets up sections of robot which only ever need to be set once
 */
void Robot::initial_setup()
{
    // Calibrate the motors for use
	my_motors.calibrate();

	// Set the default direction for the robot to move in
	my_motors.set_direction(my_motors.DIR_FORWARDS);

	// Attach the interrupts for the encoders on the motors
	attach_encoder_interrupts();

	// Setup the occupancy grid array to initialise the maze prior to the robot moving
	my_map.setup_occupancy_grid();
	
	//my_robot.calculate_starting_location();
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






	// try
	// {
	// 	while (this->check_route_ahead(-1.0f))
	// 	{
	// 		this->move_robot(5.0f);

	// 		if (this->check_side_space_left())
	// 		{
	// 			this->rotate_robot(-90);
	// 		}
	// 		else if (this->check_side_space_right())
	// 		{
	// 			this->rotate_robot(90);
	// 		}
	// 	}
	// }
	// catch(ErrorFlag err)
	// {
	// 	// char reason[] = err.reason;
	// 	// Serial.println("%s", err.reason);
	// 	// Serial.println()
	// 	// Serial.println("err.reason");
	// }
}

/**
 * @brief Starts the robot driving forwards at its default speed of 0.5
 */
void Robot::drive_forwards()
{
	my_motors.set_direction(my_motors.DIR_FORWARDS);
	my_motors.set_speed(0.5f);
	my_motors.drive_forwards(0);
}

/**
 * @brief Stops the robot from moving
 */
void Robot::stop_moving()
{
	my_motors.stop_driving();
	this->current_state = this->STATE_STOP;
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
 * @param distance_to_move 	The number of cm to move the robot
 */
void Robot::move_robot(float distance_to_move)
{
    float initial_distance_moved_left = 0.0f;
	float initial_distance_moved_right = 0.0f;
	float distance_moved_while_turning_left = 0.0f;
	float distance_moved_while_turning_right = 0.0f;
	float difference_between_motor_distances = 0.0f;

	bool route_ahead_free = false;
	
	route_ahead_free = this->check_route_ahead(distance_to_move);
    //my_map.check_route_ahead_in_map(this->bearing, distance_to_move);

	if (route_ahead_free == false)
	{
		//throw ErrorFlag("Could not move forwards as space was not free", true);
		return;
	}

    // Stop robot moving
	my_motors.stop_driving();

    // Call direction change function on motors depending on the angle direction
	if (distance_to_move > 0)
	{
		my_motors.set_direction(my_motors.DIR_FORWARDS);
	}
	else if (distance_to_move < 0)
	{
		distance_to_move = distance_to_move * -1;
		my_motors.set_direction(my_motors.DIR_BACKWARDS);
	}
	else
	{
		return;
	}

    // Calculate current distance the robot has moved before the robot starts turning the set amount
	initial_distance_moved_left = get_distance_travelled_left();
	initial_distance_moved_right = get_distance_travelled_right();

    // Start robot moving
	my_motors.set_speed(0.5f);
	my_motors.drive_forwards(0);

    while (abs(difference_between_motor_distances) < (distance_to_move * 2.0f))
	{
		// Get left wheel distance moved whilst turning robot
		distance_moved_while_turning_left = get_distance_travelled_left() - initial_distance_moved_left;

		// Get right wheel distance moved whilst turning robot
		distance_moved_while_turning_right = get_distance_travelled_right() - initial_distance_moved_right;

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
	initial_distance_moved_left = get_distance_travelled_left();
	initial_distance_moved_right = get_distance_travelled_right();

	// Calculate equivalent arc of circle
	// Left wheel 80mm out from centre
	// Right wheel 80mm out
	// arc distance = (degrees/360) * 2*pi*radius
	// Distance in mm
	float arc_distance_to_turn = ((float)degrees / 360.0f) * 2.0f * 3.141f * ROBOT_WHEEL_RADIUS;

	// Start robot moving
	my_motors.set_speed(0.5f);
	my_motors.drive_forwards(0);

	// While distance moved < arc length
	// loop
	// Get distance moved by wheels
	// (distance moved could be an average of the two wheels)
	// delay between distance moved polls
	while (abs(difference_between_motor_distances) < (arc_distance_to_turn * 2.0f))
	{
		// Get left wheel distance moved whilst turning robot
		distance_moved_while_turning_left = get_distance_travelled_left() - initial_distance_moved_left;

		// Get right wheel distance moved whilst turning robot
		distance_moved_while_turning_right = get_distance_travelled_right() - initial_distance_moved_right;

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
	int left_sensor_distance = my_sensors.read_averaged_usonic_sensor_left(5);
	int right_sensor_distance = my_sensors.read_averaged_usonic_sensor_right(5);

    // Calculate direction robot is facing
    // Assume the robot is facing towards finish from centre of start location

    // Set coordinate position based on sensor readings
    this->current_position.x_coordinate = (float)left_sensor_distance/5.0f;
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

    if (fmod(this->current_position.x_coordinate, 5) > 2.5)
    {
        if (this->bearing == 0)
        {
            this->rotate_robot(90);
            this->move_robot(-(fmod(this->current_position.x_coordinate, 5)-2.5));
        }
        else if (this->bearing == 90)
        {
            this->move_robot(-(fmod(this->current_position.x_coordinate, 5)-2.5));
        }
    }
    else if (fmod(this->current_position.x_coordinate, 5) < 2.5)
    {
        if (this->bearing == 0)
        {
            this->rotate_robot(90);
            this->move_robot((fmod(this->current_position.x_coordinate, 5)));
        }
        else if (this->bearing == 90)
        {
            this->move_robot((fmod(this->current_position.x_coordinate, 5)));
        }
    }

    if (fmod(this->current_position.y_coordinate, 5) > 2.5)
    {
        if (this->bearing == 0)
        {
            this->move_robot(-(fmod(this->current_position.y_coordinate, 5)-2.5));
        }
        else if (this->bearing == 90)
        {
            this->rotate_robot(-90);
            this->move_robot(-(fmod(this->current_position.y_coordinate, 5)-2.5));
        }
    }
    else if (fmod(this->current_position.x_coordinate, 5) < 2.5)
    {
        if (this->bearing == 0)
        {
            this->move_robot((fmod(this->current_position.x_coordinate, 5)));
        }
        else if (this->bearing == 90)
        {
            this->rotate_robot(-90);
            this->move_robot((fmod(this->current_position.x_coordinate, 5)));
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
		if (distance_to_objects_front > 5.0f)
		{
			return true;
		}
	}

	if (distance_to_objects_front > (distance_to_move + 10))
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
bool Robot::check_side_space_left()
{
	int left_sensor_distance = my_sensors.read_averaged_usonic_sensor_left(5);
	
	if (left_sensor_distance > 15)
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
bool Robot::check_side_space_right()
{
	int right_sensor_distance = my_sensors.read_averaged_usonic_sensor_right(5);

	if (right_sensor_distance > 15)
	{
		return true;
	}
	return false;
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
// 	my_motors.drive_forwards(0);

// 	wait_us(1000000);

// 	this->rotate_robot(90);

// 	// Set the speed of the motors to half the max speed
// 	my_motors.set_speed(0.5f);

// 	// Drive the robot forwards
// 	// 0 seconds means continue forever until told otherwise
// 	my_motors.drive_forwards(0);

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