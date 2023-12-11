/**
 * @file main.cpp
 * @author William Betteridge
 * @brief The main file and entry point into the program
 */
#include <Arduino.h>
#include <mbed/mbed.h>
#include <events/mbed_events.h>
#include "ble/BLE.h"
#include "motors.h"
#include "joystick.h"
#include "sensors.h"
#include "pindef.h"

// Instantiate objects for the other classes
Joystick my_joystick;
Motors my_motors;
Sensors my_sensors;

bool continue_running = true;

/**
 * @brief A class to act as a wrapper and contain all functions 
 * and variables for the main file
 * 
 */
class Robot
{
public:

	/**
	 * @brief Called from the main arduino loop() function
 	 * The main function to run the robot and all its processes.
 	 * Is continuously called whilst the robot is set to continue running 
	 * 
	 * @return true		The robot should continue running its program
	 * @return false	The robot stops running
	 */
	bool run();

	/**
	 * @brief Stops the robot from moving
	 * 
	 */
	void stop();

	/**
	 * @brief Setup the occupancy grid
	 * 
	 */
	void setup_occupancy_grid();

private:

	/**
	 * @brief Checks with the sensors whether objects are within 
	 * range of the robot
	 * 
	 */
	void detect_obstacle();

	//enum object_detected detect_obstacle();

	/**
	 * @brief Tell the robot to avoid the obstacles
	 * that were detected in the detect_obstacle() function
	 * 
	 */
	void avoid_obstacle();

	void turn_right();
	void turn_left();

	/**
	 * @brief Prints the occupancy map to the serial port
	 * and displays the occupancy map in a readable format
	 * 
	 */
	void display_map();

	void rotate_robot(int degrees);

	enum object_detected { front_ir, back_ir, left_usonic, right_usonic };

	enum robot_state
	{
		STATE_STOP = 0,
		STATE_FORWARD,
		STATE_BACKWARD,
		STATE_LEFT,
		STATE_RIGHT,
	} current_state;

	/**
	 * @brief Integer to represent which objects are within range of the robot
	 * 
	 */
	unsigned int objects;

	/**
	 * @brief The Occupancy grid which is used to map out all objects within the map
	 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
	 * 
	 */
	bool occupancy_grid[44][32] = { 0 };
}my_robot;


/**
 * mbed::callback for class interrupts
 * lambda function to call the class interrupt function
*/

//mbed::InterruptIn ir_front(0x5E);

/**
 * @brief The setup function runs once when you press reset or power the board
 * 
 */
void setup() {

	// // Initialize digital pin LED_BUILTIN as an output.
	// pinMode(LED_BUILTIN, OUTPUT);

	// Start Serial port at 9600 Baudrate
	Serial.begin(BAUDRATE);

	// Call the setup function to initialise the joystick so it can be used to control the robot
	Joystick_setup();

	// Calibrate the motors for use
	my_motors.calibrate();

	// Set the default direction for the robot to move in
	my_motors.set_direction(my_motors.DIR_FORWARDS);

	// Attach the interrupts for the encoders on the motors
	attach_encoder_interrupts();
	
	// Setup the occupancy grid array to initialise the maze prior to the robot moving
	my_robot.setup_occupancy_grid();

}

/**
 * @brief The loop function runs over and over again forever
 * This is the main loop of the entire program and only exits this loop when reset or powered off
 */
void loop() {
	
	// If the robot is set to continue running keep on checking the joystick button presses
	// Else stop the robot from moving
	if (continue_running == true)
	{
		// Forwards direction pressed on Joystick --> Run
		if (my_joystick.check_button_press() == 3)
		{
			continue_running = my_robot.run();
		}
		// Backwards direction pressed on Joystick --> Stop
		else if (my_joystick.check_button_press() == 2)
		{
			my_robot.stop();
		}
	}
	else
	{
		set_button_state(0);
		my_robot.stop();
		// Temp variable assignment
		continue_running = true;
	}

	// else if (my_joystick.check_button_press() == 0)
	// {
	// 	my_motors.set_speed(my_motors.get_speed()+0.1f);
	// }
	// else if (my_joystick.check_button_press() == 1)
	// {
	// 	my_motors.set_speed(my_motors.get_speed()-0.1f);
	// }
	// else
	// {
	// 	Serial.println("No Function Selected");
	// }
}

/**
 * @brief Called from the main arduino loop() function
 * The main function to run the robot and all its processes.
 * Is continuously called whilst the robot is set to continue running 
 * 
 * @return true		The robot should continue running its program
 * @return false	The robot stops running
 */
bool Robot::run()
{

	// switch (current_state)
	// {
	// 	case STATE_STOP:
	// 		my_robot.stop();
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

	// Run the functions to read values from the infrared sensors
	my_sensors.run_IR_sensors();

	//wait_us(500000);

	// Run the functions to read values from the ultrasonic sensors
	my_sensors.run_usonic_sensors();

	// Set the speed of the motors to half the max speed
	my_motors.set_speed(0.5f);
	
	// Drive the robot forwards
	// 0 seconds means continue forever until told otherwise
	my_motors.drive_forwards(0);

	// Detect obstacles in the nearby area
	// my_robot.detect_obstacle();

	// Avoid obstacles in the nearby area by moving away from them
	// my_robot.avoid_obstacle();

	// Display the occupancy grid in a readable format
	my_robot.display_map();

	// Debug statements for calibrating the encoders to check the distance travelled
	Serial.println("Distance Travelled Left in mm:");
	Serial.print(get_distance_travelled_left());
	Serial.print(" mm\n");
	Serial.println("Distance Travelled Left in cm:");
	Serial.print(get_distance_travelled_left()/10);
	Serial.print(" cm\n\n");

	Serial.println("Distance Travelled Right in mm:");
	Serial.print(get_distance_travelled_right());
	Serial.print(" mm\n");
	Serial.println("Distance Travelled Right in cm:");
	Serial.print(get_distance_travelled_right()/10);
	Serial.print(" cm\n\n");

	// if (get_encoder_revolutions_left() >= 1000)
	// {
	// 	return false;
	// }

	
	// Return true to continue running the loop
	return true;
}

/**
 * @brief Checks with the sensors whether objects are within
 * range of the robot
 *
 * @return objects - bitfield defining the surrounding objects
 * 0 - No Objects
 * 1 - Front Object
 * 2 - Rear Object
 * 3 - Left Object
 * 4 - Right Object
 * 5 - Front and Rear Object
 * 6 - Front and Left Object
 * 7 - Front and Right Object
 * 8 - Rear and Left Object
 * 9 - Rear and Right Object
 * 10 - Left and Right Objects
 * 11 - Front and Rear and Left Objects
 * 12 - Front and Rear and Right Objects
 * 13 - Front and Left and Right Objects
 * 14 - Rear and Left and Right Objects
 * 15 - Front and Rear and Left and Right Objects
*/
void Robot::detect_obstacle()
{
	if (my_sensors.get_front_IR_distance() < MIN_IR_DIST)
	{
		if (my_sensors.get_left_usonic_distance() < MIN_USONIC_DIST && my_sensors.get_right_usonic_distance() < MIN_USONIC_DIST)
		{
			objects = 13;
		}
		else if (my_sensors.get_left_usonic_distance() < MIN_USONIC_DIST)
		{
			objects = 6;
		}
		else if (my_sensors.get_right_usonic_distance() < MIN_USONIC_DIST)
		{
			objects = 7;
		}
		else
		{
			objects = 1;
		}
	}
	else
	{
		objects = 0;
	}
	
}

/**
 * @brief Tell the robot to avoid the obstacles
 * that were detected in the detect_obstacle() function
 * 
 */
void Robot::avoid_obstacle()
{
	switch(objects)
	{
		case 0:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			//my_motors.set_speed(1.0f);
			break;
		case 1:
			my_motors.set_direction(my_motors.DIR_ANTICLOCKWISE);
			my_motors.set_speed(0.5f);
			//my_robot.turn_left();
			break;
		case 2:
			my_motors.set_direction(my_motors.DIR_CLOCKWISE);
			my_motors.set_speed(0.5f);
			break;
		case 3:
			my_motors.set_direction(my_motors.DIR_ANTICLOCKWISE);
			my_motors.set_speed(0.5f);
			break;
		case 4:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 5:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 6:
			my_motors.set_direction(my_motors.DIR_CLOCKWISE);
			my_motors.set_speed(0.5f);
			break;
		case 7:
			my_motors.set_direction(my_motors.DIR_ANTICLOCKWISE);
			my_motors.set_speed(0.5f);
			break;
		case 8:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 9:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 10:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 11:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 12:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 13:
			my_motors.set_direction(my_motors.DIR_BACKWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 14:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
		case 15:
			my_motors.set_direction(my_motors.DIR_FORWARDS);
			my_motors.set_speed(0.5f);
			break;
	}
}

void Robot::stop()
{
	my_motors.stop_driving();
}

void Robot::turn_left()
{
	my_motors.turn_left(90);
}

void Robot::turn_right()
{
	my_motors.turn_right(90);
}

void Robot::setup_occupancy_grid()
{
	// Set all values in occupancy grid to 0
	for (int i = 0; i < 44; i++)
	{
		for (int j = 0; j < 32; j++)
		{
			occupancy_grid[i][j] = 0;
		}
	}

	// Set all values in the outer perimeter to 1 to represent the edge of the maze
	for (int i = 0; i < 44; i++)
	{
		occupancy_grid[i][0] = 1;
		occupancy_grid[i][31] = 1;
	}

	for (int i = 0; i < 32; i++)
	{
		occupancy_grid[0][i] = 1;
		occupancy_grid[43][i] = 1;
	}
}

void Robot::display_map()
{

	for (int i = 0; i < 44; i++)
	{
		for (int j = 0; j < 32; j++)
		{
			Serial.print(occupancy_grid[i][j]);
		}
		Serial.print("\n");
	}
	// Serial.println("--------------------------------");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110010000001000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110010001111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("|110011111111000111001111111111|");
	// Serial.println("--------------------------------");

}


void Robot::rotate_robot(int degrees)
{
	// Stop robot moving
	// Call direction change function on motors

	// Calculate equivalent arc of circle

	// Radius from centre of robot to centre of wheel
	// Radius may be different for both wheels (take average)

	// arc distance = (degrees/360) * 2*pi*radius
	
	// Define which wheel is left and which is right.
	// Left = forwards for turning clockwise
	// Right = backwards for turning clockwise
	// Opposite if turning anticlockwise

	// Start robot moving

	// While distance moved < arc length
	// loop
	// Get distance moved by wheels
	// (distance moved could be an average of the two wheels)
	// delay between distance moved polls


	// Stop robot moving
}