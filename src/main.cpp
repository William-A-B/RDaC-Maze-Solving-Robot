#include <Arduino.h>
#include <mbed/mbed.h>
#include <events/mbed_events.h>
#include "ble/BLE.h"
#include "motors.h"
#include "joystick.h"
#include "sensors.h"
#include "pindef.h"

Joystick my_joystick;
Motors my_motors;
Sensors my_sensors;

bool continue_running = true;

class Robot
{
public:

	void setup_robot();
	bool run();
	void stop();

	void setup_occupancy_grid();

private:

	void detect_obstacle();
	//enum object_detected detect_obstacle();
	void avoid_obstacle();

	void turn_right();
	void turn_left();

	
	void display_map();

	

	enum object_detected { front_ir, back_ir, left_usonic, right_usonic };

	enum robot_state
	{
		STATE_STOP = 0,
		STATE_FORWARD,
		STATE_BACKWARD,
		STATE_LEFT,
		STATE_RIGHT,
	} current_state;

	unsigned int objects;

	bool occupancy_grid[44][32] = { 0 };




}my_robot;


/**
 * mbed::callback for class interrupts
 * lambda function to call the class interrupt function
*/

//mbed::InterruptIn ir_front(0x5E);

// the setup function runs once when you press reset or power the board
void setup() {

	// // Initialize digital pin LED_BUILTIN as an output.
	// pinMode(LED_BUILTIN, OUTPUT);

	// Start Serial port at 9600 Baudrate
	Serial.begin(BAUDRATE);

	Joystick_setup();

	my_motors.calibrate();
	my_motors.set_direction(my_motors.DIR_FORWARDS);

	attach_encoder_interrupts();

	my_robot.setup_occupancy_grid();

}

// the loop function runs over and over again forever
void loop() {

	//Serial.println(get_encoder_revolutions_left());
	//Serial.println(get_encoder_revolutions_right());
	
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

	my_sensors.run_IR_sensors();

	//wait_us(500000);

	my_sensors.run_usonic_sensors();

	my_motors.set_speed(0.5f);
	my_motors.drive_forwards(0);
	// my_robot.detect_obstacle();
	// my_robot.avoid_obstacle();
	my_robot.display_map();

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

	

	return true;
}

/**
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