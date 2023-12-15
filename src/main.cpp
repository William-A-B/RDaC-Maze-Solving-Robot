/**
 * @file main.cpp
 * @author William Betteridge
 * @brief The main file and entry point into the program
 */
#include <Arduino.h>
#include <mbed/mbed.h>
#include <events/mbed_events.h>
#include "ble/BLE.h"
#include "pindef.h"
#include "joystick.h"
#include "robot.h"

Robot my_robot;
Joystick my_joystick;

bool continue_running = true;



/**
 * @brief The setup function runs once when you press reset or power the board
 */
void setup()
{
	// Start Serial port at 9600 Baudrate
	Serial.begin(BAUDRATE);

	// Call the setup function to initialise the joystick so it can be used to control the robot
	Joystick_setup();

	// Setup for robot that is run once at the very start
	my_robot.initial_setup();

	my_robot.current_state = my_robot.STATE_SETUP;
	// my_robot.current_state = my_robot.STATE_SOLVE;
}

/**
 * @brief The loop function runs over and over again forever
 * This is the main loop of the entire program and only exits this loop when reset or powered off
 */
void loop()
{
	if (my_joystick.check_button_press() == 3)
	{
		switch (my_robot.current_state)
		{
			case my_robot.STATE_SETUP:
				my_robot.setup();
				break;
			case my_robot.STATE_LOCATE:
				break;
			case my_robot.STATE_SOLVE:
				my_robot.solve_maze();
				break;
			case my_robot.STATE_STOP:
				my_robot.stop_moving();
				break;
			case my_robot.STATE_FORWARD:
				my_robot.drive_forwards();
				break;
			case my_robot.STATE_BACKWARD:
				break;
			case my_robot.STATE_LEFT:
				my_robot.rotate_robot(-90);
				break;
			case my_robot.STATE_RIGHT:
				my_robot.rotate_robot(90);
				break;
			case my_robot.STATE_END:
				set_button_state(0);
				break;
		}
	}
	else if (my_joystick.check_button_press() == 2)
	{
		my_robot.end();
	}



	// // If the robot is set to continue running keep on checking the joystick button presses
	// // Else stop_moving the robot from moving
	// if (continue_running == true)
	// {
	// 	// Forwards direction pressed on Joystick --> Run
	// 	if (my_joystick.check_button_press() == 3)
	// 	{
	// 		continue_running = my_robot.start();
	// 		//continue_running = my_robot.run();
	// 	}
	// 	// Backwards direction pressed on Joystick --> Stop
	// 	else if (my_joystick.check_button_press() == 2)
	// 	{
	// 		my_robot.stop_moving();
	// 	}
	// }
	// else
	// {
	// 	set_button_state(0);
	// 	my_robot.stop_moving();
	// }
}

