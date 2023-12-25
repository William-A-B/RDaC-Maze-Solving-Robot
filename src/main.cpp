/**
 * @file main.cpp
 * @author William Betteridge
 * @brief The main file and entry point into the program
 */
#include <Arduino.h>
#include <mbed/mbed.h>
#include <events/mbed_events.h>
// #include "ble/BLE.h"
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
	my_robot.initialSetup();

	// my_robot.currentState = my_robot.STATE_TESTING;
	my_robot.currentState = my_robot.STATE_SETUP;
}

/**
 * @brief The loop function runs over and over again forever
 * This is the main loop of the entire program and only exits this loop when reset or powered off
 */
void loop()
{
	if (my_joystick.check_button_press() == 3)
	{
		switch (my_robot.currentState)
		{
			case my_robot.STATE_TESTING:
				my_robot.test();
				break;
			case my_robot.STATE_SETUP:
				my_robot.setup();
				break;
			case my_robot.STATE_LOCATE:
				break;
			case my_robot.STATE_SOLVE:
				my_robot.solveMaze();
				break;
			case my_robot.STATE_STOP:
				my_robot.stopMoving();
				break;
			case my_robot.STATE_FORWARD:
				my_robot.driveForwards();
				break;
			case my_robot.STATE_BACKWARD:
				break;
			case my_robot.STATE_LEFT:
				my_robot.rotateRobot(-90);
				break;
			case my_robot.STATE_RIGHT:
				my_robot.rotateRobot(90);
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
	// // Else stopMoving the robot from moving
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
	// 		my_robot.stopMoving();
	// 	}
	// }
	// else
	// {
	// 	set_button_state(0);
	// 	my_robot.stopMoving();
	// }
}

