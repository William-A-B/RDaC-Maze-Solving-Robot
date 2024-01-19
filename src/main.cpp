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

Robot myRobot;
Joystick my_joystick;

bool continue_running = true;

void waitForSerial();

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
	myRobot.initialSetup();

	// myRobot.currentState = myRobot.RobotState::STATE_TESTING;
	myRobot.currentState = myRobot.RobotState::STATE_SETUP;
}

/**
 * @brief The loop function runs over and over again forever
 * This is the main loop of the entire program and only exits this loop when reset or powered off
 */
void loop()
{
	if (my_joystick.check_button_press() == 3)
	{
		// waitForSerial();
		switch (myRobot.currentState)
		{
			case myRobot.STATE_TESTING:
				myRobot.test();
				break;
			case myRobot.STATE_SETUP:
				myRobot.setRGBLED(1, 1, 0);
				myRobot.setup();
				break;
			case myRobot.STATE_LOCATE:
				break;
			case myRobot.STATE_SOLVE:
				myRobot.setRGBLED(0, 0, 0);
				myRobot.solveMaze();
				break;
			case myRobot.STATE_SOLVE_KNOWN_MAZE:
				myRobot.solveKnownMaze();
				break;
			case myRobot.STATE_DETERMINE_DIRECTION:
				myRobot.setRGBLED(0, 1, 1);
				myRobot.determineDirection();
				break;
			case myRobot.STATE_LEFT_AND_FORWARD:
				myRobot.setRGBLED(0, 0, 1);
				myRobot.leftAndForwards();
				break;
			case myRobot.STATE_RIGHT_AND_FORWARD:
				myRobot.setRGBLED(0, 0, 1);
				myRobot.rightAndForwards();
				break;
			case myRobot.STATE_STOP:
				myRobot.setRGBLED(1, 1, 1);
				myRobot.stopMoving();
				break;
			case myRobot.STATE_FORWARD:
				myRobot.setRGBLED(1, 0, 1);
				myRobot.moveForwards();
				break;
			case myRobot.STATE_BACKWARD:
				break;
			case myRobot.STATE_LEFT:
				myRobot.rotateRobot(-90);
				break;
			case myRobot.STATE_RIGHT:
				myRobot.rotateRobot(90);
				break;
			case myRobot.STATE_RETRACE_ROUTE:
				myRobot.setRGBLED(1, 0, 0);
				myRobot.retraceRouteBack();
				break;
			case myRobot.STATE_END:
				set_button_state(0);
				break;
		}
	}
	else if (my_joystick.check_button_press() == 2)
	{
		myRobot.stopMoving();
		//myRobot.myMap.displayRobotHistory();
		myRobot.myMap.displayMap();
		my_joystick.set_button_press(1);
		myRobot.currentState = myRobot.RobotState::STATE_STOP;
	}



	// // If the robot is set to continue running keep on checking the joystick button presses
	// // Else stopMoving the robot from moving
	// if (continue_running == true)
	// {
	// 	// Forwards direction pressed on Joystick --> Run
	// 	if (my_joystick.check_button_press() == 3)
	// 	{
	// 		continue_running = myRobot.start();
	// 		//continue_running = myRobot.run();
	// 	}
	// 	// Backwards direction pressed on Joystick --> Stop
	// 	else if (my_joystick.check_button_press() == 2)
	// 	{
	// 		myRobot.stopMoving();
	// 	}
	// }
	// else
	// {
	// 	set_button_state(0);
	// 	myRobot.stopMoving();
	// }
}

void waitForSerial(){
	while(Serial.available()){Serial.read();}
	while (!Serial.available()) { }
	Serial.read();
}