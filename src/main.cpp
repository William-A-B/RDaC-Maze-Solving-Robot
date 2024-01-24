/**
 * @file main.cpp
 * @author Y3905304
 * @brief The main file and entry point into the program
 */
#include <Arduino.h>
#include <mbed/mbed.h>
#include <events/mbed_events.h>
// #include "ble/BLE.h"
#include "pindef.h"
#include "joystick.h"
#include "robot.h"

// Objects for main robot and joystick
Robot myRobot;
Joystick myJoystick;

/**
 * @brief Waits for a serial key press to occur and then continues
 * Debug function to stop the program each time it switches state
 */
void waitForSerial();

/**
 * @brief The setup function runs once when you press reset or power the board
 */
void setup()
{
	// Start Serial port at 9600 Baudrate
	Serial.begin(BAUDRATE);

	// Call the setup function to initialise the joystick so it can be used to control the robot
	myJoystick.joystickSetup();

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
	// Only start state machine when forwards joystick direction pressed
	// Allows more control over when the robot starts
	if (myJoystick.checkJoystickPress() == 0)
	{
		// Debug function used if wanting to wait for a serial keypress before switching to each next state
		// waitForSerial();

		// Switch to the current robot state in the state machine
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
			case myRobot.STATE_REVERSE_DIRECTION:
				myRobot.reverseDirection();
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
				//set_button_state(0);
				break;
		}
	}
	// If backwards direction on joystick pressed stop robot and display map
	// Robot state machine has ended
	else if (myJoystick.checkJoystickPress() == 1)
	{
		myRobot.stopMoving();
		myRobot.myMap.displayRobotHistory();
		myRobot.myMap.displayMap();
		myRobot.currentState = myRobot.RobotState::STATE_STOP;
	}
}

/**
 * @brief Waits for a serial key press to occur and then continues
 * Debug function to stop the program each time it switches state
 */
void waitForSerial(){
	while(Serial.available()){Serial.read();}
	while (!Serial.available()) { }
	Serial.read();
}