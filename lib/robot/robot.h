#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"
#include "motor.h"
#include "sensors.h"
#include "map.h"
#include "errorflag.h"
#include "bluetooth.h"

// Radius of wheels from centre of robot to centre of wheels
#define ROBOT_WHEEL_RADIUS 8.0f
#define MAX_IR_DISTANCE 52.0f // 63.75 max distance from sensor reference point. 71.45f from robot centre
#define MAX_USONIC_DISTANCE 52.0f // 120.0f originally
// #define DEFAULT_ROBOT_SPEED 0.75f
#define DEFAULT_ROBOT_SPEED 0.50f

#define MIN_IR_DIST_FRONT 16.0f // Minimum distance for IR sensor to start detecting objects
#define MIN_IR_DIST_REAR 16.0f
#define MIN_USONIC_DIST 16.0f // Minimum distance for USonic sensor to start detecting objects

#define DEFAULT_AVG_SENSOR_READINGS 3

struct Position
{
	// Float coordinate positions for the absolute position of the robot within the maze
	float xCoordinate;
	float yCoordinate;
};


/**
 * @brief A class to act as a wrapper and contain all functions
 * and variables for the main file
 *
 */
class Robot
{
public:

	// FUNCTION DECLARATIONS

	/**
	 * @brief Function used to test various features. Any code implementation is often temporary
	 * But may be promoted up to its own function at later use.
	 */
	void test();

	void processSensorInfo();

	/**
	 * @brief Run once upon powering on the robot.
	 * Sets up sections of robot which only ever need to be set once
	 */
    void initialSetup();

	/**
	 * @brief Run once the robot is placed at the starting position in the maze
	 * Is a state within the state machine, can be called again to restart the robots algorithms
	 */
	void setup();

	/**
	 * @brief Robot starts solving the maze and finding its way to the end
	 * Runs all algorithms for solving and mapping from this function
	 * Part of state SOLVE within the state machine
	 */
	void solveMaze();

	/**
	 * @brief Moves the robot a given distance, if the value given is negative the robot will move backwards
	 * 
	 * @param distanceToMove 	The number of cm to move the robot
	 */
    void moveRobot(float distanceToMove);

    /**
     * @brief Rotates the robot about a point a set number of degrees
     *
     * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
	 * 
     */
    void rotateRobot(int degrees);

	/**
     * @brief Rotates the robot about a point a set number of degrees
     *
     * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
	 * 
     */
    void rotateRobot(int degrees, bool ignoreBearingUpdate);

	void determineDirection();

	void moveForwards();

	void stopMoving();

	void leftAndForwards();

	void rightAndForwards();

	void solveKnownMaze();

	/**
	 * @brief Starts the robot driving forwards at its default speed of 0.5
	 */
    void driveForwards();


    // VARIABLE DECLARATIONS

	/**
	 * @brief 
	 * 
	 */
    enum RobotState
	{
		STATE_TESTING,
		STATE_SETUP,
		STATE_LOCATE,
		STATE_SOLVE,
		STATE_SOLVE_KNOWN_MAZE,
		STATE_DETERMINE_DIRECTION,
		STATE_LEFT_AND_FORWARD,
		STATE_RIGHT_AND_FORWARD,
		STATE_STOP,
		STATE_FORWARD,
		STATE_BACKWARD,
		STATE_LEFT,
		STATE_RIGHT,
		STATE_END,
	} currentState;


	bool driveForwardsStarted = false;
private:

	// Instantiate objects for the other classes
	Motor myMotors;
	// Motor left_motor;
	// Motor right_motor;
	Sensors mySensors;
public:
	// Map objects to map the maze
	Map myMap;

	Bluetooth robotBLE;
	
private:
	// FUNCTION DECLARATIONS

	/**
	 * @brief Runs the algorithm for the robot to solve the maze via moving
	 * directly forwards until it reaches an obstacle then it turns into the
	 * next free direction and repeats
	 */
	void runDirectAlgorithm();

	void moveHelper();

	/**
	 * @brief Calculates the starting position of the robot
	 * Checks surrounding areas based on sensors
	 * And sets the coordinate values based on the sensors
	 */
	void calculateStartingLocation();

	/**
	 * @brief Centres the robot in the middle of the occupancy grid square
	 */
    void centreOnMapGrid();

	/**
	 * @brief Updates the robots bearing to identify the direction in which the robot is currently heading
	 * 
	 * @param angleToAdd 	The angle in which the robot is currently turning by
	 */
	void updateBearing(int angleToAdd);

	/**
	 * @brief Checks the space in front of the robot to see if there is free space or not
	 * Decides whether the robot is able to move forwards the specified distance
	 * 
	 * @param distanceToMove 	The distance to check whether the robot can move forwards
	 * @return true 			True if the robot is allowed to move forwards
	 * @return false 			False if there is an object in the way and the robot can't move forwards
	 */
	bool checkRouteAhead(float distanceToMove);

	/**
	 * @brief Checks if there is free space to move into on the left of the robot
	 * 
	 * @return true 	True if the robot can turn left and move forwards
	 * @return false 	False if the robot cannot move left and something is blocking it
	 */
	bool checkSideSpaceLeft(int numReadings);

	/**
	 * @brief Checks if there is free space to move into on the right of the robot
	 * 
	 * @return true 	True if the robot can turn right and move forwards
	 * @return false 	False if the robot cannot move left and something is blocking it
	 */
    bool checkSideSpaceRight(int numReadings);


	void updateCoordinateLocation();

	void correctOrientation();

	void correctOrientationHelper(float a, float b, float aC, bool leftOfCentre);

	void setRGBLED(int red, int green, int blue);

	void runShortestNavigationAlgorithm();

	/**
	 * @brief Convert bearing into a numbering system to determine the direction
	 * for the finish location distance, using integer maths
	 * 
	 * @return int - The direction the robot is facing
	 */
	int adjustedMapDirection();

	/**
	 * @brief Runs the algorithm for the robot to solve the maze via following
	 * the left hand wall until it reaches the end.
	 */
	void runWallFollowingAlgorithm();

	void correctOrientationWall();


	// VARIABLE DECLARATIONS

	enum object_detected
	{
		front_ir,
		back_ir,
		left_usonic,
		right_usonic
	};

	enum algorithm
	{
		FOLLOW_WALL,
		DIRECT_AND_AROUND,
		NAVIGATE_MAP,
		KNOWN_MAZE,
	} algorithm;

	/**
	 * @brief Integer to represent which objects are within range of the robot
	 *
	 */
	unsigned int objects;

	// Bearing of the robot in relation to maze finish location
	// Bearing of zero is "North", and faces directly towards the finish from the starting location
	int bearing = 0;

    // Float coordinate positions for the absolute position of the robot within the maze
    Position currentPosition;

	float initialDistanceMovedLeft;
	float initialDistanceMovedRight;

	float distanceToMoveForwards;


	bool verbose;
	
};


	// /**
	//  * @brief Checks with the sensors whether objects are within
	//  * range of the robot
	//  *
	//  */
	// void detect_obstacle();

	// // enum object_detected detect_obstacle();

	// /**
	//  * @brief Tell the robot to avoid the obstacles
	//  * that were detected in the detect_obstacle() function
	//  *
	//  */
	// void avoid_obstacle();



	// /**
	//  * @brief Called from the main arduino loop() function
	//  * The main function to run the robot and all its processes.
	//  * Is continuously called whilst the robot is set to continue running
	//  *
	//  * @return true		The robot should continue running its program
	//  * @return false	The robot stops running
	//  */


	

	// /**
	//  * @brief Called when the robot must run its main algorithm
	//  *
	//  * @return true		The robot should continue running its program
	//  * @return false	The robot stops running
	//  */
	// bool run();




#endif