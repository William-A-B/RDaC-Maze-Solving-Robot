#ifndef ROBOT_H
#define ROBOT_H

/**
 * @file robot.h
 * @author Y3905304
 * @brief The header file for the Robot class, containing the main logic for the Robot.
 * The Robot class represents a robot in a 2D grid. It includes methods for moving the robot,
 * checking its surroundings, and other actions the robot will need to perform such as updating its position,
 * adding information into the map etc.
 */

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

// Maximum distance that the sensors detect objects for when adding them into the map
// 63.75 max distance for sensors. 71.45f from robot centre
// After 20cm these values become marginally inaccurate so anything being added into the map becomes unsuitable. 
#define MAX_IR_DISTANCE 20.0f 
#define MAX_USONIC_DISTANCE 20.0f

// Default speed for the robot to move at - 50% its maximum speed results in a greater accuracy of movement
#define DEFAULT_ROBOT_SPEED 0.50f

// Minimum distance that the sensors must read for the robot to be suitably far away from any obstacles
#define MIN_IR_DIST_FRONT 16.0f
#define MIN_IR_DIST_REAR 16.0f
#define MIN_USONIC_DIST 16.0f

// Number of sensor readings to average over when reading values from the sensors
#define DEFAULT_AVG_SENSOR_READINGS 3

// Position struct, containing two floats for an x and y coordinate in the maze
struct Position
{
	float xCoordinate;
	float yCoordinate;
};

/**
 * @brief A class to act as a wrapper and contain all functions
 * and variables for the main file
 * All robot logic is handled here and extra function calls to motors, or sensors are made here.
 */
class Robot
{
public:
	// Map objects to map the maze
	Map myMap;
	// Object for the bluetooth class
	Bluetooth robotBLE;

private:

	// Instantiate objects for the other classes
	// Object for the motors class
	Motor myMotors;
	// Object for the sensors class
	Sensors mySensors;

public:

	/**
	 * @brief Function used to test various features. Any code implementation is often temporary
	 * But may be promoted up to its own function at later use.
	 * Set the robot state to STATE_TESTING in the main setup() function to run any code here
	 */
	void test();

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
    bool moveRobot(float distanceToMove);

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
	* @param ignoreBearingUpdate 	Whether to update the robots bearing or not
	*/
    void rotateRobot(int degrees, bool ignoreBearingUpdate);

	/**
	 * @brief Reverses the robots direction so it rotates 180
	 * and faces the other way
	 */
	void reverseDirection();

	/**
	 * @brief Gets sensor distances from all four sensors
	 * If obstacles are within the maximum measuring distance, add them into the map
	 */
	void processSensorInfo();

	/**
	 * @brief Determines the direction to move at each step forwards in the map grid.
	 * Determines the next robot state based on the obstacles sensors detect
	 * as well as any objects within the maze that affect the next movement
	 */
	void determineDirection();

	/**
	 * @brief Wrapper for the moveHelper() function, called when the robot is in the state STATE_FORWARD
	 * At the end, returns the robot back to the STATE_DETERMINE_DIRECTION state
	 */
	void moveForwards();

	/**
	 * @brief Stops the robot moving completely, so that it is stationary
	 */
	void stopMoving();

	/**
	 * @brief Sets the robot to turn left and then move forwards by setting the robot state to STATE_FORWARD
	 * Called from the STATE_LEFT_AND_FORWARD state
	 */
	void leftAndForwards();

	/**
	 * @brief Sets the robot to turn right and then move forwards by setting the robot state to STATE_FORWARD
	 * Called from the STATE_RIGHT_AND_FORWARD state
	 */
	void rightAndForwards();

	/**
	 * @brief Starts the robot driving forwards at its default speed of 0.5
	 */
    void driveForwards();

	/**
	 * @brief Called once the robot has reached the end of the maze
	 * Robot retraces its steps back along the coordinates it took when 
	 * solving the maze.
	 * Does not require any navigation sensing so is much faster to return to the start
	 */
	void retraceRouteBack();

	/**
	 * @brief Sets the RGB LED onboard the Arduino Nano to a given colour
	 * LED is active low, so 0 = on, 1 = off
	 * 
	 * @param red 	- controls the red LED
	 * @param green - controls the green LED
	 * @param blue 	- controls the blue LED
	 */
	void setRGBLED(int red, int green, int blue);


    // VARIABLE DECLARATIONS

	/**
	 * @brief The enum of all the possible states the robot can take
	 * Used to represent a state machine that determines the actions of the robot
	 * currentState is the current state and current action of the robot
	 */
    enum RobotState
	{
		STATE_TESTING,
		STATE_SETUP,
		STATE_LOCATE,
		STATE_SOLVE,
		STATE_DETERMINE_DIRECTION,
		STATE_LEFT_AND_FORWARD,
		STATE_RIGHT_AND_FORWARD,
		STATE_STOP,
		STATE_FORWARD,
		STATE_REVERSE_DIRECTION,
		STATE_LEFT,
		STATE_RIGHT,
		STATE_RETRACE_ROUTE,
		STATE_END,
	} currentState;

private:
	// FUNCTION DECLARATIONS

	/**
	 * @brief Runs the algorithm for the robot to solve the maze via moving
	 * directly forwards until it reaches an obstacle then it turns into the
	 * next free direction and repeats
	 */
	void runDirectAlgorithm();

	/**
	 * @brief Helper function for the moveRobot() function
	 * Series of function calls that must be run each time the robot moves forward one grid square
	 * Moves the robot forwards the distanceToMoveForwards value
	 * Updates the new coordinate location in the map
	 * And adds any obstacles into the map
	 * 
	 * @return true 	- If the robot has moved forwards successfully
	 * @return false 	- If the robot has not moved forwards successfully
	 * 
	 * @return used to know whether the robot has moved forwards successfully or not
	 * So that when the robot is retracing its steps it doesn't get stuck infront of a wall
	 */
	bool moveHelper();

	/**
	 * @brief Calculates the starting position of the robot
	 * Checks surrounding areas based on sensors
	 * And sets the coordinate values based on the sensors
	 */
	void calculateStartingLocation();

	/**
	 * @brief Centres the robot in the middle of the occupancy grid square that it is currently in.
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

	/**
	 * @brief updates the robots absolute coordinate locations within the maze,
	 * based on the distance moved by each wheel
	 * Uses the initialDistanceMovedLeft/Right member variables to calculate the distance moved
	 */
	void updateCoordinateLocation();

	/**
	 * @brief Each step the robot moves forwards, if it is not facing perpendicular to the wall
	 * it will correct its orientation to be perpendicular to the wall.
	 * Rotates left and right to determine the distance to the wall in front of the robot
	 * Then rotates back to the original position and turns to face the wall based on the angle it calculates
	 */
	void correctOrientation();

	/**
	 * @brief Helper funciton for correctOrientation().
	 * Uses the distances calculated by the front sensor to determine the angle the robot needs
	 * to turn in order to face perpendicular towards the wall.
	 * 
	 * @param a 			- Original sensor distance in the direction the robot is currently facing
	 * @param b 			- Shortest sensor distance between the left and right rotation checks
	 * @param aC 			- The angle the robot turns to check the other two distances
	 * @param leftOfCentre 	- Whether the robot is left of the centre of its original direction or not
	 * 						  Used to determine whether the robot needs to turn left or right to face the wall when correcting itself
	 */
	void correctOrientationHelper(float a, float b, float aC, bool leftOfCentre);

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

	/**
	 * @brief Adjusts the orientation to be perpendicular to the wall
	 * Called when the robot is in the FOLLOW_WALL algorithm
	 */
	void correctOrientationWall();

	/**
	 * @brief Rotates the robot to the direction given
	 * The direction can be any cardinal direction
	 * 
	 * @param direction - The direction to turn towards
	 * 0 - North
	 * 1 - East
	 * 2 - South
	 * 3 - West
	 */
	void rotateRobotToGivenDirection(int direction);


	// VARIABLE DECLARATIONS

	/**
	 * @brief Enum to determine which algorithm for solving the maze the robot should run
	 */
	enum algorithm
	{
		FOLLOW_WALL,
		NAVIGATE_MAP,
	} algorithm;

	// Bearing of the robot in relation to maze finish location
	// Bearing of zero is "North", and faces directly towards the finish from the starting location
	int bearing = 0;

    // Float coordinate positions for the absolute position of the robot within the maze
    Position currentPosition;

	/**
	 * @brief Used to store the initial distance moved right when the robot starts moving forwards
	 * Can be used to calculate the distance the robot moves in one movement
	 * 
	 * @param initialDistanceMovedRight 	The initial right wheel distance
	 * @param initialDistanceMovedLeft 		The initial left wheel distance
	 */
	float initialDistanceMovedRight;
	float initialDistanceMovedLeft;

	/**
	 * @brief The distance the robot moves forwards each loop of the state machine
	 * Defaults to 5cm which is the same as the size of one grid square in the map
	 */
	float distanceToMoveForwards;

	/**
	 * @brief Debug variable, set to true for extra debug information to be printed to the serial port
	 */
	bool verbose;
	
};

#endif