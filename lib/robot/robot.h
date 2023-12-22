#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"
#include "motor.h"
#include "sensors.h"
#include "map.h"
#include "errorflag.h"

// Radius of wheels from centre of robot to centre of wheels
#define ROBOT_WHEEL_RADIUS 8.0f
#define MAX_IR_DISTANCE 63.75
#define DEFAULT_ROBOT_SPEED 0.75f

#define MIN_IR_DIST_FRONT 22.7f // Minimum distance for IR sensor to start detecting objects
#define MIN_IR_DIST_REAR 26.3f
#define MIN_USONIC_DIST 22.75f // Minimum distance for USonic sensor to start detecting objects



struct robot_position
{
	// Float coordinate positions for the absolute position of the robot within the maze
	float x_coordinate;
	float y_coordinate;
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

	/**
	 * @brief Run once upon powering on the robot.
	 * Sets up sections of robot which only ever need to be set once
	 */
    void initial_setup();

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
	void solve_maze();

	/**
	 * @brief Starts the robot driving forwards at its default speed of 0.5
	 */
    void drive_forwards();

	/**
	 * @brief Starts the robot driving backwards at its default speed of 0.5
	 */
	void drive_backwards();

	/**
	 * @brief Stops the robot from moving
	 */
	void stop_moving();

	/**
	 * @brief Ends all processes on the robot and stops the state machine
	 */
    void end();

	/**
	 * @brief Moves the robot a given distance, if the value given is negative the robot will move backwards
	 * 
	 * @param distance_to_move 	The number of cm to move the robot
	 */
    void move_robot(float distance_to_move);

    /**
     * @brief Rotates the robot about a point a set number of degrees
     *
     * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
     */
    void rotate_robot(int degrees);


    // VARIABLE DECLARATIONS

    enum robot_state
	{
		STATE_TESTING,
		STATE_SETUP,
		STATE_LOCATE,
		STATE_SOLVE,
		STATE_STOP,
		STATE_FORWARD,
		STATE_BACKWARD,
		STATE_LEFT,
		STATE_RIGHT,
		STATE_END,
	} current_state;

private:

	// FUNCTION DECLARATIONS

	/**
	 * @brief Calculates the starting position of the robot
	 * Checks surrounding areas based on sensors
	 * And sets the coordinate values based on the sensors
	 */
	void calculate_starting_location();

	/**
 	* @brief Inserts the starting location into the occupancy grid
 	*/
	void initialise_starting_location_in_map();

	/**
	 * @brief Centres the robot in the middle of the occupancy grid square
	 */
    void centre_on_map_grid();

	/**
	 * @brief Updates the robots bearing to identify the direction in which the robot is currently heading
	 * 
	 * @param angle_to_add 	The angle in which the robot is currently turning by
	 */
	void update_bearing(int angle_to_add);

	/**
	 * @brief Checks the space in front of the robot to see if there is free space or not
	 * Decides whether the robot is able to move forwards the specified distance
	 * 
	 * @param distance_to_move 	The distance to check whether the robot can move forwards
	 * @return true 			True if the robot is allowed to move forwards
	 * @return false 			False if there is an object in the way and the robot can't move forwards
	 */
	bool check_route_ahead(float distance_to_move);

	/**
	 * @brief Checks if there is free space to move into on the left of the robot
	 * 
	 * @return true 	True if the robot can turn left and move forwards
	 * @return false 	False if the robot cannot move left and something is blocking it
	 */
	bool check_side_space_left(int num_readings);

	/**
	 * @brief Checks if there is free space to move into on the right of the robot
	 * 
	 * @return true 	True if the robot can turn right and move forwards
	 * @return false 	False if the robot cannot move left and something is blocking it
	 */
    bool check_side_space_right(int num_readings);


	void determine_new_distance_moved();


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
    robot_position current_position;
	
	float initial_distance_moved_left;
	float initial_distance_moved_right;
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