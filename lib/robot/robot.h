#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"
#include "motor.h"
#include "sensors.h"
#include "map.h"

// Radius of wheels from centre of robot to centre of wheels
#define ROBOT_WHEEL_RADIUS 80.0f;


/**
 * @brief A class to act as a wrapper and contain all functions
 * and variables for the main file
 *
 */
class Robot
{
public:

	// FUNCTION DEFINITIONS

    void initial_setup();

    void end();

	/**
	 * @brief Called from the main arduino loop() function
	 * The main function to run the robot and all its processes.
	 * Is continuously called whilst the robot is set to continue running
	 *
	 * @return true		The robot should continue running its program
	 * @return false	The robot stops running
	 */
	void start();

	/**
	 * @brief Called when the robot must run its main algorithm
	 *
	 * @return true		The robot should continue running its program
	 * @return false	The robot stops running
	 */
	bool run();

	void setup();

    void drive_forwards();

	/**
	 * @brief Stops the robot from moving
	 *
	 */
	void stop();

	// Calculate the starting position as coordinates of the robot
	void calculate_starting_location();

    void move_robot(float distance_to_move);

    /**
     * @brief Rotates the robot about a point a set number of degrees
     *
     * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
     */
    void rotate_robot(int degrees);


    // VARIABLE DEFINITIONS

    enum robot_state
	{
		STATE_SETUP,
		STATE_LOCATE,
		STATE_STOP,
		STATE_FORWARD,
		STATE_BACKWARD,
		STATE_LEFT,
		STATE_RIGHT,
	} current_state;

private:
	// FUNCTION DEFINITIONS
	/**
	 * @brief Checks with the sensors whether objects are within
	 * range of the robot
	 *
	 */
	void detect_obstacle();

	// enum object_detected detect_obstacle();

	/**
	 * @brief Tell the robot to avoid the obstacles
	 * that were detected in the detect_obstacle() function
	 *
	 */
	void avoid_obstacle();

	/**
 	* @brief Inserts the starting location into the occupancy grid
 	*/
	void initialise_starting_location_in_map();

    void centre_on_map_grid();

	
    void update_bearing(int angle_to_add);
	


	// VARIABLE DEFINITIONS

	enum object_detected
	{
		front_ir,
		back_ir,
		left_usonic,
		right_usonic
	};

	/**
	 * @brief Integer to represent which objects are within range of the robot
	 *
	 */
	unsigned int objects;

	// Bearing of the robot in relation to maze
	int bearing;

    struct robot_position
    {
        // Float coordinate positions for the absolute position of the robot
        float x_coordinate;
        float y_coordinate;
    } current_position;

};

#endif