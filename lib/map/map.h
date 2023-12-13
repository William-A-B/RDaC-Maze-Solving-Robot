#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <mbed/mbed.h>

#define X_MAX 32
#define Y_MAX 44


struct robot_position
{
    // Robot y position in occupancy grid
    int y_coordinate;
    // Robot x position in occupancy grid
    int x_coordinate;
};

class Map
{

public:
    // CONSTRUCTOR
    /**
     * @brief Construct a new Map object
     * 
     * @param x_coordinate Robot x starting position in map
     * @param y_coordinate Robot y starting position in map
     */
    Map(int x_coordinate, int y_coordinate);

    // FUNCTION DEFINITIONS
    /**
	 * @brief Setup the occupancy grid
	 *
	 */
	void setup_occupancy_grid();

    void add_obstacles_to_map(int x_coordinate, int y_coordinate);

    /**
	 * @brief Prints the occupancy map to the serial port
	 * and displays the occupancy map in a readable format
	 *
	 */
	void display_map();

    void set_robot_location(int x_coordinate, int y_coordinate);

    bool check_route_ahead(int bearing_heading, int distance_to_move);



    

private:

    // FUNCTION DEFINITIONS



    // VARIABLE DEFINITIONS

    robot_position current_position;
    

    /**
	 * @brief The Occupancy grid which is used to map out all objects within the map
	 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
	 *
	 */
	bool occupancy_grid[32][44] = {0};

public:
    robot_position get_position_in_map();


};


#endif