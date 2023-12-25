#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "errorflag.h"

#define X_MAX 32
#define Y_MAX 44


struct RobotPositionInMap
{
    // Robot y position in occupancy grid
    int yCoordinate;
    // Robot x position in occupancy grid
    int xCoordinate;
};

class Map
{

public:
    // CONSTRUCTOR
    /**
     * @brief Construct a new Map object
     * 
     * @param xCoordinate Robot x starting position in map
     * @param yCoordinate Robot y starting position in map
     */
    Map(int xCoordinate, int yCoordinate);

    // FUNCTION DEFINITIONS
    /**
	 * @brief Setup the occupancy grid
	 *
	 */
	void setupOccupancyGrid();

    void addObstaclesToMap(int xCoordinate, int yCoordinate);

    /**
	 * @brief Prints the occupancy map to the serial port
	 * and displays the occupancy map in a readable format
	 *
	 */
	void displayMap();

    void setRobotLocation(int xCoordinate, int yCoordinate);

    bool checkRouteAheadInMap(int bearing_heading, int distance_to_move);



    

private:

    // FUNCTION DEFINITIONS



    // VARIABLE DEFINITIONS

    RobotPositionInMap currentPosition;
    

    /**
	 * @brief The Occupancy grid which is used to map out all objects within the map
	 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
	 *
	 */
	bool occupancyGrid[32][44] = {0};

public:
    RobotPositionInMap getPositionInMap();


};


#endif