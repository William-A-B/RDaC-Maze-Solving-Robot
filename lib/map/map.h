#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "errorflag.h"

// Default Maze Size
// #define MAP_WIDTH_X 32
// #define MAP_HEIGHT_Y 44

// Reduced Maze Size
// 100cm width
// 130cm height
#define MAP_WIDTH_X 22
#define MAP_HEIGHT_Y 28


struct RobotPosition
{
    // Robot y position in occupancy grid
    int yGridSquare;
    // Robot x position in occupancy grid
    int xGridSquare;
};

class Map
{

public:
    // CONSTRUCTOR
    
    Map();

    // FUNCTION DEFINITIONS

    /**
     * @brief Initial setup for the map.
     * 
     * @param xGridSquareInitial Initialise x grid square to a value
     * @param yGridSquareInitial Initialise y grid square to a value
     */
    void initialSetup(int xGridSquareInitial, int yGridSquareInitial);

    /**
	 * @brief Setup the occupancy grid
	 *
	 */
	void setupOccupancyGrid();

    void addObstaclesToMap(float frontSensorDistance, float backSensorDistance, float leftSensorDistance, float rightSensorDistance, int robotBearing);

    void updateRobotPosition(float robotXCoord, float robotYCoord);

    /**
	 * @brief Prints the occupancy map to the serial port
	 * and displays the occupancy map in a readable format
	 *
	 */
	void displayMap();

    void displayRobotHistory();

    bool checkRouteAheadInMap(int bearing_heading, int distance_to_move);



    

private:

    // FUNCTION DEFINITIONS



    // VARIABLE DEFINITIONS

    RobotPosition robotCurrentPosition;
    

    /**
	 * @brief The Occupancy grid which is used to map out all objects within the map
	 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
	 *
	 */
	int occupancyGrid[MAP_WIDTH_X][MAP_HEIGHT_Y] = {0};

    RobotPosition robotPositionHistory[100];

    int robotHistoryCount;



public:
    RobotPosition getPositionInMap();


};


#endif