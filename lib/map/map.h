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

#define OBSTACLE 3
#define BORDER 2
#define ROBOT 1
#define FREE 0


struct GridPosition
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

    GridPosition getMazeFinishPosition();

    void getMazeFinishPosition(float &xPos, float &yPos);

    float determineDistanceToFinish(int direction);

    bool checkNextGridSpace(int direction);


private:

    // FUNCTION DEFINITIONS

    void calculateShortestPath();

    float calculateLengthToFinish(int xPos, int yPos);

    void addObstacleBorder(int xCoord, int yCoord, int extent);


    // VARIABLE DEFINITIONS

    GridPosition robotCurrentPosition;
    

    /**
	 * @brief The Occupancy grid which is used to map out all objects within the map
	 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
	 *
	 */
	int occupancyGrid[MAP_WIDTH_X][MAP_HEIGHT_Y] = {0};

    GridPosition robotPositionHistory[100];

    int robotHistoryCount;

    GridPosition mazeFinish;



public:
    GridPosition getPositionInMap();



};


#endif