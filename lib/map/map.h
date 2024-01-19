#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "errorflag.h"

// Default Maze Size for Labs
#define MAP_WIDTH_X 32
#define MAP_HEIGHT_Y 42

// Maze size for 2cm grids
// #define MAP_WIDTH_X 77
// #define MAP_HEIGHT_Y 102

// Reduced Maze Size for Home
// 100cm width
// 130cm height
// #define MAP_WIDTH_X 22
// #define MAP_HEIGHT_Y 28

// Reduced Maze Size for Home York
// 80cm Width
// 100cm Height
// #define MAP_WIDTH_X 16
// #define MAP_HEIGHT_Y 20

#define MAP_GRID_SIZE 5

#define FINISH 5
#define OBSTACLE 4
#define BORDER 3
#define PARTIAL_BORDER 2
#define ROBOT 1
#define FREE 0

#define MAX_POSITION_HISTORY 200


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

    int checkNextGridSpace(int direction);

    bool checkIfReachedFinish();

    int retraceStepBack();

    void setTrackRobot(bool track);

    bool getTrackRobot();

    void setRobotStartingLocation(float xPos, float yPos);

    int getRobotHistoryCount();


private:

    // FUNCTION DEFINITIONS

    void calculateShortestPath();

    float calculateLengthToFinish(int xPos, int yPos);

    void addObstacleBorder(int xCoord, int yCoord, int extent, int partialBorder);


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

    bool trackRobot;

    GridPosition robotStartLocation;



public:
    GridPosition getPositionInMap();



};


#endif