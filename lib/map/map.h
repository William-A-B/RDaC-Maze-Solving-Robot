#ifndef MAP_H
#define MAP_H

/**
 * @file map.h
 * @author Y3905304
 * @brief Header file for map class, contains all information related to the map for the maze
 * Contains functions to handle adding obstacles to the map and checking the route the robot can move
 * 
 */

#include <Arduino.h>
#include <mbed/mbed.h>
#include "errorflag.h"

// Default Maze Size for Labs - for 5cm grid squares
#define MAP_WIDTH_X 32
#define MAP_HEIGHT_Y 42

// Maze size for 2cm grids
// #define MAP_WIDTH_X 77
// #define MAP_HEIGHT_Y 102

// Size of the grids in the occupancy grid
#define MAP_GRID_SIZE 5

// The values of various objects that are placed in the map.
#define FINISH 5
#define OBSTACLE 4
#define BORDER 3
#define PARTIAL_BORDER 2
#define ROBOT 1
#define FREE 0

// Max size of the robot position history array
// 200 provides enough space to store all the movements the robot will make from start to finish
#define MAX_POSITION_HISTORY 200

// Compass directions defined for easier to read code
// NORTH is up in the maze
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3


/**
 * @brief A struct to contain an integer grid coordinate position
 */
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
    
    /**
     * @brief Construct a new Map object
     * Iniialises all variables, sets the finish position 
     * and makes sure the occupancy grid is setup.
     */
    Map();

    // FUNCTION DEFINITIONS

    /**
     * @brief Initialises the robot current position at the given values
     * 
     * @param xGridSquareInitial Initial X grid square
     * @param yGridSquareInitial Initial Y grid square
     */
    void initialSetup(int xGridSquareInitial, int yGridSquareInitial);

    /**
	 * @brief Setup the occupancy grid with all outer walls, and free space in the middle
     * Add the finish position into the map for easier visualisation
	 */
	void setupOccupancyGrid();

    /**
     * @brief Adds obstacles into the map at the given distances from the robot
     * Positions within the array are determined by the current robot bearing
     * 
     * @param frontSensorDistance	The distance to an obstacle in front of the robot
     * @param backSensorDistance 	The distance to an obstacle behind the robot
     * @param leftSensorDistance 	The distance to an obstacle to the left of the robot
     * @param rightSensorDistance 	The distance to an obstacle to the right of the robot
     * @param robotBearing 			The current bearing that the robot is heading in
     */
    void addObstaclesToMap(float frontSensorDistance, float backSensorDistance, float leftSensorDistance, float rightSensorDistance, int robotBearing);

    /**
     * @brief Adds all the required obstacles and borders at the given coordinates
     * Obstacles are placed as one obstacle in a single grid square at a time, 
     * and borders are placed around the obstacles with a radius equal to the extent (default = 2) 
     * Partial borders allow the robot to travel forwards through the border, but not to turn right or left into the border.
     * For full borders, the robot cannot turn left or right or forwards into them.
     * 
     * @param xCoord 			The x coordinate of the obstacle to add
     * @param yCoord 			The y coordinate of the obstacle to add
     * @param extent 			The extent that the borders around the obstacle extend outwards (radius equal to number of grid squares)
     * @param partialBorderDir 	The direction that the robot is travelling in so it is known whereabouts to add the partial borders.
     */
    void updateRobotPosition(float robotXCoord, float robotYCoord);

    /**
	 * @brief Prints the occupancy map to the serial port
	 * and displays the occupancy map in a readable format
	 *
	 */
	void displayMap();

    /**
     * @brief Prints out the robot position hitory array
     * Allows you to see all the past map grid positions the robot has been in
     */
    void displayRobotHistory();

    /**
     * @brief Gets the grid position that the maze finish is located at
     * 
     * @return GridPosition - the grid position that the finish is located at
     */
    GridPosition getMazeFinishPosition();

    /**
     * @brief determine the shortest distance to the finish location from three points around the robot
     * Determine which of the three grid squares in front of, to the left of or to the right of the robot
     * is closes to the finish position and return the distance to the finish from that point.
     * 
     * @param direction     - The direction the robot is heading in
     * @return float        - The distance to the finish from each of the points
     */
    float determineDistanceToFinish(int direction);

    /**
     * @brief Checks three grid squares around the robot.
     * The front, left and right grid squares and checked to see which one is free and can be moved into
     * The closest of these to the finish position that is free is then moved into by the robot 
     * 
     * @param direction     - Direction the robot is heading in
     * @return int          - Returns a value depending on what the grid square is occupied by
     * 0 - can't move
     * 1 - Free space
     * 2 - Partial border
     */
    int checkNextGridSpace(int direction);

    /**
     * @brief Checks if the current robot position is at the known finish position
     * 
     * @return true     - If the robot is at the finish
     * @return false    - If the robot is not yet at the finish
     */
    bool checkIfReachedFinish();

    /**
     * @brief Checks the robot position history array and iterates back one step to determine
     * the direction the robot needs to turn and move in to retrace one step back
     * 
     * @return int  - The direction to turn towards, or if it has reached the start position
     * -1 if robot should remain in the same square
     */
    int retraceStepBack();

    /**
     * @brief Set the Track Robot variable.
     * If true, the robot position history array will be updated with the current robot position
     * If false, the robot position history array will not be updated
     * 
     * @param track   - True or false depending on whether the robot position history array should be updated
     */
    void setTrackRobot(bool track);

    /**
     * @brief Get the Track Robot variable
     * 
     * @return true 
     * @return false 
     */
    bool getTrackRobot();

    /**
     * @brief Set the Robot Starting Location object
     * 
     * @param xPos  - The x position of the robot starting location
     * @param yPos  - The y position of the robot starting location
     */
    void setRobotStartingLocation(float xPos, float yPos);

    /**
     * @brief Get the Robot History Count variable
     * Indicates which index the latest position in the robot position history array is at.
     * 
     * @return int  - The index of the latest position in the robot position history array
     */
    int getRobotHistoryCount();


private:

    // FUNCTION DEFINITIONS

    /**
     * @brief calculates the length to the finish from the given position
     * 
     * @param xPos      - The x position to calculate the distance from
     * @param yPos      - The y position to calculate the distance from
     * @return float    - The distance to the finish from the given position
     */
    float calculateLengthToFinish(int xPos, int yPos);

    /**
     * @brief Adds all the required obstacles and borders at the given coordinates
     * Obstacles are placed as one obstacle in a single grid square at a time, 
     * and borders are placed around the obstacles with a radius equal to the extent (default = 2) 
     * Partial borders allow the robot to travel forwards through the border, but not to turn right or left into the border.
     * For full borders, the robot cannot turn left or right or forwards into them.
     * 
     * @param xCoord 			The x coordinate of the obstacle to add
     * @param yCoord 			The y coordinate of the obstacle to add
     * @param extent 			The extent that the borders around the obstacle extend outwards (radius equal to number of grid squares)
     * @param partialBorderDir 	The direction that the robot is travelling in so it is known whereabouts to add the partial borders.
     */
    void addObstacleBorder(int xCoord, int yCoord, int extent, int partialBorder);

    /**
     * @brief Check for walls in the map when retracing steps back to be able to avoid any obstacles
     * 
     * @param xPos          - The x position to check for walls at
     * @param yPos          - The y position to check for walls at
     * @param direction     - The direction the robot is heading in
     * @return true         - If there is a not a wall, and the robot is allowed to move there.
     * @return false        - If there is a wall, and the robot cannot move there
     */
    bool checkForWallsInMap(int xPos, int yPos, int direction);


    // VARIABLE DEFINITIONS

    // The current position of the robot in the occupancy grid as an integer grid coordinate position.
    GridPosition robotCurrentPosition;
    

    /**
	 * @brief The Occupancy grid which is used to map out all objects within the map
	 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
	 *
	 */
	int occupancyGrid[MAP_WIDTH_X][MAP_HEIGHT_Y] = {0};

    // The robot position history array, stores the history of the positions the robot has been to
    GridPosition robotPositionHistory[MAX_POSITION_HISTORY];

    // The index of the latest position in the robot position history array
    int robotHistoryCount;

    // The position of the maze finish in the occupancy grid
    GridPosition mazeFinish;

    // Whether the robot position history array should be updated or not
    bool trackRobot;

    // The starting location of the robot in the occupancy grid
    GridPosition robotStartLocation;



public:
    // The current position of the robot in the occupancy grid as an integer grid coordinate position.
    GridPosition getPositionInMap();



};


#endif