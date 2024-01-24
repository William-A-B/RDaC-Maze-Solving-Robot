#include "map.h"


/**
 * @brief Construct a new Map object
 * Iniialises all variables, sets the finish position 
 * and makes sure the occupancy grid is setup.
 */
Map::Map()
{
	robotCurrentPosition.xGridSquare = 0;
	robotCurrentPosition.yGridSquare = 0;

	robotStartLocation.xGridSquare = 0;
	robotStartLocation.yGridSquare = 0;

	robotHistoryCount = 0;

	trackRobot = true;

	// Add finishing position
	// mazeFinish.xGridSquare = 15;
	// mazeFinish.yGridSquare = 39;
	mazeFinish.xGridSquare = (MAP_WIDTH_X / 2)+1;
	mazeFinish.yGridSquare = MAP_HEIGHT_Y - 6;
	// mazeFinish.xGridSquare = 10;
	// mazeFinish.yGridSquare = 12;

	// Setup the occupancy grid array to initialise the maze prior to the robot moving
	setupOccupancyGrid();
}

/**
 * @brief Initialises the robot current position at the given values
 * 
 * @param xGridSquareInitial Initial X grid square
 * @param yGridSquareInitial Initial Y grid square
 */
void Map::initialSetup(int xGridSquareInitial, int yGridSquareInitial)
{
	robotCurrentPosition.xGridSquare = xGridSquareInitial;
	robotCurrentPosition.yGridSquare = yGridSquareInitial;
}

/**
 * @brief Setup the occupancy grid with all outer walls, and free space in the middle
 * Add the finish position into the map for easier visualisation
 */
void Map::setupOccupancyGrid()
{
	// Set all values in occupancy grid to 0
	for (int i = 0; i < MAP_WIDTH_X; i++)
	{
		for (int j = 0; j < MAP_HEIGHT_Y; j++)
		{
			this->occupancyGrid[i][j] = FREE;
		}
	}

	// Set all values in the outer perimeter to 1 to represent the edge of the maze
	for (int i = 0; i < MAP_WIDTH_X; i++)
	{
		this->occupancyGrid[i][0] = OBSTACLE;
		this->occupancyGrid[i][MAP_HEIGHT_Y-1] = OBSTACLE;
	}

	for (int i = 0; i < MAP_HEIGHT_Y; i++)
	{
		this->occupancyGrid[0][i] = OBSTACLE;
		this->occupancyGrid[MAP_WIDTH_X-1][i] = OBSTACLE;
	}

	// Add finish position into the map to be able to visualise it
	occupancyGrid[mazeFinish.xGridSquare][mazeFinish.yGridSquare] = FINISH;
}

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
void Map::addObstaclesToMap(float frontSensorDistance, float backSensorDistance, float leftSensorDistance, float rightSensorDistance, int robotBearing)
{
	// Convert sensor distances to grid spaces
	int objRelPos_f = frontSensorDistance / MAP_GRID_SIZE;
	int objRelPos_b = backSensorDistance / MAP_GRID_SIZE;
	int objRelPos_l = leftSensorDistance / MAP_GRID_SIZE;
	int objRelPos_r = rightSensorDistance / MAP_GRID_SIZE;

	// Assign current robot position to simpler variable names for array indexing
	const int rPosX = robotCurrentPosition.xGridSquare;
	const int rPosY = robotCurrentPosition.yGridSquare;

	// The number of grid squares that the borders extend out of obstacles.
	int borderExtent = 2;

	/**
	 * @brief add obstacles and borders & partial borders based on the current orientation
	 * @param partialBorderDir - directions
	 * 
	 */
	if (robotBearing == 0)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_f, borderExtent, NORTH);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_b, borderExtent, SOUTH);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_l, rPosY, borderExtent, WEST);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_r, rPosY, borderExtent, EAST);
	}
	else if (robotBearing == 90)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_f, rPosY, borderExtent, EAST);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_b, rPosY, borderExtent, WEST);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_l, borderExtent, NORTH);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_r, borderExtent, SOUTH);
	}
	else if (robotBearing == 180)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_f, borderExtent, SOUTH);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_b, borderExtent, NORTH);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_l, rPosY, borderExtent, EAST);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_r, rPosY, borderExtent, WEST);
	}
	else if (robotBearing == 270)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_f, rPosY, borderExtent, WEST);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_b, rPosY, borderExtent, EAST);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_l, borderExtent, SOUTH);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_r, borderExtent, NORTH);
	}
}

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
void Map::addObstacleBorder(int xCoord, int yCoord, int extent, int partialBorderDir)
{
	for (int x = xCoord-extent; x <= xCoord+extent; x++)
	{
		//Change tox > 0 and Map_width_x - 1 because the outer edge is always obstacles
		if (x > 0 || x < MAP_WIDTH_X-1)
		{
			for (int y = yCoord-extent; y <= yCoord+extent; y++)
			{
				// Change to y> 0 and  Map_width_y - 1 because the outer edge is always obstacles
				if (y > 0 || y < MAP_HEIGHT_Y-1)
				{
					if (x == xCoord && y == yCoord)
					{
						occupancyGrid[x][y] = OBSTACLE;
					}
					else
					{
						if (occupancyGrid[x][y] != OBSTACLE && occupancyGrid[x][y] != ROBOT)
						{
							if (partialBorderDir == NORTH && x == xCoord && y < yCoord)
							{
								occupancyGrid[x][y] = BORDER;
							}
							else if (partialBorderDir == EAST && y == yCoord && x < xCoord)
							{
								occupancyGrid[x][y] = BORDER;
							}
							else if (partialBorderDir == SOUTH && x == xCoord && y > yCoord)
							{
								occupancyGrid[x][y] = BORDER;
							}
							else if (partialBorderDir == WEST && y == yCoord && x > xCoord)
							{
								occupancyGrid[x][y] = BORDER;
							}
							else 
							{
								if (occupancyGrid[x][y] != BORDER)
									occupancyGrid[x][y] = PARTIAL_BORDER;
							}
						}
					}
				}
			}
		}
	}
}

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
void Map::updateRobotPosition(float robotXCoord, float robotYCoord)
{
	robotCurrentPosition.xGridSquare = (robotXCoord / MAP_GRID_SIZE) + 1;
	robotCurrentPosition.yGridSquare = (robotYCoord / MAP_GRID_SIZE) + 1;

	if (trackRobot == true)
	{
		robotPositionHistory[robotHistoryCount].xGridSquare = robotCurrentPosition.xGridSquare;
		robotPositionHistory[robotHistoryCount].yGridSquare = robotCurrentPosition.yGridSquare;
		
		occupancyGrid[robotCurrentPosition.xGridSquare][robotCurrentPosition.yGridSquare] = ROBOT;

		robotHistoryCount++;

		if (robotHistoryCount >= MAX_POSITION_HISTORY)
		{
			robotHistoryCount = 0;
		}
	}

	

	Serial.println("\nupdateRobotPosition");
	Serial.print("( ");
	Serial.print(robotCurrentPosition.xGridSquare);
	Serial.print(", ");
	Serial.print(robotCurrentPosition.yGridSquare);
	Serial.println(" )");
}

/**
 * @brief Prints the occupancy map to the serial port
 * and displays the occupancy map in a readable format
 *
 */
void Map::displayMap()
{
	Serial.println("\nOCCUPANCY GRID MAP\n");
	// sbsb should be starting at -1
	for (int y = MAP_HEIGHT_Y-1; y >= 0; y--)
	{
		for (int x = 0; x < MAP_WIDTH_X; x++)
		{
			Serial.print(this->occupancyGrid[x][y]);
		}
		Serial.print("\n");
	}
}

/**
 * @brief Prints out the robot position hitory array
 * Allows you to see all the past map grid positions the robot has been in
 */
void Map::displayRobotHistory()
{
	for (int historyIndex = 0; historyIndex < 100; historyIndex++)
	{
		Serial.print((String)"\nRobot Position: "+historyIndex);
		Serial.print(" --> ");
		Serial.print(robotPositionHistory[historyIndex].xGridSquare);
		Serial.print(", ");
		Serial.print(robotPositionHistory[historyIndex].yGridSquare);
	}
}

GridPosition Map::getPositionInMap()
{
    return this->robotCurrentPosition;
}

/**
 * @brief Gets the grid position that the maze finish is located at
 * 
 * @return GridPosition - the grid position that the finish is located at
 */
GridPosition Map::getMazeFinishPosition()
{
	return mazeFinish;
}

/**
 * @brief determine the shortest distance to the finish location from three points around the robot
 * Determine which of the three grid squares in front of, to the left of or to the right of the robot
 * is closes to the finish position and return the distance to the finish from that point.
 * 
 * @param direction     - The direction the robot is heading in
 * @return float        - The distance to the finish from each of the points
 */
float Map::determineDistanceToFinish(int direction)
{
	if (direction >= 4)
	{
		direction = direction - 4;
	}

	// Direction 0 = NORTH
	// 1 = EAST
	// 2 = SOUTH
	// 3 = WEST
	if (direction == 0)
		return calculateLengthToFinish(robotCurrentPosition.xGridSquare, robotCurrentPosition.yGridSquare+1);
	else if(direction == 1)
		return calculateLengthToFinish(robotCurrentPosition.xGridSquare+1, robotCurrentPosition.yGridSquare);
	else if(direction == 2)
		return calculateLengthToFinish(robotCurrentPosition.xGridSquare, robotCurrentPosition.yGridSquare-1);
	else if(direction == 3)
		return calculateLengthToFinish(robotCurrentPosition.xGridSquare-1, robotCurrentPosition.yGridSquare);
	else
		return 99999.9f; // Should never get here. Reset to max value to make sure route not taken
}

/**
 * @brief calculates the length to the finish from the given position
 * 
 * @param xPos      - The x position to calculate the distance from
 * @param yPos      - The y position to calculate the distance from
 * @return float    - The distance to the finish from the given position
 */
float Map::calculateLengthToFinish(int xPos, int yPos)
{
	return sqrtf(powf((float)(mazeFinish.xGridSquare - xPos), 2.0f) + powf((float)(mazeFinish.yGridSquare - yPos), 2.0f));
}

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
int Map::checkNextGridSpace(int direction)
{
	if (direction >= 4)
	{
		direction = direction - 4;
	}
	// Serial.println("Map when checking next grid space"); // TEMP
	// displayMap();

	

	Serial.println("\nChecking Next Grid Square\n");
	Serial.print(direction);
	Serial.print(", ( ");
	Serial.print(robotCurrentPosition.xGridSquare);
	Serial.print(", ");
	Serial.print(robotCurrentPosition.yGridSquare);
	Serial.println(" )");



	if (direction == 0)
	{
		int occupancy = occupancyGrid[robotCurrentPosition.xGridSquare][robotCurrentPosition.yGridSquare+1];
		if (occupancy == FREE || occupancy == ROBOT || occupancy == FINISH)
		{
			Serial.println("No border or obstacle");
			return 1;
		}
		else if (occupancy == PARTIAL_BORDER)
		{
			Serial.println("Partial Border");
			return 2;
		}

		if (occupancy == BORDER)
			Serial.println("Border North");
		if (occupancy == OBSTACLE)
			Serial.println("Obstacle North");
	}
	else if (direction == 1)
	{
		int occupancy = occupancyGrid[robotCurrentPosition.xGridSquare+1][robotCurrentPosition.yGridSquare];
		if (occupancy == FREE || occupancy == ROBOT || occupancy == FINISH)
		{
			Serial.println("No border or obstacle");
			return 1;
		}
		else if (occupancy == PARTIAL_BORDER)
		{
			Serial.println("Partial Border");
			return 2;
		}

		if (occupancy == BORDER)
			Serial.println("Border East");
		if (occupancy == OBSTACLE)
			Serial.println("Obstacle East");
	}
	else if (direction == 2)
	{
		int occupancy = occupancyGrid[robotCurrentPosition.xGridSquare][robotCurrentPosition.yGridSquare-1];
		if (occupancy == FREE || occupancy == ROBOT || occupancy == FINISH)
		{
			Serial.println("No border or obstacle");
			return 1;
		}
		else if (occupancy == PARTIAL_BORDER)
		{
			Serial.println("Partial Border");
			return 2;
		}

		if (occupancy == BORDER)
			Serial.println("Border South");
		if (occupancy == OBSTACLE)
			Serial.println("Obstacle South");
	}
	else if (direction == 3)
	{
		int occupancy = occupancyGrid[robotCurrentPosition.xGridSquare-1][robotCurrentPosition.yGridSquare];
		if (occupancy == FREE || occupancy == ROBOT || occupancy == FINISH)
		{
			Serial.println("No border or obstacle");
			return 1;
		}
		else if (occupancy == PARTIAL_BORDER)
		{
			Serial.println("Partial Border");
			return 2;
		}

		if (occupancy == BORDER)
			Serial.println("Border West");
		if (occupancy == OBSTACLE)
			Serial.println("Obstacle West");
	}
	else
	{	
		return 0;
	}
	return 0;
}

/**
 * @brief Checks if the current robot position is at the known finish position
 * 
 * @return true     - If the robot is at the finish
 * @return false    - If the robot is not yet at the finish
 */
bool Map::checkIfReachedFinish()
{
	const int currentPosX = robotCurrentPosition.xGridSquare;
	const int currentPosY = robotCurrentPosition.yGridSquare;

	const int finishX = mazeFinish.xGridSquare;
	const int finishY = mazeFinish.yGridSquare;

	if (currentPosX == finishX && currentPosY == finishY)
		return true;
	else if (currentPosX == finishX && currentPosY == finishY+1)
		return true;
	else if (currentPosX == finishX+1 && currentPosY == finishY+1)
		return true;
	else if (currentPosX == finishX+1 && currentPosY == finishY)
		return true;
	else if (currentPosX == finishX+1 && currentPosY == finishY-1)
		return true;
	else if (currentPosX == finishX && currentPosY == finishY-1)
		return true;
	else if (currentPosX == finishX-1 && currentPosY == finishY-1)
		return true;
	else if (currentPosX == finishX-1 && currentPosY == finishY)
		return true;
	else if (currentPosX == finishX-1 && currentPosY == finishY+1)
		return true;
	else
		return false;

	return false;
}

/**
 * @brief Checks the robot position history array and iterates back one step to determine
 * the direction the robot needs to turn and move in to retrace one step back
 * 
 * @return int  - The direction to turn towards, or if it has reached the start position
 * -1 if robot should remain in the same square
 */
int Map::retraceStepBack()
{
	if (robotHistoryCount == 0)
	{
		return 5;
	}

	// Decrease robot history count
	Serial.println("\nRobot History Count");
	Serial.println(robotHistoryCount);
	robotHistoryCount--;

	// Get the next grid to move to on the route back
	int nextX = robotPositionHistory[robotHistoryCount].xGridSquare;
	int nextY = robotPositionHistory[robotHistoryCount].yGridSquare;

	int gridSquareDiff = 0;
	int orientation = -1;

	// Y value changes if next x is equal to current x
	if (nextX == robotCurrentPosition.xGridSquare)
	{
		// Y value changed
		Serial.println("Y Value Changed");

		// Calculate difference between next grid square and current grid square
		gridSquareDiff = nextY - robotCurrentPosition.yGridSquare;

		// Determine direction to turn towards
		if (gridSquareDiff == 0)
		{
			// It's at the same location
			orientation = -1;
		}

		if (gridSquareDiff < 0)
		{
			// South
			orientation = 2;
		}
		else if (gridSquareDiff > 0)
		{
			// North
			orientation = 0;
		}
	}
	// X value changes if next y is equal to current y
	else if (nextY == robotCurrentPosition.yGridSquare)
	{
		// X value changed
		Serial.println("X Value Changed");

		// Calculate difference between next grid square and current grid square
		gridSquareDiff = nextX - robotCurrentPosition.xGridSquare;

		// Determine direction to turn towards
		if (gridSquareDiff == 0)
		{
			// It's at the same location
			orientation = -1;
		}

		if (gridSquareDiff < 0)
		{
			// West
			orientation = 3;
		}
		else if (gridSquareDiff > 0)
		{
			// East
			orientation = 1;
		}
	}
	else
	{
		const int gridSquareDiffY = nextY - robotCurrentPosition.yGridSquare;
		const int gridSquareDiffX = nextX - robotCurrentPosition.xGridSquare;

		Serial.println("Both Values Changed");

		if (gridSquareDiffX > 0 && gridSquareDiffY > 0)
		{
			// Diagonal next grid position, in North East direction.
			// Turn North then recalculate, East should be determined
			if (checkForWallsInMap(robotCurrentPosition.xGridSquare, robotCurrentPosition.yGridSquare, 0) == true)
				orientation = 1;
			else
				orientation = 0;
			robotHistoryCount++;
		}
		else if (gridSquareDiffX > 0 && gridSquareDiffY < 0)
		{
			// Diagonal next grid position, in South East Direction
			if (checkForWallsInMap(robotCurrentPosition.xGridSquare, robotCurrentPosition.yGridSquare, 0) == true)
				orientation = 1;
			else
				orientation = 2;
			robotHistoryCount++;
		}
		else if (gridSquareDiffX < 0 && gridSquareDiffY < 0)
		{
			// Diagonal next grid position, in South West direction
			if (checkForWallsInMap(robotCurrentPosition.xGridSquare, robotCurrentPosition.yGridSquare, 0) == true)
				orientation = 3;
			else
				orientation = 2;
			robotHistoryCount++;
		}
		else if (gridSquareDiffX < 0 && gridSquareDiffY > 0)
		{
			// Diagonal next grid postion, in North West direction
			if (checkForWallsInMap(robotCurrentPosition.xGridSquare, robotCurrentPosition.yGridSquare, 0) == true)
				orientation = 3;
			else
				orientation = 0;
			robotHistoryCount++;
		}
		else
		{
			// Still in same square
			orientation = -1;
		}

	}

	return orientation;


}

/**
 * @brief Check for walls in the map when retracing steps back to be able to avoid any obstacles
 * 
 * @param xPos          - The x position to check for walls at
 * @param yPos          - The y position to check for walls at
 * @param direction     - The direction the robot is heading in
 * @return true         - If there is a not a wall, and the robot is allowed to move there.
 * @return false        - If there is a wall, and the robot cannot move there
 */
bool Map::checkForWallsInMap(int xPos, int yPos, int direction)
{
	if (direction == 0)
	{
		if (occupancyGrid[xPos+2][yPos] >= PARTIAL_BORDER)
			return false;
		else
			return true;
	}
	else if (direction == 1)
	{
		if (occupancyGrid[xPos+2][yPos] >= PARTIAL_BORDER)
			return false;
		else
			return true;
	}
	else if (direction == 2)
	{
		if (occupancyGrid[xPos-2][yPos] >= PARTIAL_BORDER)
			return false;
		else
			return true;
	}
	else if (direction == 3)
	{
		if (occupancyGrid[xPos-2][yPos] >= PARTIAL_BORDER)
			return false;
		else
			return true;
	}

	return false;
}

/**
 * @brief Set the Track Robot variable.
 * If true, the robot position history array will be updated with the current robot position
 * If false, the robot position history array will not be updated
 * 
 * @param track   - True or false depending on whether the robot position history array should be updated
 */
void Map::setTrackRobot(bool track)
{
	trackRobot = track;
}

/**
 * @brief Get the Track Robot variable
 * 
 * @return true 
 * @return false 
 */
bool Map::getTrackRobot()
{
	return trackRobot;
}

/**
 * @brief Set the Robot Starting Location object
 * 
 * @param xPos  - The x position of the robot starting location
 * @param yPos  - The y position of the robot starting location
 */
void Map::setRobotStartingLocation(float xPos, float yPos)
{
	robotStartLocation.xGridSquare = xPos / 5;
	robotStartLocation.yGridSquare = yPos / 5;
}

/**
 * @brief Get the Robot History Count variable
 * Indicates which index the latest position in the robot position history array is at.
 * 
 * @return int  - The index of the latest position in the robot position history array
 */
int Map::getRobotHistoryCount()
{
	return robotHistoryCount;
}