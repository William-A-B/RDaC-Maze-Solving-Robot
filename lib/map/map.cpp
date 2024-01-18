#include "map.h"



Map::Map()
{
	robotCurrentPosition.xGridSquare = 0;
	robotCurrentPosition.yGridSquare = 0;

	robotHistoryCount = 0;

	// Add finishing position
	// mazeFinish.xGridSquare = 15;
	// mazeFinish.yGridSquare = 39;
	mazeFinish.xGridSquare = (MAP_WIDTH_X / 2);
	mazeFinish.yGridSquare = MAP_HEIGHT_Y - 4;

	// Setup the occupancy grid array to initialise the maze prior to the robot moving
	setupOccupancyGrid();
}

void Map::initialSetup(int xGridSquareInitial, int yGridSquareInitial)
{
	robotCurrentPosition.xGridSquare = xGridSquareInitial;
	robotCurrentPosition.yGridSquare = yGridSquareInitial;
}

/**
 * @brief The Occupancy grid which is used to map out all objects within the map
 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
 *
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
	occupancyGrid[mazeFinish.xGridSquare][mazeFinish.yGridSquare] = 5;
}


void Map::addObstaclesToMap(float frontSensorDistance, float backSensorDistance, float leftSensorDistance, float rightSensorDistance, int robotBearing)
{
	int objRelPos_f = frontSensorDistance / MAP_GRID_SIZE;
	int objRelPos_b = backSensorDistance / MAP_GRID_SIZE;
	int objRelPos_l = leftSensorDistance / MAP_GRID_SIZE;
	int objRelPos_r = rightSensorDistance / MAP_GRID_SIZE;

	// Add 1 to adjust for occupancy grid walls
	const int rPosX = robotCurrentPosition.xGridSquare;
	const int rPosY = robotCurrentPosition.yGridSquare;

	int borderExtent = 2;

	/**
	 * @brief add obstacles and borders & partial borders based on the current orientation
	 * @param partialBorder - directions
	 * 0 - North
	 * 1 - South
	 * 2 - West
	 * 3 - East
	 * 
	 */
	if (robotBearing == 0)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_f, borderExtent, 0);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_b, borderExtent, 1);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_l, rPosY, borderExtent, 2);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_r, rPosY, borderExtent, 3);
	}
	else if (robotBearing == 90)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_f, rPosY, borderExtent, 3);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_b, rPosY, borderExtent, 2);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_l, borderExtent, 0);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_r, borderExtent, 1);
	}
	else if (robotBearing == 180)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_f, borderExtent, 1);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_b, borderExtent, 0);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_l, rPosY, borderExtent, 3);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_r, rPosY, borderExtent, 2);
	}
	else if (robotBearing == 270)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_f, rPosY, borderExtent, 2);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_b, rPosY, borderExtent, 3);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_l, borderExtent, 1);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_r, borderExtent, 0);
	}
}

void Map::addObstacleBorder(int xCoord, int yCoord, int extent, int partialBorder)
{
	for (int x = xCoord-extent; x <= xCoord+extent; x++)
	{
		// sbsb Change tox > 0 and Map_width_x - 1 because the outer edge is always obstacles
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
							// if (partialBorder == 0 && y >= yCoord)
							// {	
							// 	occupancyGrid[x][y] = PARTIAL_BORDER;
							// }
							// else if (partialBorder == 1 && y <= yCoord)
							// {
							// 	occupancyGrid[x][y] = PARTIAL_BORDER;
							// }
							// else if (partialBorder == 2 && x <= xCoord)
							// {
							// 	occupancyGrid[x][y] = PARTIAL_BORDER;
							// }
							// else if (partialBorder == 3 && x >= xCoord)
							// {
							// 	occupancyGrid[x][y] = PARTIAL_BORDER;
							// }
							// else
							// {
							// 	occupancyGrid[x][y] = BORDER;
							// }

							if (partialBorder == 0 && x == xCoord)
							{
								occupancyGrid[x][y] = BORDER;
							}
							else if (partialBorder == 1 && x == xCoord)
							{
								occupancyGrid[x][y] = BORDER;
							}
							else if (partialBorder == 2 && y == yCoord)
							{
								occupancyGrid[x][y] = BORDER;
							}
							else if (partialBorder == 3 && y == yCoord)
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

void Map::updateRobotPosition(float robotXCoord, float robotYCoord)
{
	robotCurrentPosition.xGridSquare = (robotXCoord / MAP_GRID_SIZE) + 1;
	robotCurrentPosition.yGridSquare = (robotYCoord / MAP_GRID_SIZE) + 1;

	robotPositionHistory[robotHistoryCount].xGridSquare = robotCurrentPosition.xGridSquare;
	robotPositionHistory[robotHistoryCount].yGridSquare = robotCurrentPosition.yGridSquare;

	occupancyGrid[robotCurrentPosition.xGridSquare][robotCurrentPosition.yGridSquare] = ROBOT;

	robotHistoryCount++;

	if (robotHistoryCount >=100)
	{
		robotHistoryCount = 0;
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

bool Map::checkRouteAheadInMap(int bearingHeading, int distance_to_move)
{
	// Check robot current bearing in map

	// Search along direction in map to check for objects

	// If object along axis check if its greater than the distance wanting to move

	// Return true if can move forwards

	// Else return false

    // Get current robot position
    // Check direction wanting to move in
    // Check the free space ahead within the space wanting to move in
    // return true or false whether the space ahead is free to move in



	if (bearingHeading == 0)
	{
		
	}

	return false;
}


GridPosition Map::getPositionInMap()
{
    return this->robotCurrentPosition;
}

GridPosition Map::getMazeFinishPosition()
{
	return mazeFinish;
}

void Map::getMazeFinishPosition(float &xPos, float &yPos)
{
	xPos = (float)(mazeFinish.xGridSquare * MAP_GRID_SIZE);
	yPos = (float)(mazeFinish.yGridSquare * MAP_GRID_SIZE);
}

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

float Map::calculateLengthToFinish(int xPos, int yPos)
{
	return sqrtf(powf((float)(mazeFinish.xGridSquare - xPos), 2.0f) + powf((float)(mazeFinish.yGridSquare - yPos), 2.0f));
}

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
		if (occupancy == FREE || occupancy == ROBOT)
		{
			Serial.println("No border or obstacle");
			return true;
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
		if (occupancy == FREE || occupancy == ROBOT)
		{
			Serial.println("No border or obstacle");
			return true;
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
		if (occupancy == FREE || occupancy == ROBOT)
		{
			Serial.println("No border or obstacle");
			return true;
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
		if (occupancy == FREE || occupancy == ROBOT)
		{
			Serial.println("No border or obstacle");
			return true;
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
		return false;
	}
	return false;
}

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

void Map::retraceStepBack(int *bearingToHead)
{

}

void Map::calculateShortestPath()
{
	
}


// void Map::addObstaclesToMap(int xPos, int yPos)
// {
// 	// Place an obstacle into the map at the given coordinate.
// 	// Add a 5cm (1 grid-space) tolerance around all sides

// 	if (xPos < 0 || xPos > X_MAX)
// 	{
// 		//throw ErrorFlag("x coordinate outside map area", true);
// 		return;
// 	}
// 	else if (yPos < 0 || yPos > Y_MAX)
// 	{
// 		//throw ErrorFlag("y coordinate outside map area", true);
// 		return;
// 	}



// 	if (this->occupancyGrid[xPos][yPos] == 0)
// 	{
// 		this->occupancyGrid[xPos][yPos] = 1;
// 		// this->occupancyGrid[xPos][yPos+1] = 1;
// 		// this->occupancyGrid[xPos+1][yPos+1] = 1;
// 		// this->occupancyGrid[xPos+1][yPos] = 1;
// 		// this->occupancyGrid[xPos+1][yPos-1] = 1;
// 		// this->occupancyGrid[xPos][yPos-1] = 1;
// 		// this->occupancyGrid[xPos-1][yPos-1] = 1;
// 		// this->occupancyGrid[xPos-1][yPos] = 1;
// 		// this->occupancyGrid[xPos-1][yPos+1] = 1;
// 	}
// 	else
// 	{
// 		//throw ErrorFlag("Object already in map at this location", false);
// 	}
// }