#include "map.h"



Map::Map()
{
	robotCurrentPosition.xGridSquare = 0;
	robotCurrentPosition.yGridSquare = 0;

	robotHistoryCount = 0;

	// Setup the occupancy grid array to initialise the maze prior to the robot moving
	setupOccupancyGrid();

	// Add finishing position
	// mazeFinish.xGridSquare = 15;
	// mazeFinish.yGridSquare = 39;
	mazeFinish.xGridSquare = 13;
	mazeFinish.yGridSquare = 20;
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
}


void Map::addObstaclesToMap(float frontSensorDistance, float backSensorDistance, float leftSensorDistance, float rightSensorDistance, int robotBearing)
{
	int objRelPos_f = frontSensorDistance / 5;
	int objRelPos_b = backSensorDistance / 5;
	int objRelPos_l = leftSensorDistance / 5;
	int objRelPos_r = rightSensorDistance / 5;

	// Add 1 to adjust for occupancy grid walls
	const int rPosX = robotCurrentPosition.xGridSquare + 1;
	const int rPosY = robotCurrentPosition.yGridSquare + 1;

	int border = 2;

	// Add other bearing conditions
	// Make sure object position adding is not outside the array
	if (robotBearing == 0)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_f, border);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_b, border);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_l, rPosY, border);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_r, rPosY, border);
	}
	else if (robotBearing == 90)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_f, rPosY, border);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_b, rPosY, border);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_l, border);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_r, border);
	}
	else if (robotBearing == 180)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_f, border);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_b, border);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_l, rPosY, border);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_r, rPosY, border);
	}
	else if (robotBearing == 270)
	{
		if (frontSensorDistance >= 0)
			addObstacleBorder(rPosX - objRelPos_f, rPosY, border);

		if (backSensorDistance >= 0)
			addObstacleBorder(rPosX + objRelPos_b, rPosY, border);
		
		if (leftSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY - objRelPos_l, border);

		if (rightSensorDistance >= 0)
			addObstacleBorder(rPosX, rPosY + objRelPos_r, border);
	}
}

void Map::addObstacleBorder(int xCoord, int yCoord, int extent)
{
	for (int x = xCoord-extent; x <= xCoord+extent; x++)
	{
		if (x >= 0 || x < MAP_WIDTH_X)
		{
			for (int y = yCoord-extent; y <= yCoord+extent; y++)
			{
				if (y >= 0 || y < MAP_HEIGHT_Y)
				{
					if (x == xCoord && y == yCoord)
					{
						occupancyGrid[x][y] = OBSTACLE;
					}
					else
					{
						if (occupancyGrid[x][y] != OBSTACLE || occupancyGrid[x][y] != ROBOT)
							occupancyGrid[x][y] = BORDER;
					}
				}
			}
		}
	}
}

void Map::updateRobotPosition(float robotXCoord, float robotYCoord)
{
	robotCurrentPosition.xGridSquare = robotXCoord / 5;
	robotCurrentPosition.yGridSquare = robotYCoord / 5;

	robotPositionHistory[robotHistoryCount].xGridSquare = robotCurrentPosition.xGridSquare;
	robotPositionHistory[robotHistoryCount].yGridSquare = robotCurrentPosition.yGridSquare;

	occupancyGrid[robotCurrentPosition.xGridSquare+1][robotCurrentPosition.yGridSquare+1] = ROBOT;

	robotHistoryCount++;

	if (robotHistoryCount >=100)
	{
		robotHistoryCount = 0;
	}
}

/**
 * @brief Prints the occupancy map to the serial port
 * and displays the occupancy map in a readable format
 *
 */
void Map::displayMap()
{
	Serial.println("\n\n\nOCCUPANCY GRID MAP\n");
	for (int y = MAP_HEIGHT_Y; y >= 0; y--)
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
	xPos = (float)(mazeFinish.xGridSquare * 5);
	yPos = (float)(mazeFinish.yGridSquare * 5);
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

bool Map::checkNextGridSpace(int direction)
{
	if (direction >= 4)
	{
		direction = direction - 4;
	}

	if (direction == 0)
	{
		int occupancy = occupancyGrid[robotCurrentPosition.xGridSquare][robotCurrentPosition.yGridSquare+1];
		if (occupancy == FREE || occupancy == ROBOT)
			return true;
		if (occupancy == BORDER)
			Serial.println("Border North");
		if (occupancy == OBSTACLE)
			Serial.println("Obstacle North");
	}
	else if (direction == 1)
	{
		int occupancy = occupancyGrid[robotCurrentPosition.xGridSquare+1][robotCurrentPosition.yGridSquare];
		if (occupancy == FREE || occupancy == ROBOT)
			return true;
		if (occupancy == BORDER)
			Serial.println("Border East");
		if (occupancy == OBSTACLE)
			Serial.println("Obstacle East");
	}
	else if (direction == 2)
	{
		int occupancy = occupancyGrid[robotCurrentPosition.xGridSquare][robotCurrentPosition.yGridSquare-1];
		if (occupancy == FREE || occupancy == ROBOT)
			return true;
		if (occupancy == BORDER)
			Serial.println("Border South");
		if (occupancy == OBSTACLE)
			Serial.println("Obstacle South");
	}
	else if (direction == 3)
	{
		int occupancy = occupancyGrid[robotCurrentPosition.xGridSquare-1][robotCurrentPosition.yGridSquare];
		if (occupancy == FREE || occupancy == ROBOT)
			return true;
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