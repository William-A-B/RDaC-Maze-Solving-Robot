#include "map.h"



Map::Map()
{
	robotCurrentPosition.xGridSquare = 0;
	robotCurrentPosition.yGridSquare = 0;

	robotHistoryCount = 0;

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
			this->occupancyGrid[i][j] = 0;
		}
	}

	// Set all values in the outer perimeter to 1 to represent the edge of the maze
	for (int i = 0; i < MAP_WIDTH_X; i++)
	{
		this->occupancyGrid[i][0] = 1;
		this->occupancyGrid[i][MAP_HEIGHT_Y-1] = 1;
	}

	for (int i = 0; i < MAP_HEIGHT_Y; i++)
	{
		this->occupancyGrid[0][i] = 1;
		this->occupancyGrid[MAP_WIDTH_X-1][i] = 1;
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

	// Add other bearing conditions
	// Make sure object position adding is not outside the array
	if (robotBearing == 0)
	{
		if (frontSensorDistance >= 0)
			occupancyGrid[rPosX][rPosY + objRelPos_f] = 1;

		if (backSensorDistance >= 0)
			occupancyGrid[rPosX][rPosY - objRelPos_b] = 1;
		
		if (leftSensorDistance >= 0)
			occupancyGrid[rPosX - objRelPos_l][rPosY] = 1;

		if (rightSensorDistance >= 0)
			occupancyGrid[rPosX + objRelPos_r][rPosY] = 1;
	}
	else if (robotBearing == 90)
	{
		if (frontSensorDistance >= 0)
			occupancyGrid[rPosX + objRelPos_f][rPosY] = 1;

		if (backSensorDistance >= 0)
			occupancyGrid[rPosX - objRelPos_b][rPosY] = 1;
		
		if (leftSensorDistance >= 0)
			occupancyGrid[rPosX][rPosY + objRelPos_l] = 1;

		if (rightSensorDistance >= 0)
			occupancyGrid[rPosX][rPosY - objRelPos_r] = 1;
	}
	else if (robotBearing == 180)
	{
		if (frontSensorDistance >= 0)
			occupancyGrid[rPosX][rPosY - objRelPos_f] = 1;

		if (backSensorDistance >= 0)
			occupancyGrid[rPosX][rPosY + objRelPos_b] = 1;
		
		if (leftSensorDistance >= 0)
			occupancyGrid[rPosX + objRelPos_l][rPosY] = 1;

		if (rightSensorDistance >= 0)
			occupancyGrid[rPosX - objRelPos_r][rPosY] = 1;
	}
	else if (robotBearing == 270)
	{
		if (frontSensorDistance >= 0)
			occupancyGrid[rPosX - objRelPos_f][rPosY] = 1;

		if (backSensorDistance >= 0)
			occupancyGrid[rPosX + objRelPos_b][rPosY] = 1;
		
		if (leftSensorDistance >= 0)
			occupancyGrid[rPosX][rPosY - objRelPos_l] = 1;

		if (rightSensorDistance >= 0)
			occupancyGrid[rPosX][rPosY + objRelPos_r] = 1;
	}


	// Place an obstacle into the map at the given coordinate.
	// Add a 5cm (1 grid-space) tolerance around all sides

	// if (xPos < 0 || xPos > MAP_WIDTH_X)
	// {
	// 	//throw ErrorFlag("x coordinate outside map area", true);
	// 	return;
	// }
	// else if (yPos < 0 || yPos > MAP_HEIGHT_Y)
	// {
	// 	//throw ErrorFlag("y coordinate outside map area", true);
	// 	return;
	// }



	// if (this->occupancyGrid[xPos][yPos] == 0)
	// {
	// 	this->occupancyGrid[xPos][yPos] = 1;
	// }
	// else
	// {
	// 	//throw ErrorFlag("Object already in map at this location", false);
	// }
}

void Map::updateRobotPosition(float robotXCoord, float robotYCoord)
{
	robotCurrentPosition.xGridSquare = robotXCoord / 5;
	robotCurrentPosition.yGridSquare = robotYCoord / 5;

	robotPositionHistory[robotHistoryCount].xGridSquare = robotCurrentPosition.xGridSquare;
	robotPositionHistory[robotHistoryCount].yGridSquare = robotCurrentPosition.yGridSquare;

	occupancyGrid[robotCurrentPosition.xGridSquare+1][robotCurrentPosition.yGridSquare+1] = 2;

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


RobotPosition Map::getPositionInMap()
{
    return this->robotCurrentPosition;
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