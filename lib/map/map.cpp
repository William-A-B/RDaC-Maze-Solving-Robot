#include "map.h"



Map::Map(int xCoordinate, int yCoordinate)
{
    this->currentPosition.xCoordinate = xCoordinate;
    this->currentPosition.yCoordinate = yCoordinate;
}

/**
 * @brief The Occupancy grid which is used to map out all objects within the map
 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
 *
 */
void Map::setupOccupancyGrid()
{
	// Set all values in occupancy grid to 0
	for (int i = 0; i < 32; i++)
	{
		for (int j = 0; j < 44; j++)
		{
			this->occupancyGrid[i][j] = 0;
		}
	}

	// Set all values in the outer perimeter to 1 to represent the edge of the maze
	for (int i = 0; i < 32; i++)
	{
		this->occupancyGrid[i][0] = 1;
		this->occupancyGrid[i][43] = 1;
	}

	for (int i = 0; i < 44; i++)
	{
		this->occupancyGrid[0][i] = 1;
		this->occupancyGrid[31][i] = 1;
	}
}


void Map::addObstaclesToMap(int xPos, int yPos)
{
	// Place an obstacle into the map at the given coordinate.
	// Add a 5cm (1 grid-space) tolerance around all sides

	if (xPos < 0 || xPos > X_MAX)
	{
		//throw ErrorFlag("x coordinate outside map area", true);
		return;
	}
	else if (yPos < 0 || yPos > Y_MAX)
	{
		//throw ErrorFlag("y coordinate outside map area", true);
		return;
	}



	if (this->occupancyGrid[xPos][yPos] == 0)
	{
		this->occupancyGrid[xPos][yPos] = 1;
	}
	else
	{
		//throw ErrorFlag("Object already in map at this location", false);
	}
}


/**
 * @brief Prints the occupancy map to the serial port
 * and displays the occupancy map in a readable format
 *
 */
void Map::displayMap()
{
	for (int y = 0; y < Y_MAX; y++)
	{
		for (int x = 0; x < X_MAX; x++)
		{
			Serial.print(this->occupancyGrid[x][y]);
		}
		Serial.print("\n");
	}
}

void Map::setRobotLocation(int xCoordinate, int yCoordinate)
{
    this->currentPosition.xCoordinate = xCoordinate;
    this->currentPosition.yCoordinate = yCoordinate;
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


RobotPositionInMap Map::getPositionInMap()
{
    return this->currentPosition;
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