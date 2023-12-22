#include "map.h"



Map::Map(int x_coordinate, int y_coordinate)
{
    this->current_position.x_coordinate = x_coordinate;
    this->current_position.y_coordinate = y_coordinate;
}

/**
 * @brief The Occupancy grid which is used to map out all objects within the map
 * Set to 5cm per grid/index giving the maze a total size of approximately 220cm by 160cm
 *
 */
void Map::setup_occupancy_grid()
{
	// Set all values in occupancy grid to 0
	for (int i = 0; i < 32; i++)
	{
		for (int j = 0; j < 44; j++)
		{
			this->occupancy_grid[i][j] = 0;
		}
	}

	// Set all values in the outer perimeter to 1 to represent the edge of the maze
	for (int i = 0; i < 32; i++)
	{
		this->occupancy_grid[i][0] = 1;
		this->occupancy_grid[i][43] = 1;
	}

	for (int i = 0; i < 44; i++)
	{
		this->occupancy_grid[0][i] = 1;
		this->occupancy_grid[31][i] = 1;
	}
}


void Map::add_obstacles_to_map(int x_pos, int y_pos)
{
	// Place an obstacle into the map at the given coordinate.
	// Add a 5cm (1 grid-space) tolerance around all sides

	if (x_pos < 0 || x_pos > X_MAX)
	{
		//throw ErrorFlag("x coordinate outside map area", true);
		return;
	}
	else if (y_pos < 0 || y_pos > Y_MAX)
	{
		//throw ErrorFlag("y coordinate outside map area", true);
		return;
	}



	if (this->occupancy_grid[x_pos][y_pos] == 0)
	{
		this->occupancy_grid[x_pos][y_pos] = 1;
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
void Map::display_map()
{
	for (int y = 0; y < Y_MAX; y++)
	{
		for (int x = 0; x < X_MAX; x++)
		{
			Serial.print(this->occupancy_grid[x][y]);
		}
		Serial.print("\n");
	}
}

void Map::set_robot_location(int x_coordinate, int y_coordinate)
{
    this->current_position.x_coordinate = x_coordinate;
    this->current_position.y_coordinate = y_coordinate;
}

bool Map::check_route_ahead_in_map(int bearing_heading, int distance_to_move)
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



	if (bearing_heading == 0)
	{
		
	}

	return false;
}


robot_position_in_map Map::get_position_in_map()
{
    return this->current_position;
}



// void Map::add_obstacles_to_map(int x_pos, int y_pos)
// {
// 	// Place an obstacle into the map at the given coordinate.
// 	// Add a 5cm (1 grid-space) tolerance around all sides

// 	if (x_pos < 0 || x_pos > X_MAX)
// 	{
// 		//throw ErrorFlag("x coordinate outside map area", true);
// 		return;
// 	}
// 	else if (y_pos < 0 || y_pos > Y_MAX)
// 	{
// 		//throw ErrorFlag("y coordinate outside map area", true);
// 		return;
// 	}



// 	if (this->occupancy_grid[x_pos][y_pos] == 0)
// 	{
// 		this->occupancy_grid[x_pos][y_pos] = 1;
// 		// this->occupancy_grid[x_pos][y_pos+1] = 1;
// 		// this->occupancy_grid[x_pos+1][y_pos+1] = 1;
// 		// this->occupancy_grid[x_pos+1][y_pos] = 1;
// 		// this->occupancy_grid[x_pos+1][y_pos-1] = 1;
// 		// this->occupancy_grid[x_pos][y_pos-1] = 1;
// 		// this->occupancy_grid[x_pos-1][y_pos-1] = 1;
// 		// this->occupancy_grid[x_pos-1][y_pos] = 1;
// 		// this->occupancy_grid[x_pos-1][y_pos+1] = 1;
// 	}
// 	else
// 	{
// 		//throw ErrorFlag("Object already in map at this location", false);
// 	}
// }