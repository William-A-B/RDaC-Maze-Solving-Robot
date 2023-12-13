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


void Map::add_obstacles_to_map(int x_coordinate, int y_coordinate)
{

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

bool Map::check_route_ahead(int bearing_heading, int distance_to_move)
{
    // Get current robot position
    // Check direction wanting to move in
    // Check the free space ahead within the space wanting to move in
    // return true or false whether the space ahead is free to move in
}


robot_position Map::get_position_in_map()
{
    return this->current_position;
}