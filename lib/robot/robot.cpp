#include "robot.h"



/**
 * @brief Function used to test various features. Any code implementation is often temporary
 * But may be promoted up to its own function at later use.
 */
void Robot::test()
{
	float frontSensorDistance = mySensors.read_averaged_IR_sensor_front(DEFAULT_AVG_SENSOR_READINGS);
	float backSensorDistance = mySensors.read_averaged_IR_sensor_back(DEFAULT_AVG_SENSOR_READINGS);
	float leftSensorDistance = mySensors.read_averaged_usonic_sensor_left(DEFAULT_AVG_SENSOR_READINGS);
	float rightSensorDistance = mySensors.read_averaged_usonic_sensor_right(DEFAULT_AVG_SENSOR_READINGS);

	Serial.println("Sensor distances to add to map");
	Serial.print("F: ");
	Serial.print(frontSensorDistance);
	Serial.print(", B: ");
	Serial.print(backSensorDistance);
	Serial.print(", L: ");
	Serial.print(leftSensorDistance);
	Serial.print(", R: ");
	Serial.println(rightSensorDistance);

	wait_us(200000);

}

/**
 * @brief Run once upon powering on the robot.
 * Sets up sections of robot which only ever need to be set once
 */
void Robot::initialSetup()
{
	// Set default forwards driving distance to 5cm
	distanceToMoveForwards = 5.0f;

	verbose = false;


    // Calibrate the motors for use
	myMotors.setup();

	// Set the default direction for the robot to move in
	myMotors.setDirection(myMotors.DIR_FORWARDS);

	// Attach the interrupts for the encoders on the motors
	myMotors.attachEncoderInterrupts();

	myMap.initialSetup(0, 0);

	if (robotBLE.initialise_ble() == true)
	{
		Serial.println("Bluetooth initialised successfully");
		robotBLE.pollBLE();
	}
	else
	{
		Serial.println("Bluetooth failed and could not initialise");
	}



	// Initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(LEDR, OUTPUT);
	pinMode(LEDG, OUTPUT);
	pinMode(LEDB, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	setRGBLED(1, 1, 1);
}

/**
 * @brief Run once the robot is placed at the starting position in the maze
 * Is a state within the state machine, can be called again to restart the robots algorithms
 */
void Robot::setup()
{
	if (verbose)
		Serial.println("\nsetup\n");

	// while (robotBLE.isClientConnected() == false)
	// {
	// 	Serial.println("Not Connected, waiting to connect");
	// 	wait_us(1000);
	// }

	// Serial.println("Connected");


	this->calculateStartingLocation();
    this->centreOnMapGrid();


	// if (this->checkSideSpaceLeft(1))
	// {
	// 	digitalWrite(LEDR, LOW);
	// 	digitalWrite(LEDG, LOW);
	// 	digitalWrite(LEDB, LOW);

	// 	this->rotateRobot(-90);
	// 	while (this->checkRouteAhead(-1))
	// 	{
	// 		this->driveForwards();
	// 	}
	// 	this->rotateRobot(90);
	// }

	setRGBLED(1, 1, 1);

	this->currentState = this->STATE_SOLVE;
}

/**
 * @brief Robot starts solving the maze and finding its way to the end
 * Runs all algorithms for solving and mapping from this function
 * Part of state SOLVE within the state machine
 */
void Robot::solveMaze()
{
	algorithm = NAVIGATE_MAP;
	// algorithm = KNOWN_MAZE;
	// algorithm = FOLLOW_WALL;

	if (algorithm == DIRECT_AND_AROUND)
	{
		runDirectAlgorithm();
	}
	else if (algorithm == NAVIGATE_MAP)
	{
		// runShortestNavigationAlgorithm();
		currentState = RobotState::STATE_DETERMINE_DIRECTION;
	}
	else if (algorithm == KNOWN_MAZE)
	{
		currentState = RobotState::STATE_SOLVE_KNOWN_MAZE;
	}
	else if (algorithm == FOLLOW_WALL)
	{
		runWallFollowingAlgorithm();
	}
}

void Robot::solveKnownMaze()
{
	// Distances too big and causing the "not enough free space ahead", so it needs to be removed for this algorithm
	Serial.println("Start");
	Serial.println("Right");
	rotateRobot(90, true);
	Serial.println("Forwards");
	moveRobot(45.0f);
	Serial.println("Left");
	rotateRobot(-90, true);
	Serial.println("Forwards");
	moveRobot(45.0f);
	Serial.println("Left");
	rotateRobot(-90, true);
	Serial.println("Forwards");
	moveRobot(48.0f);
	Serial.println("Right");
	rotateRobot(90, true);
	Serial.println("Forwards");
	moveRobot(59.0f);
	Serial.println("Right");
	rotateRobot(90, true);
	Serial.println("Forwards");
	moveRobot(58.0f);
	Serial.println("Left");
	rotateRobot(-90, true);
	Serial.println("Forwards");
	moveRobot(55.0f);
	Serial.println("End");
}

void Robot::runShortestNavigationAlgorithm()
{
	
	if (verbose)
		Serial.println("\nrunShortestNavigationAlgorithm\n");

	bool canMoveForwards = false;
	//bool reverse = false;

	

	// Check if the robot has free space in front of it to move forwards by 5cm
	canMoveForwards = this->checkRouteAhead(distanceToMoveForwards);

	if (canMoveForwards)
	{

		setRGBLED(1, 0, 1);
		moveHelper();
	}
	else
	{
		myMap.displayRobotHistory();
		myMap.displayMap();

		if (this->checkSideSpaceLeft(DEFAULT_AVG_SENSOR_READINGS))
		{
			setRGBLED(1, 1, 0);
			this->rotateRobot(-90);
		}
		else if (this->checkSideSpaceRight(DEFAULT_AVG_SENSOR_READINGS))
		{
			setRGBLED(1, 1, 0);
			this->rotateRobot(90);
		}
		// else
		// {
		// 	reverse = true;
		// 	while (reverse)
		// 	{
		// 		setRGBLED(0, 1, 1);
		// 		this->driveBackwards();
		// 		wait_us(10000);

		// 		if (this->checkSideSpaceLeft(1))
		// 		{
		// 			reverse = false;
		// 			this->rotateRobot(-90);
		// 		}
		// 		else if (this->checkSideSpaceRight(1))
		// 		{
		// 			reverse = false;
		// 			this->rotateRobot(90);
		// 		}
		// 	}
		// }
	}
}


/**
 * @brief Runs the algorithm for the robot to solve the maze via moving
 * directly forwards until it reaches an obstacle then it turns into the
 * next free direction and repeats
 */
void Robot::runDirectAlgorithm()
{
	if (verbose)
		Serial.println("\nrunDirectAlgorithm\n");

	bool canMoveForwards = false;
	//bool reverse = false;

	canMoveForwards = this->checkRouteAhead(distanceToMoveForwards);

	if (canMoveForwards)
	{
		setRGBLED(1, 0, 1);
		moveHelper();
		// if (driveForwardsStarted == false)
		// {
		// 	if (robotBLE.connectToClient() == true)
		// 	{
		// 		Serial.println("Connected to Client, starting going forwards");
		// 	}
		// 	this->driveForwards();

		// 	initialDistanceMovedLeft = myMotors.getDistanceTravelledLeft();
		// 	initialDistanceMovedRight = myMotors.getDistanceTravelledRight();
		// 	driveForwardsStarted = true;
		// }
		
		if (robotBLE.isClientConnected() == true)
		{
			robotBLE.pingService(0x01);
			Serial.println("Connected - Robot going forwards");
		}
	}
	else
	{
		myMap.displayRobotHistory();
		myMap.displayMap();

		// this->stopMoving();
		// driveForwardsStarted = false;

		// robotBLE.enableSendingData();
		// updateCoordinateLocation();
		// robotBLE.disableSendingData();

		if (robotBLE.disconnectFromClient() == true)
		{
			Serial.println("Disconnected from Client, stopping moving and now turning");
		}

		if (this->checkSideSpaceLeft(DEFAULT_AVG_SENSOR_READINGS))
		{
			setRGBLED(1, 1, 0);
			this->rotateRobot(-90);
		}
		else if (this->checkSideSpaceRight(DEFAULT_AVG_SENSOR_READINGS))
		{
			setRGBLED(1, 1, 0);
			this->rotateRobot(90);
		}
		// else
		// {
		// 	reverse = true;
		// 	while (reverse)
		// 	{
		// 		setRGBLED(0, 1, 1);
		// 		this->driveBackwards();
		// 		wait_us(10000);
		// 		// if (mySensors.get_back_IR_distance() < MIN_IR_DIST_REAR)
		// 		// {
		// 		// 	reverse = false;
		// 		// 	this->stopMoving();
		// 		// 	digitalWrite(LEDR, LOW);
		// 		// 	digitalWrite(LEDG, LOW);
		// 		// 	digitalWrite(LEDB, LOW);
		// 		// }

		// 		if (this->checkSideSpaceLeft(1))
		// 		{
		// 			reverse = false;
		// 			this->rotateRobot(-90);
		// 		}
		// 		else if (this->checkSideSpaceRight(1))
		// 		{
		// 			reverse = false;
		// 			this->rotateRobot(90);
		// 		}
				
		// 	}
		// }
	}
}

/**
 * @brief Runs the algorithm for the robot to solve the maze via following
 * the left hand wall until it reaches the end.
 */
void Robot::runWallFollowingAlgorithm()
{
	Serial.println("\nrunWallFollowingAlgorithm\n");

	bool reachedNextWall = false;

	while (this->checkSideSpaceLeft(1) == false)
	{	
		reachedNextWall = false;
		//this->driveForwards();
		//correctOrientationWall();
		if (this->checkRouteAhead(-1))
		{
			setRGBLED(1, 0, 1);
			Serial.println("Free space ahead");
			Serial.println("Driving forwards\n\n");
			this->driveForwards();
		}
		else
		{
			if (this->checkSideSpaceLeft(1) == false)
			{
				setRGBLED(0, 1, 1);
				Serial.println("No space ahead and wall on left");
				Serial.println("Turning Right\n\n");
				this->rotateRobot(90, true);
				this->driveForwards();
			}
		}
	}

	if (this->checkSideSpaceLeft(1) == true)
	{
		setRGBLED(1, 1, 0);
		Serial.println("Reached end of left wall");
		Serial.println("Turning left to continue along next wall\n\n");
		
		this->moveRobot(13.0f);
		this->rotateRobot(-90, true);
		moveRobot(16.0f);

		// while (!reachedNextWall)
		// {
		// 	this->driveForwards();
		// 	if (!this->checkRouteAhead(-1))
		// 	{
		// 		this->stopMoving();
		// 		break;
		// 	}
		// 	if (!this->checkSideSpaceLeft(1))
		// 	{
		// 		reachedNextWall = true;
		// 		this->stopMoving();
		// 		wait_us(500000);
		// 	}
		// }

		// this->moveRobot(80.0f);
		// this->rotateRobot(-90);
		// this->moveRobot(80.0f);
	}
}

void Robot::correctOrientationWall()
{
	float leftSensorDistance = mySensors.read_averaged_usonic_sensor_left(DEFAULT_AVG_SENSOR_READINGS);
	if (leftSensorDistance > MIN_USONIC_DIST + 7.0f)
		rotateRobot(-2, true);
	else if (leftSensorDistance < MIN_USONIC_DIST)
		rotateRobot(2, true);
}

/**
 * @brief Starts the robot driving forwards at its default speed of 0.5
 */
void Robot::driveForwards()
{
	myMotors.setDirection(myMotors.DIR_FORWARDS);
	myMotors.setSpeed(DEFAULT_ROBOT_SPEED);
	myMotors.drive(0);
}

void Robot::leftAndForwards()
{
	rotateRobot(-90);
	currentState = RobotState::STATE_FORWARD;
}

void Robot::rightAndForwards()
{
	rotateRobot(90);
	currentState = RobotState::STATE_FORWARD;
}

void Robot::moveHelper()
{
	if (verbose)
		Serial.println("\nmoveHelper\n");

	initialDistanceMovedLeft = myMotors.getDistanceTravelledLeft();
	initialDistanceMovedRight = myMotors.getDistanceTravelledRight();

	moveRobot(distanceToMoveForwards);


	//correctOrientation();

	// Updates the coordinate positions of the robot based 
	// on the initial distance before moving
	updateCoordinateLocation();

	// Reads sensors to update map with any obstacles
	processSensorInfo();

	myMap.displayMap();
	// myMap.displayRobotHistory();
}


void Robot::moveForwards()
{
	moveHelper();
	currentState = RobotState::STATE_DETERMINE_DIRECTION;
}

/**
 * @brief Moves the robot a given distance, if the value given is negative the robot will move backwards
 * 
 * @param distanceToMove 	The number of mm to move the robot
 */
void Robot::moveRobot(float distanceToMove)
{
	if (verbose)
		Serial.println("\nmoveRobot\n");

    float initialDistanceMovedLeft = 0.0f;
	float initialDistanceMovedRight = 0.0f;
	float distanceMovedLeft = 0.0f;
	float distanceMovedRight = 0.0f;
	float differenceBetweenMotorDistances = 0.0f;

	bool routeAheadFree = false;
	bool reachedDistance = false;

	int loopCount = 0;


	// Stop robot moving (just in case)
	myMotors.stopDriving();
	
	// WABWAB
	// Check that space ahead the robot is wanting to move is free
	routeAheadFree = this->checkRouteAhead(distanceToMove);

	// If route ahead is not clear, return and don't move forwards.
	if (routeAheadFree == false)
	{
		//throw ErrorFlag("Could not move forwards as space was not free", true);
		Serial.println("Not enough free space ahead");
		return;
	}

    // Call direction change function on motors depending on the angle direction
	// positive = forwards
	// negative = backwards
	if (distanceToMove > 0)
	{
		myMotors.setDirection(myMotors.DIR_FORWARDS);
	}
	else if (distanceToMove < 0)
	{
		// Since the distance is negative, make it positive for using in the calculation later
		distanceToMove = distanceToMove * -1.0f;
		myMotors.setDirection(myMotors.DIR_BACKWARDS);
	}
	else
	{
		return;
	}

    // Calculate current distance the robot has moved before the robot starts moving the set amount
	initialDistanceMovedLeft = myMotors.getDistanceTravelledLeft();
	initialDistanceMovedRight = myMotors.getDistanceTravelledRight();

    // Start robot moving
	myMotors.setSpeed(DEFAULT_ROBOT_SPEED);
	myMotors.drive(0);


	// Loop until robot has moved the distance specified
	// Loops until the sum of the distances moved by both motors is equal to or greater than
	// the set distance to move multiplied by two since the sum of the two wheels is taken.
    do 
	{
		// Get left wheel distance moved whilst turning robot
		distanceMovedLeft = myMotors.getDistanceTravelledLeft() - initialDistanceMovedLeft;

		// Get right wheel distance moved whilst turning robot
		distanceMovedRight = myMotors.getDistanceTravelledRight() - initialDistanceMovedRight;

		// Get sum/difference between both motors
		differenceBetweenMotorDistances = distanceMovedLeft + distanceMovedRight;

		// Delay to ensure polling of interrupt is not backlogged
		wait_us(100);

		if (fabs(differenceBetweenMotorDistances) >= (distanceToMove * 2.0f))
		{
			reachedDistance = true;
		}
		loopCount++;
		

	} while (!reachedDistance);

	if (verbose)
		Serial.println(loopCount);

	// // Loop until robot has moved the distance specified
	// // Loops until the sum of the distances moved by both motors is equal to or greater than
	// // the set distance to move multiplied by two since the sum of the two wheels is taken.
    // while (fabs(differenceBetweenMotorDistances) < (distanceToMove * 2.0f))
	// {
	// 	// Get left wheel distance moved whilst turning robot
	// 	distanceMovedLeft = myMotors.getDistanceTravelledLeft() - initialDistanceMovedLeft;

	// 	// Get right wheel distance moved whilst turning robot
	// 	distanceMovedRight = myMotors.getDistanceTravelledRight() - initialDistanceMovedRight;

	// 	// Get sum/difference between both motors
	// 	differenceBetweenMotorDistances = distanceMovedLeft + distanceMovedRight;

	// 	// Delay to ensure polling of interrupt is not backlogged
	// 	wait_us(100);
	// }

    // Stop robot moving and reset direction to forwards (incase it was going backwards)
	myMotors.stopDriving();
    myMotors.setDirection(myMotors.DIR_FORWARDS);
}

/**
 * @brief Rotates the robot about a point a set number of degrees
 *
 * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
 */
void Robot::rotateRobot(int degrees)
{
	rotateRobot(degrees, false);
	currentState = RobotState::STATE_DETERMINE_DIRECTION;
}

/**
 * @brief Rotates the robot about a point a set number of degrees
 *
 * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
 */
void Robot::rotateRobot(int degrees, bool ignoreBearingUpdate)
{
	if (ignoreBearingUpdate == false)
		Serial.println("\nrotateRobot");

	float initialDistanceMovedLeft = 0.0f;
	float initialDistanceMovedRight = 0.0f;
	float distanceMovedWhileTurningLeft = 0.0f;
	float distanceMovedWhileTurningRight = 0.0f;
	float differenceBetweenMotorDistances = 0.0f;

	// Stop robot moving
	myMotors.stopDriving();

	if (ignoreBearingUpdate == false)
	{
		this->updateBearing(degrees);
	}
    
	// Call direction change function on motors depending on the angle direction
	if (degrees > 0)
	{
		myMotors.setDirection(myMotors.DIR_CLOCKWISE);
	}
	else if (degrees < 0)
	{
		degrees = degrees * -1;
		myMotors.setDirection(myMotors.DIR_ANTICLOCKWISE);
	}
	else
	{
		return;
	}

	// Calculate current distance the robot has moved before the robot starts turning the set amount
	initialDistanceMovedLeft = myMotors.getDistanceTravelledLeft();
	initialDistanceMovedRight = myMotors.getDistanceTravelledRight();

	// Calculate equivalent arc of circle
	// Left wheel 80mm out from centre
	// Right wheel 80mm out
	// arc distance = (degrees/360) * 2*pi*radius
	// Distance in mm
	float arc_distance_to_turn = ((float)degrees / 360.0f) * 2.0f * 3.141f * ROBOT_WHEEL_RADIUS;

	// Start robot moving
	myMotors.setSpeed(DEFAULT_ROBOT_SPEED);
	myMotors.drive(0);

	// While distance moved < arc length
	// loop
	// Get distance moved by wheels
	// (distance moved could be an average of the two wheels)
	// delay between distance moved polls
	while (fabs(differenceBetweenMotorDistances) < (arc_distance_to_turn * 2.0f))
	{
		// Get left wheel distance moved whilst turning robot
		distanceMovedWhileTurningLeft = myMotors.getDistanceTravelledLeft() - initialDistanceMovedLeft;

		// Get right wheel distance moved whilst turning robot
		distanceMovedWhileTurningRight = myMotors.getDistanceTravelledRight() - initialDistanceMovedRight;

		// Get difference between both motors
		differenceBetweenMotorDistances = distanceMovedWhileTurningLeft - distanceMovedWhileTurningRight;

		// Delay to ensure polling of interrupt is not backlogged
		wait_us(100);
	}

	// Stop robot moving and reset direction to forwards
	myMotors.stopDriving();
	myMotors.setDirection(myMotors.DIR_FORWARDS);
}

/**
 * @brief Calculates the starting position of the robot
 * Checks surrounding areas based on sensors
 * And sets the coordinate values based on the sensors
 */
void Robot::calculateStartingLocation()
{
	Serial.println("\ncalculateStartingLocation");
	
	// float frontSensorDistance = mySensors.read_averaged_IR_sensor_front(5);
	float backSensorDistance = mySensors.read_averaged_IR_sensor_back(DEFAULT_AVG_SENSOR_READINGS);
	float leftSensorDistance = mySensors.read_averaged_usonic_sensor_left(DEFAULT_AVG_SENSOR_READINGS);
	// float rightSensorDistance = mySensors.read_averaged_usonic_sensor_right(5);

    // Calculate direction robot is facing
    // Assume the robot is facing towards finish from centre of start location

    // Set coordinate position based on sensor readings
    this->currentPosition.xCoordinate = leftSensorDistance;
    this->currentPosition.yCoordinate = backSensorDistance;

	myMap.updateRobotPosition(currentPosition.xCoordinate, currentPosition.yCoordinate);

	Serial.println("\n(X Coordinate, Y Coordinate)");
	Serial.print("( ");
    Serial.print(this->currentPosition.xCoordinate);
	Serial.print(", ");
    Serial.print(this->currentPosition.yCoordinate);
	Serial.print(" )\n");
}

/**
 * @brief Centres the robot in the middle of the occupancy grid square
 */
void Robot::centreOnMapGrid()
{
	if (verbose)
		Serial.println("\ncentreOnMapGrid");


	float coordinateGridRemainderX = fmod(this->currentPosition.xCoordinate, 5);
	float coordinateGridRemainderY = fmod(this->currentPosition.yCoordinate, 5);

    if (coordinateGridRemainderX > 2.5f)
    {
        if (this->bearing == 0)
        {
            this->rotateRobot(90);
            this->moveRobot(-1.0f * (coordinateGridRemainderX - 2.5f));
        }
        else if (this->bearing == 90)
        {
            this->moveRobot(-1.0f * (coordinateGridRemainderX - 2.5f));
        }
    }
    else if (coordinateGridRemainderX < 2.5f)
    {
        if (this->bearing == 0)
        {
            this->rotateRobot(90);
            this->moveRobot(coordinateGridRemainderX);
        }
        else if (this->bearing == 90)
        {
            this->moveRobot(coordinateGridRemainderX);
        }
    }
	else
	{
		// In middle of horizontal coordinate, no need to move
	}

    if (coordinateGridRemainderY > 2.5f)
    {
        if (this->bearing == 0)
        {
            this->moveRobot(-1.0f * (coordinateGridRemainderY - 2.5f));
        }
        else if (this->bearing == 90)
        {
            this->rotateRobot(-90);
            this->moveRobot(-1.0f * (coordinateGridRemainderY - 2.5f));
        }
    }
    else if (coordinateGridRemainderY < 2.5f)
    {
        if (this->bearing == 0)
        {
            this->moveRobot(coordinateGridRemainderY);
        }
        else if (this->bearing == 90)
        {
            this->rotateRobot(-90);
            this->moveRobot(coordinateGridRemainderY);
        }
    }
}

/**
 * @brief Updates the robots bearing to identify the direction in which the robot is currently heading
 * 
 * @param angle_to_add 	The angle in which the robot is currently turning by
 */
void Robot::updateBearing(int angle_to_add)
{
    if ((this->bearing + angle_to_add) < 360 && (this->bearing + angle_to_add) >= 0)
    {
        this->bearing = this->bearing + angle_to_add;
    }
    else if ((this->bearing + angle_to_add) == 360)
    {
        this->bearing = 0;
    }
    else if ((this->bearing + angle_to_add) > 360)
    {
        this->bearing = (this->bearing + angle_to_add) - 360;
    }
    else if ((this->bearing + angle_to_add) < 0)
    {
        this->bearing = this->bearing + angle_to_add + 360;
    }
	else 
	{
		return;
	}
}

/**
 * @brief Checks the space in front of the robot to see if there is free space or not
 * Decides whether the robot is able to move forwards the specified distance
 * 
 * @param distanceToMove 	The distance to check whether the robot can move forwards
 * 							If it is -1 then it checks for any free space ahead.
 * @return true 			True if the robot is allowed to move forwards
 * @return false 			False if there is an object in the way and the robot can't move forwards
 */
bool Robot::checkRouteAhead(float distanceToMove)
{
	float distanceToObjectsFront = mySensors.read_averaged_IR_sensor_front(DEFAULT_AVG_SENSOR_READINGS);

	if (distanceToMove == -1.0f)
	{
		if (distanceToObjectsFront > MIN_IR_DIST_FRONT)
		{
			return true;
		}
	}

	if (distanceToObjectsFront > (distanceToMove + MIN_IR_DIST_FRONT))
	{
		return true;
	}
	else if (distanceToObjectsFront == MAX_IR_DISTANCE)
	{
		Serial.println("Front range sensor max limit reached, or too close to object");
		return true;
	}
	else
	{
		return false;
	}
	return false;
}

/**
 * @brief Checks if there is free space to move into on the left of the robot
 * 
 * @return true 	True if the robot can turn left and move forwards
 * @return false 	False if the robot cannot move left and something is blocking it
 */
bool Robot::checkSideSpaceLeft(int num_readings)
{
	int leftSensorDistance = mySensors.read_averaged_usonic_sensor_left(num_readings);
	
	if (leftSensorDistance > MIN_USONIC_DIST+distanceToMoveForwards)
	{
		return true;
	}
	return false;
}

/**
 * @brief Checks if there is free space to move into on the right of the robot
 * 
 * @return true 	True if the robot can turn right and move forwards
 * @return false 	False if the robot cannot move left and something is blocking it
 */
bool Robot::checkSideSpaceRight(int num_readings)
{
	int rightSensorDistance = mySensors.read_averaged_usonic_sensor_right(num_readings);

	if (rightSensorDistance > MIN_USONIC_DIST+distanceToMoveForwards)
	{
		return true;
	}
	return false;
}


void Robot::updateCoordinateLocation()
{
	Serial.println("\nupdateCoordinateLocation");

	// float distanceMovedLeft = 0.0f;
	float distanceMovedRight = 0.0f;
	// float averageDistanceMoved = 0.0f;

	// distanceMovedLeft = myMotors.getDistanceTravelledLeft() - initialDistanceMovedLeft;
	distanceMovedRight = myMotors.getDistanceTravelledRight() - initialDistanceMovedRight;

	// Average distance between both wheels
	// averageDistanceMoved = (distanceMovedLeft + distanceMovedRight) / 2.0f;

	if (bearing == 0) 
	{
		currentPosition.yCoordinate = currentPosition.yCoordinate + distanceMovedRight;
	}
	else if (bearing == 90)
	{
		currentPosition.xCoordinate = currentPosition.xCoordinate + distanceMovedRight;
	}
	else if (bearing == 180)
	{
		currentPosition.yCoordinate = currentPosition.yCoordinate - distanceMovedRight;
	}
	else if (bearing == 270)
	{
		currentPosition.xCoordinate = currentPosition.xCoordinate - distanceMovedRight;
	}

	Serial.print(bearing);
	Serial.print(", ( ");
	Serial.print(currentPosition.xCoordinate);
	Serial.print(", ");
	Serial.print(currentPosition.yCoordinate);
	Serial.println(" )");

	//robotBLE.updateRobotLocationInfo(bearing, currentPosition.xCoordinate, currentPosition.yCoordinate);

	myMap.updateRobotPosition(currentPosition.xCoordinate, currentPosition.yCoordinate);

}

void Robot::processSensorInfo()
{
	if (verbose)
		Serial.println("\nprocessSensorInfo\n");

	float frontSensorDistance = mySensors.read_averaged_IR_sensor_front(DEFAULT_AVG_SENSOR_READINGS);
	float backSensorDistance = mySensors.read_averaged_IR_sensor_back(DEFAULT_AVG_SENSOR_READINGS);
	float leftSensorDistance = mySensors.read_averaged_usonic_sensor_left(DEFAULT_AVG_SENSOR_READINGS);
	float rightSensorDistance = mySensors.read_averaged_usonic_sensor_right(DEFAULT_AVG_SENSOR_READINGS);

	Serial.println("Sensor distances to add to map");
	Serial.print("F: ");
	Serial.print(frontSensorDistance);
	Serial.print(", B: ");
	Serial.print(backSensorDistance);
	Serial.print(", L: ");
	Serial.print(leftSensorDistance);
	Serial.print(", R: ");
	Serial.println(rightSensorDistance);

	if (frontSensorDistance >= MAX_IR_DISTANCE)
		frontSensorDistance = -1.0f;

	if (backSensorDistance >= MAX_IR_DISTANCE)
		backSensorDistance = -1.0f;
	
	if (leftSensorDistance >= MAX_USONIC_DISTANCE)
		leftSensorDistance = -1.0f;

	if (rightSensorDistance >= MAX_USONIC_DISTANCE)
		rightSensorDistance = -1.0f;
	
	

	myMap.addObstaclesToMap(frontSensorDistance, backSensorDistance, leftSensorDistance, rightSensorDistance, bearing);
}

void Robot::correctOrientation()
{	
	if (verbose)
		Serial.println("\ncorrectOrientation");

	float turningAngle = 10.0f;
	float originalSensorDistance = mySensors.read_averaged_IR_sensor_front(DEFAULT_AVG_SENSOR_READINGS);

	rotateRobot((int)(-1.0f*turningAngle), true);

	float adjLeftSensorDistance = mySensors.read_averaged_IR_sensor_front(DEFAULT_AVG_SENSOR_READINGS);

	rotateRobot((int)(2.0f*turningAngle), true);

	float adjRightSensorDistance = mySensors.read_averaged_IR_sensor_front(DEFAULT_AVG_SENSOR_READINGS);

	if (verbose)
	{
		Serial.println("\n\ncorrectOrientation Sensor Readings");
		Serial.println("Left, Original, Right");
		Serial.println(adjLeftSensorDistance);
		Serial.println(originalSensorDistance);
		Serial.println(adjRightSensorDistance);
	}
	

	/**
	 * If two or more of the lengths are equal.
	 * The robot orientation cannot be corrected,
	 * so just return from this function
	 */

	if (adjLeftSensorDistance > originalSensorDistance)
	{
		if (adjRightSensorDistance < originalSensorDistance)
		{
			Serial.println("\ncorrectOrientation 1");
			// Robot is steering left of centre line
			correctOrientationHelper(originalSensorDistance, adjRightSensorDistance, turningAngle, true);
		}
		else
		{
			Serial.println("\ncorrectOrientation 2");
			// Two intercepts are either side of turning line
			// Therefore no turning to orientate required
		}
	}
	else
	{
		if (adjRightSensorDistance > originalSensorDistance)
		{
			Serial.println("\ncorrectOrientation 3");
			// Robot is steering right of centre line
			correctOrientationHelper(originalSensorDistance, adjLeftSensorDistance, turningAngle, false);
		}
		else
		{
			Serial.println("\ncorrectOrientation 4");
			// Two intercepts are either side of turning line
			// Therefore no turning to orientate required
		}
	}

}

void Robot::correctOrientationHelper(float a, float b, float aC, bool leftOfCentre)
{
	//if (verbose)
	//	Serial.println("\ncorrectOrientationHelper");

	if(leftOfCentre == true)
	{
		Serial.print("orig: ");
		Serial.print(a);
		Serial.print(", left:");
		Serial.print(b);
		Serial.println("\n");
	}
	else 
	{
		Serial.print("orig: ");
		Serial.print(a);
		Serial.print(", right:");
		Serial.print(b);
		Serial.println(" ");
	}
	

	float c = sqrtf((a*a) + (b*b) - (2*a*b*cosf(aC*PI/180.0f)));

	float aA = asinf((a*sinf(aC*PI/180.0f))/c);

	Serial.print("c: ");
	Serial.print(c);
	Serial.println(", Angle aA: ");
	Serial.print(aA*180.0f/PI);
	
	float rotAngle = 90.0f - (aA*180.0f/PI);

	// If robot going towards right, then apply a negative rotation
	if (leftOfCentre == false)
	{
		rotAngle = rotAngle + (2.0f * aC);
		rotAngle = rotAngle * -1.0f;
	}
	rotAngle = rotAngle / 2.0f;
		
	rotateRobot((int)rotAngle, true);

	Serial.print("\n\nAdjustment Angle ");
	Serial.println(rotAngle);
	
}

void Robot::setRGBLED(int red, int green, int blue)
{
	digitalWrite(LEDR, red);
	digitalWrite(LEDG, green);
	digitalWrite(LEDB, blue);
}

void Robot::determineDirection()
{
	Serial.println("\nDetermining Direction: Next Step Forwards\n");

	float forwardsDistance = 99999.9f;
	float leftDistance = 99999.9f;
	float rightDistance = 99999.9f;

	// Determine forwards direction
	// Is space in front of sensor

	int sensorAdjustment = adjustedMapDirection();

	if (sensorAdjustment == -1)
	{
		// Bearing is not 0, 90, 180 or 270
		Serial.println("Bearing Not Correct");
		currentState = RobotState::STATE_STOP;
	}

	if (checkRouteAhead(distanceToMoveForwards) == true)
	{
		if (myMap.checkNextGridSpace(sensorAdjustment) == true)
		{
			forwardsDistance = myMap.determineDistanceToFinish(sensorAdjustment);
		}
	}

	if (checkSideSpaceLeft(5) == true)
	{
		if (myMap.checkNextGridSpace(sensorAdjustment+3) == true)
		{
			leftDistance = myMap.determineDistanceToFinish(sensorAdjustment+3);
		}
	}

	if (checkSideSpaceRight(5) == true)
	{
		if (myMap.checkNextGridSpace(sensorAdjustment+1) == true)
		{
			rightDistance = myMap.determineDistanceToFinish(sensorAdjustment+1);
		}
	}

	// Left smaller than Right
	if (leftDistance <= rightDistance)
	{
		if (forwardsDistance <= leftDistance)
		{
			currentState = RobotState::STATE_FORWARD;
			Serial.println("FORWARD 1");
			Serial.print(leftDistance);
			Serial.print(", ");
			Serial.print(forwardsDistance);
			Serial.print(", ");
			Serial.println(rightDistance);
		}
		else
		{
			currentState = RobotState::STATE_LEFT_AND_FORWARD;
			Serial.println("LEFT AND FORWARD");
			Serial.print(leftDistance);
			Serial.print(", ");
			Serial.print(forwardsDistance);
			Serial.print(", ");
			Serial.println(rightDistance);
		}
	}
	// Right smaller than left
	else
	{
		if (forwardsDistance <= rightDistance)
		{
			currentState = RobotState::STATE_FORWARD;
			Serial.println("FORWARD 2");
			Serial.print(leftDistance);
			Serial.print(", ");
			Serial.print(forwardsDistance);
			Serial.print(", ");
			Serial.println(rightDistance);
		}
		else
		{
			currentState = RobotState::STATE_RIGHT_AND_FORWARD;
			Serial.println("RIGHT AND FORWARD");
			Serial.print(leftDistance);
			Serial.print(", ");
			Serial.print(forwardsDistance);
			Serial.print(", ");
			Serial.println(rightDistance);
		}
	}
}

/**
 * @brief Convert bearing into a numbering system to determine the direction
 * for the finish location distance, using integer maths
 * 
 * @return int - The direction the robot is facing
 */
int Robot::adjustedMapDirection()
{
	if (bearing == 0)
	{
		return 0;
	}
	else if (bearing == 90)
	{
		return 1;
	}
	else if (bearing == 180)
	{
		return 2;
	}
	else if (bearing == 270)
	{
		return 3;
	}
	
	return -1;
}

/**
 * @brief Stops the robot from moving
 */
void Robot::stopMoving()
{
	myMotors.stopDriving();
	//this->currentState = this->STATE_STOP;
}