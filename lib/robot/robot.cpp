#include "robot.h"



/**
 * @brief Function used to test various features. Any code implementation is often temporary
 * But may be promoted up to its own function at later use.
 */
void Robot::test()
{
	// Serial.println(mySensors.read_averaged_IR_sensor_front(1));

	// wait_us(100);

	// float initialSensors = mySensors.read_averaged_IR_sensor_front(1);

	// Serial.println("\n\nACTUAL DISTANCE LEFT");
	// Serial.println(myMotors.get_distance_travelled_left());
	// Serial.println("\nACTUAL DISTANCE RIGHT");
	// Serial.println(myMotors.get_distance_travelled_right());
	// Serial.println("\nFront Sensor Distance");
	// Serial.println(mySensors.read_averaged_IR_sensor_front(1));
	// Serial.println("Initial Front sensor value");
	// Serial.println(initialSensors);


	// moveRobot(40.0f);

	// float finalSensors = mySensors.read_averaged_IR_sensor_front(1);

	// Serial.println("\n\nACTUAL DISTANCE LEFT");
	// Serial.println(myMotors.get_distance_travelled_left());
	// Serial.println("\nACTUAL DISTANCE RIGHT");
	// Serial.println(myMotors.get_distance_travelled_right());
	// Serial.println("\nDifference between left - right");
	// Serial.println(myMotors.get_distance_travelled_left()-myMotors.get_distance_travelled_right());
	// Serial.println("\nFront Sensor Distance");
	// Serial.println(mySensors.read_averaged_IR_sensor_front(1));
	// Serial.println("Final Front sensor value");
	// Serial.println(finalSensors);
	// Serial.println("\nDistance Moved According to sensors");
	// Serial.println(initialSensors - finalSensors);

	

	// long int i = 0;
	

	// while (myMotors.get_distance_travelled_right() <= 70.0f)
	// {
	// 	if (i == 0)
	// 	{
	// 		this->driveForwards();
	// 	}


	// 	i++;
	// }
	// this->stopMoving();

	// this->driveForwards();







	
	// // START
	// calculateStartingLocation();

	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(10.0f);

	// updateCoordinateLocation();

	// mySensors.read_averaged_IR_sensor_front(1);
	// mySensors.read_averaged_IR_sensor_back(1);
	// mySensors.read_averaged_usonic_sensor_left(1);
	// mySensors.read_averaged_usonic_sensor_right(1);


	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(10.0f);

	// updateCoordinateLocation();

	// mySensors.read_averaged_IR_sensor_front(1);
	// mySensors.read_averaged_IR_sensor_back(1);
	// mySensors.read_averaged_usonic_sensor_left(1);
	// mySensors.read_averaged_usonic_sensor_right(1);

	// // LEFT 1
	// rotateRobot(-90);

	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(20.0f);

	// updateCoordinateLocation();

	// // LEFT 2
	// rotateRobot(-90);

	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(20.0f);

	// updateCoordinateLocation();

	// // LEFT 3
	// rotateRobot(-90);

	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(20.0f);

	// updateCoordinateLocation();

	// myMap.displayRobotHistory();


	// Serial.print(mySensors.read_averaged_IR_sensor_front(1));
	// Serial.print(", ");
	// Serial.println(mySensors.read_averaged_IR_sensor_back(1));
	Serial.print(mySensors.read_averaged_usonic_sensor_left(1));
	Serial.print(", ");
	Serial.println(mySensors.read_averaged_usonic_sensor_right(1));

	wait_us(200000);
	

	// // START
	// calculateStartingLocation();

	// processSensorInfo();

	// // MOVE 1
	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(5.0f);

	// updateCoordinateLocation();

	// processSensorInfo();

	// // MOVE 2
	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(5.0f);

	// updateCoordinateLocation();

	// processSensorInfo();

	// // MOVE 3
	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(5.0f);

	// updateCoordinateLocation();

	// processSensorInfo();

	// // MOVE 4
	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(5.0f);

	// updateCoordinateLocation();

	// processSensorInfo();

	// // MOVE 5
	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(5.0f);

	// updateCoordinateLocation();

	// processSensorInfo();

	// // MOVE 6
	// initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	// initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// moveRobot(5.0f);

	// updateCoordinateLocation();

	// processSensorInfo();


	// myMap.displayRobotHistory();
	// myMap.displayMap();

	// wait_us(10000000);


}

void Robot::processSensorInfo()
{
	float frontSensorDistance = mySensors.read_averaged_IR_sensor_front(1);
	float backSensorDistance = mySensors.read_averaged_IR_sensor_back(1);
	float leftSensorDistance = mySensors.read_averaged_usonic_sensor_left(1);
	float rightSensorDistance = mySensors.read_averaged_usonic_sensor_right(1);

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


/**
 * @brief Run once upon powering on the robot.
 * Sets up sections of robot which only ever need to be set once
 */
void Robot::initialSetup()
{
    // Calibrate the motors for use
	myMotors.setup();

	// Set the default direction for the robot to move in
	myMotors.set_direction(myMotors.DIR_FORWARDS);

	// Attach the interrupts for the encoders on the motors
	myMotors.attach_encoder_interrupts();

	myMap.initialSetup(0, 0);

	// // Setup the occupancy grid array to initialise the maze prior to the robot moving
	// myMap.setupOccupancyGrid();
	
	//my_robot.calculateStartingLocation();

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
	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, HIGH);
	digitalWrite(LEDB, HIGH);
}

/**
 * @brief Run once the robot is placed at the starting position in the maze
 * Is a state within the state machine, can be called again to restart the robots algorithms
 */
void Robot::setup()
{
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

	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, HIGH);
	digitalWrite(LEDB, HIGH);

	// this->moveRobot(1.0f);
	// wait_us(4000000);

	this->currentState = this->STATE_SOLVE;
	



	// this->moveRobot(20.0f);
	// this->rotateRobot(90);
	// this->moveRobot(20.0f);
}

/**
 * @brief Robot starts solving the maze and finding its way to the end
 * Runs all algorithms for solving and mapping from this function
 * Part of state SOLVE within the state machine
 */
void Robot::solveMaze()
{
	algorithm = DIRECT_AND_AROUND;

	if (algorithm == DIRECT_AND_AROUND)
	{
		runDirectAlgorithm();
	}
	else if (algorithm == FOLLOW_WALL)
	{
		runWallFollowingAlgorithm();
	}
	else if (algorithm == NAVIGATE_MAP)
	{

	}
}

/**
 * @brief Runs the algorithm for the robot to solve the maze via moving
 * directly forwards until it reaches an obstacle then it turns into the
 * next free direction and repeats
 */
void Robot::runDirectAlgorithm()
{
	bool canMoveForwards = false;
	bool reverse = false;

	canMoveForwards = this->checkRouteAhead(5.0f);

	if (canMoveForwards)
	{
		moveHelper();
		// if (driveForwardsStarted == false)
		// {
		// 	if (robotBLE.connectToClient() == true)
		// 	{
		// 		Serial.println("Connected to Client, starting going forwards");
		// 	}
		// 	this->driveForwards();

		// 	initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
		// 	initialDistanceMovedRight = myMotors.get_distance_travelled_right();
		// 	driveForwardsStarted = true;
		// }
		
		if (robotBLE.isClientConnected() == true)
		{
			robotBLE.pingService(0x01);
			Serial.println("Connected - Robot going forwards");
		}

		digitalWrite(LEDG, LOW);
		digitalWrite(LEDR, HIGH);
		digitalWrite(LEDB, HIGH);
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

		if (this->checkSideSpaceLeft(1))
		{
			this->rotateRobot(-90);
			digitalWrite(LEDB, LOW);
			digitalWrite(LEDR, HIGH);
			digitalWrite(LEDG, HIGH);
		}
		else if (this->checkSideSpaceRight(1))
		{
			this->rotateRobot(90);
			digitalWrite(LEDB, LOW);
			digitalWrite(LEDR, HIGH);
			digitalWrite(LEDG, HIGH);
		}
		else
		{
			reverse = true;
			while (reverse)
			{
				this->driveBackwards();
				digitalWrite(LEDR, LOW);
				digitalWrite(LEDG, HIGH);
				digitalWrite(LEDB, HIGH);
				wait_us(10000);
				// if (mySensors.get_back_IR_distance() < MIN_IR_DIST_REAR)
				// {
				// 	reverse = false;
				// 	this->stopMoving();
				// 	digitalWrite(LEDR, LOW);
				// 	digitalWrite(LEDG, LOW);
				// 	digitalWrite(LEDB, LOW);
				// }

				if (this->checkSideSpaceLeft(1))
				{
					reverse = false;
					this->rotateRobot(-90);
				}
				else if (this->checkSideSpaceRight(1))
				{
					reverse = false;
					this->rotateRobot(90);
				}
				
			}
		}
	}
}

/**
 * @brief Runs the algorithm for the robot to solve the maze via following
 * the left hand wall until it reaches the end.
 */
void Robot::runWallFollowingAlgorithm()
{
	bool reachedNextWall = false;

	while (!this->checkSideSpaceLeft(1))
	{	
		reachedNextWall = false;
		//this->driveForwards();
		if (this->checkRouteAhead(-1))
		{
			digitalWrite(LEDG, LOW);
			digitalWrite(LEDR, HIGH);
			digitalWrite(LEDB, HIGH);
			Serial.println("Free space ahead");
			Serial.println("Driving forwards\n\n");
			this->driveForwards();
		}
		else
		{
			if (!this->checkSideSpaceLeft(1))
			{
				digitalWrite(LEDR, LOW);
				digitalWrite(LEDG, HIGH);
				digitalWrite(LEDB, HIGH);
				Serial.println("No space ahead and wall on left");
				Serial.println("Turning Right\n\n");
				this->rotateRobot(90);
				this->driveForwards();
			}
		}
	}

	if (this->checkSideSpaceLeft(1))
	{
		digitalWrite(LEDB, LOW);
		digitalWrite(LEDR, HIGH);
		digitalWrite(LEDG, HIGH);
		Serial.println("Reached end of left wall");
		Serial.println("Turning left to continue along next wall\n\n");
		
		this->moveRobot(11.0f);
		this->rotateRobot(-90);
		this->moveRobot(11.0f);

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

void Robot::moveHelper()
{
	initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	moveRobot(5.0f);

	correctOrientation();

	updateCoordinateLocation();

	processSensorInfo();
}

/**
 * @brief Starts the robot driving forwards at its default speed of 0.5
 */
void Robot::driveForwards()
{
	myMotors.set_direction(myMotors.DIR_FORWARDS);
	myMotors.set_speed(DEFAULT_ROBOT_SPEED);
	myMotors.drive(0);
}

/**
 * @brief Starts the robot driving backwards at its default speed of 0.5
 */
void Robot::driveBackwards()
{
	myMotors.set_direction(myMotors.DIR_BACKWARDS);
	myMotors.set_speed(DEFAULT_ROBOT_SPEED);
	myMotors.drive(0);
}

/**
 * @brief Stops the robot from moving
 */
void Robot::stopMoving()
{
	myMotors.stop_driving();
	//this->currentState = this->STATE_STOP;
}

/**
 * @brief Ends all processes on the robot and stops the state machine
 */
void Robot::end()
{
    this->stopMoving();
}

/**
 * @brief Moves the robot a given distance, if the value given is negative the robot will move backwards
 * 
 * @param distanceToMove 	The number of mm to move the robot
 */
void Robot::moveRobot(float distanceToMove)
{
    float initialDistanceMovedLeft = 0.0f;
	float initialDistanceMovedRight = 0.0f;
	float distanceMovedLeft = 0.0f;
	float distanceMovedRight = 0.0f;
	float differenceBetweenMotorDistances = 0.0f;

	bool routeAheadFree = false;
	bool reachedDistance = false;

	int loopCount = 0;


	// Stop robot moving
	myMotors.stop_driving();
	
	// Check that space ahead the robot is wanting to move is free
	routeAheadFree = this->checkRouteAhead(distanceToMove);
    //myMap.checkRouteAheadInMap(this->bearing, distanceToMove);

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
		myMotors.set_direction(myMotors.DIR_FORWARDS);
	}
	else if (distanceToMove < 0)
	{
		// Since the distance is negative, make it positive for using in the calculation later
		distanceToMove = distanceToMove * -1.0f;
		myMotors.set_direction(myMotors.DIR_BACKWARDS);
	}
	else
	{
		return;
	}

    // Calculate current distance the robot has moved before the robot starts moving the set amount
	initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	initialDistanceMovedRight = myMotors.get_distance_travelled_right();

    // Start robot moving
	myMotors.set_speed(DEFAULT_ROBOT_SPEED);
	myMotors.drive(0);


	// Loop until robot has moved the distance specified
	// Loops until the sum of the distances moved by both motors is equal to or greater than
	// the set distance to move multiplied by two since the sum of the two wheels is taken.
    do 
	{
		// Get left wheel distance moved whilst turning robot
		distanceMovedLeft = myMotors.get_distance_travelled_left() - initialDistanceMovedLeft;

		// Get right wheel distance moved whilst turning robot
		distanceMovedRight = myMotors.get_distance_travelled_right() - initialDistanceMovedRight;

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

	Serial.println(loopCount);

	// // Loop until robot has moved the distance specified
	// // Loops until the sum of the distances moved by both motors is equal to or greater than
	// // the set distance to move multiplied by two since the sum of the two wheels is taken.
    // while (fabs(differenceBetweenMotorDistances) < (distanceToMove * 2.0f))
	// {
	// 	// Get left wheel distance moved whilst turning robot
	// 	distanceMovedLeft = myMotors.get_distance_travelled_left() - initialDistanceMovedLeft;

	// 	// Get right wheel distance moved whilst turning robot
	// 	distanceMovedRight = myMotors.get_distance_travelled_right() - initialDistanceMovedRight;

	// 	// Get sum/difference between both motors
	// 	differenceBetweenMotorDistances = distanceMovedLeft + distanceMovedRight;

	// 	// Delay to ensure polling of interrupt is not backlogged
	// 	wait_us(100);
	// }

    // Stop robot moving and reset direction to forwards
	myMotors.stop_driving();
    myMotors.set_direction(myMotors.DIR_FORWARDS);
}

/**
 * @brief Rotates the robot about a point a set number of degrees
 *
 * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
 */
void Robot::rotateRobot(int degrees)
{
	rotateRobot(degrees, false);
}

/**
 * @brief Rotates the robot about a point a set number of degrees
 *
 * @param degrees 	The number of degrees to rotate, positive = clockwise direction, negative = anticlockwise
 */
void Robot::rotateRobot(int degrees, bool ignoreBearingUpdate)
{
	float initialDistanceMovedLeft = 0.0f;
	float initialDistanceMovedRight = 0.0f;
	float distanceMovedWhileTurningLeft = 0.0f;
	float distanceMovedWhileTurningRight = 0.0f;
	float differenceBetweenMotorDistances = 0.0f;

	// Stop robot moving
	myMotors.stop_driving();

	if (ignoreBearingUpdate = false)
	{
		this->updateBearing(degrees);
	}
    
	// Call direction change function on motors depending on the angle direction
	if (degrees > 0)
	{
		myMotors.set_direction(myMotors.DIR_CLOCKWISE);
	}
	else if (degrees < 0)
	{
		degrees = degrees * -1;
		myMotors.set_direction(myMotors.DIR_ANTICLOCKWISE);
	}
	else
	{
		return;
	}

	// Calculate current distance the robot has moved before the robot starts turning the set amount
	initialDistanceMovedLeft = myMotors.get_distance_travelled_left();
	initialDistanceMovedRight = myMotors.get_distance_travelled_right();

	// Calculate equivalent arc of circle
	// Left wheel 80mm out from centre
	// Right wheel 80mm out
	// arc distance = (degrees/360) * 2*pi*radius
	// Distance in mm
	float arc_distance_to_turn = ((float)degrees / 360.0f) * 2.0f * 3.141f * ROBOT_WHEEL_RADIUS;

	// Start robot moving
	myMotors.set_speed(DEFAULT_ROBOT_SPEED);
	myMotors.drive(0);

	// While distance moved < arc length
	// loop
	// Get distance moved by wheels
	// (distance moved could be an average of the two wheels)
	// delay between distance moved polls
	while (fabs(differenceBetweenMotorDistances) < (arc_distance_to_turn * 2.0f))
	{
		// Get left wheel distance moved whilst turning robot
		distanceMovedWhileTurningLeft = myMotors.get_distance_travelled_left() - initialDistanceMovedLeft;

		// Get right wheel distance moved whilst turning robot
		distanceMovedWhileTurningRight = myMotors.get_distance_travelled_right() - initialDistanceMovedRight;

		// Get difference between both motors
		differenceBetweenMotorDistances = distanceMovedWhileTurningLeft - distanceMovedWhileTurningRight;

		// Delay to ensure polling of interrupt is not backlogged
		wait_us(100);
	}

	// Stop robot moving and reset direction to forwards
	myMotors.stop_driving();
	myMotors.set_direction(myMotors.DIR_FORWARDS);
}

/**
 * @brief Calculates the starting position of the robot
 * Checks surrounding areas based on sensors
 * And sets the coordinate values based on the sensors
 */
void Robot::calculateStartingLocation()
{
	float frontSensorDistance = mySensors.read_averaged_IR_sensor_front(5);
	float backSensorDistance = mySensors.read_averaged_IR_sensor_back(5);
	float leftSensorDistance = mySensors.read_averaged_usonic_sensor_left(5);
	float rightSensorDistance = mySensors.read_averaged_usonic_sensor_right(5);

    // Calculate direction robot is facing
    // Assume the robot is facing towards finish from centre of start location

    // Set coordinate position based on sensor readings
    this->currentPosition.xCoordinate = leftSensorDistance;
    this->currentPosition.yCoordinate = backSensorDistance;

	myMap.updateRobotPosition(currentPosition.xCoordinate, currentPosition.yCoordinate);

	Serial.println("\n(X Coordinate, Y Coordinate");
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
    float xDistanceToMove = 0.0f;
    float yDistanceToMove = 0.0f;
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
	float distanceToObjectsFront = mySensors.read_averaged_IR_sensor_front(5);

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
	
	if (leftSensorDistance > MIN_USONIC_DIST+5.0f)
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

	if (rightSensorDistance > MIN_USONIC_DIST+5.0f)
	{
		return true;
	}
	return false;
}


void Robot::updateCoordinateLocation()
{
	float distanceMovedLeft = 0.0f;
	float distanceMovedRight = 0.0f;
	float averageDistanceMoved = 0.0f;

	distanceMovedLeft = myMotors.get_distance_travelled_left() - initialDistanceMovedLeft;
	distanceMovedRight = myMotors.get_distance_travelled_right() - initialDistanceMovedRight;

	averageDistanceMoved = (distanceMovedLeft + distanceMovedRight) / 2.0f;

	Serial.println("\nInitial Coordinates");
	Serial.println("Bearing");
	Serial.println(bearing);
	Serial.println("x coordinate");
	Serial.println(currentPosition.xCoordinate);
	Serial.println("y coordinate");
	Serial.println(currentPosition.yCoordinate);

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

	Serial.println("\nFinal Coordinates");
	Serial.println("Bearing");
	Serial.println(bearing);
	Serial.println("x coordinate");
	Serial.println(currentPosition.xCoordinate);
	Serial.println("y coordinate");
	Serial.println(currentPosition.yCoordinate);

	robotBLE.updateRobotLocationInfo(bearing, currentPosition.xCoordinate, currentPosition.yCoordinate);

	myMap.updateRobotPosition(currentPosition.xCoordinate, currentPosition.yCoordinate);
}

void Robot::correctOrientation()
{
	float turningAngle = 2.0f;
	float originalSensorDistance = mySensors.read_averaged_IR_sensor_front(1);

	rotateRobot((int)(-1.0f*turningAngle), true);

	float adjLeftSensorDistance = mySensors.read_averaged_IR_sensor_front(1);

	rotateRobot((int)(2.0f*turningAngle), true);

	float adjRightSensorDistance = mySensors.read_averaged_IR_sensor_front(1);

	Serial.println("\n\ncorrectOrientation Sensor Readings\n");
	Serial.println("Left, Original, Right");
	Serial.println(adjLeftSensorDistance);
	Serial.println(originalSensorDistance);
	Serial.println(adjRightSensorDistance);
	
	if (adjLeftSensorDistance > originalSensorDistance)
	{
		if (adjRightSensorDistance < originalSensorDistance)
		{
			Serial.println("\ncorrectOrientation 1\n");
			correctOrientationHelper(originalSensorDistance, adjRightSensorDistance, turningAngle);
		}
		else
		{
			Serial.println("\ncorrectOrientation 2\n");
			// Turning angle too big
			// Two intercepts are either side of turning line
		}
	}
	else
	{
		if (adjRightSensorDistance > originalSensorDistance)
		{
			Serial.println("\ncorrectOrientation 3\n");
			//func(); May need to adjust values passed
		}
		else
		{
			Serial.println("\ncorrectOrientation 4\n");
			// Turning angle too big
			// Two intercepts are either side of turning line
			// I think
		}
	}

}

void Robot::correctOrientationHelper(float a, float b, float aC)
{
	float c = sqrtf((a*a) + (b*b) - (2*a*b*cosf(aC)));

	float aA = (a*sinf(aC))/c;

	float rotAngle = aC + aA - 90.0f;

	rotateRobot(rotAngle, true);

	Serial.println("\n\nAdjustment Angle\n");
	Serial.println(rotAngle);

}








// EXTRA CODE OR OLD CODE OR TEST CODE




// /**
//  * @brief Called from the main arduino loop() function
//  * The main function to run the robot and all its processes.
//  * Is continuously called whilst the robot is set to continue running
//  *
//  * @return true		The robot should continue running its program
//  * @return false	The robot stops running
//  */
// bool Robot::run()
// {
// 	// Run the functions to read values from the infrared sensors
// 	mySensors.run_IR_sensors();

// 	// wait_us(500000);

// 	// Run the functions to read values from the ultrasonic sensors
// 	mySensors.run_usonic_sensors();

// 	// Set the speed of the motors to half the max speed
// 	myMotors.set_speed(0.5f);

// 	// Rotate on spot by 45 degrees for two complete rotations
// 	// Detect obstacles within min distance
// 	// Add into occupancy map
// 	// Decide where to move

// 	// Drive the robot forwards
// 	// 0 seconds means continue forever until told otherwise
// 	myMotors.drive(0);

// 	wait_us(1000000);

// 	this->rotateRobot(90);

// 	// Set the speed of the motors to half the max speed
// 	myMotors.set_speed(0.5f);

// 	// Drive the robot forwards
// 	// 0 seconds means continue forever until told otherwise
// 	myMotors.drive(0);

// 	wait_us(1000000);

// 	this->rotateRobot(-90);

// 	// Detect obstacles in the nearby area
// 	// my_robot.detect_obstacle();

// 	// Avoid obstacles in the nearby area by moving away from them
// 	// my_robot.avoid_obstacle();

// 	// Display the occupancy grid in a readable format
// 	// my_robot.displayMap();

// 	// Return true to continue running the loop
// 	return true;
// }




// /**
//  * @brief Checks with the sensors whether objects are within
//  * range of the robot
//  *
//  * @return objects - bitfield defining the surrounding objects
//  * 0 - No Objects
//  * 1 - Front Object
//  * 2 - Rear Object
//  * 3 - Left Object
//  * 4 - Right Object
//  * 5 - Front and Rear Object
//  * 6 - Front and Left Object
//  * 7 - Front and Right Object
//  * 8 - Rear and Left Object
//  * 9 - Rear and Right Object
//  * 10 - Left and Right Objects
//  * 11 - Front and Rear and Left Objects
//  * 12 - Front and Rear and Right Objects
//  * 13 - Front and Left and Right Objects
//  * 14 - Rear and Left and Right Objects
//  * 15 - Front and Rear and Left and Right Objects
//  */
// void Robot::detect_obstacle()
// {
// 	if (mySensors.get_front_IR_distance() < MIN_IR_DIST)
// 	{
// 		if (mySensors.get_left_usonic_distance() < MIN_USONIC_DIST && mySensors.get_right_usonic_distance() < MIN_USONIC_DIST)
// 		{
// 			objects = 13;
// 		}
// 		else if (mySensors.get_left_usonic_distance() < MIN_USONIC_DIST)
// 		{
// 			objects = 6;
// 		}
// 		else if (mySensors.get_right_usonic_distance() < MIN_USONIC_DIST)
// 		{
// 			objects = 7;
// 		}
// 		else
// 		{
// 			objects = 1;
// 		}
// 	}
// 	else
// 	{
// 		objects = 0;
// 	}
// }

// /**
//  * @brief Tell the robot to avoid the obstacles
//  * that were detected in the detect_obstacle() function
//  *
//  */
// void Robot::avoid_obstacle()
// {
// 	switch (objects)
// 	{
// 	case 0:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		// myMotors.set_speed(1.0f);
// 		break;
// 	case 1:
// 		myMotors.set_direction(myMotors.DIR_ANTICLOCKWISE);
// 		myMotors.set_speed(0.5f);
// 		// my_robot.turn_left();
// 		break;
// 	case 2:
// 		myMotors.set_direction(myMotors.DIR_CLOCKWISE);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 3:
// 		myMotors.set_direction(myMotors.DIR_ANTICLOCKWISE);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 4:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 5:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 6:
// 		myMotors.set_direction(myMotors.DIR_CLOCKWISE);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 7:
// 		myMotors.set_direction(myMotors.DIR_ANTICLOCKWISE);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 8:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 9:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 10:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 11:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 12:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 13:
// 		myMotors.set_direction(myMotors.DIR_BACKWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 14:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	case 15:
// 		myMotors.set_direction(myMotors.DIR_FORWARDS);
// 		myMotors.set_speed(0.5f);
// 		break;
// 	}
// }





/**
 * mbed::callback for class interrupts
 * lambda function to call the class interrupt function
 */

	// switch (currentState)
	// {
	// 	case STATE_STOP:
	// 		my_robot.stopMoving();
	// 		break;
	// 	case STATE_FORWARD:
	// 		break;
	// 	case STATE_BACKWARD:
	// 		break;
	// 	case STATE_LEFT:
	// 		my_robot.turn_left();
	// 		break;
	// 	case STATE_RIGHT:
	// 		my_robot.turn_right();
	// 		break;
	// }


	// // Debug statements for calibrating the encoders to check the distance travelled
	// Serial.println("Distance Travelled Left in mm:");
	// Serial.print(get_distance_travelled_left());
	// Serial.print(" mm\n");
	// Serial.println("Distance Travelled Left in cm:");
	// Serial.print(get_distance_travelled_left() / 10);
	// Serial.print(" cm\n\n");

	// Serial.println("Distance Travelled Right in mm:");
	// Serial.print(get_distance_travelled_right());
	// Serial.print(" mm\n");
	// Serial.println("Distance Travelled Right in cm:");
	// Serial.print(get_distance_travelled_right() / 10);
	// Serial.print(" cm\n\n");

	// Serial.println("==========================================");
	// Serial.println("AVERAGE DISTANCE MOVED BETWEEN BOTH MOTORS");
	// Serial.println(((get_distance_travelled_left() + get_distance_travelled_right()) / 2));
	// Serial.println("==========================================");

	// if (get_encoder_revolutions_left() >= 1000)
	// {
	// 	Serial.println("Final Encoder Count");
	// 	Serial.println(get_encoder_revolutions_left());
	// 	return false;
	// }