#include "sensors.h"

mbed::I2C i2c(I2C_SDA, I2C_SCL);

mbed::DigitalInOut usonic_left_sensor(USONIC_SENSOR_PIN_LEFT);
mbed::DigitalInOut usonic_right_sensor(USONIC_SENSOR_PIN_RIGHT);


const char mux_cmd_select_one = 0x01; // Mux select for front IR
const char mux_cmd_select_two = 0x02; // Mux select for rear IR
const char mux_addr = 0xEE;
const char bus_1 = 1;
const char shift_bit_register = 0x35;
const int shift_bit = 2;
char cmd[2];

/**
 * @brief Wrapper to call all functions to read and calculate the infrared sensor data
 * 
 * Call get_front_IR_distance() and get_back_IR_distance() to get the distance
 * from the front and back IR sensors after this function has been called
 */
void Sensors::run_IR_sensors()
{
	//float infrared_distance[2] = {0};
	read_IR_data(true);
	infrared_distance[0] = calculate_infrared_distance(cmd, true);

	read_IR_data(false);
	infrared_distance[1] = calculate_infrared_distance(cmd, false);

	print_IR_data(infrared_distance);

    //return infrared_distance;
}

/**
 * @brief Wrapper to call all functions to read and calculate the ultrasonic sensor data
 * 
 * Call get_left_usonic_distance() and get_right_usonic_distance() to get the distance
 * from the left and right usonic sensors after this function has been called
 */
void Sensors::run_usonic_sensors()
{
	//int usonic_duration[2], usonic_distance[2]; // Setup variables for results

	usonic_duration[0] = read_usonic_data(true);
	usonic_distance[0] = calculate_usonic_distance(usonic_duration[0]);

	usonic_duration[1] = read_usonic_data(false);
	usonic_distance[1] = calculate_usonic_distance(usonic_duration[1]);
	
	print_usonic_data(usonic_distance);

    //return usonic_distance;
}

/**
 * @brief Read the data from the infrared sensors on the I2C bus
 * 
 * @param front_sensor whether to read the data for the front or back sensor
 * True = front sensor
 * False = back sensor
 */
void Sensors::read_IR_data(bool front_sensor)
{
	if (front_sensor == true)
	{
		i2c.write(mux_addr, &mux_cmd_select_one, bus_1);

		// const char longDist = 0x02;
		// i2c.write(shift_bit_register, &longDist, 1);

		cmd[0] = 0x5E;
		cmd[1] = 0x5F;
		
		i2c.write(0x80, cmd, 1);
		wait_us(10000);
		i2c.read(0x80, cmd, 2);
	}
	else
	{
		i2c.write(mux_addr, &mux_cmd_select_two, bus_1);

		cmd[0] = 0x5E;
		cmd[1] = 0x5F;
		
		i2c.write(0x80, cmd, 1);
		wait_us(10000);
		i2c.read(0x80, cmd, 2);
	}
}

/**
 * @brief Read the data from the ultrasonic sensors on the I2C bus
 * 
 * @param left_sensor whether to read the data for the left or right sensor
 * True = left sensor
 * False = right sensor
 */
float Sensors::read_usonic_data(bool left_sensor)
{
	if (left_sensor == true)
	{
		// Set the pin to output, bring it low, then high, then low to generate pulse
		pinMode(USONIC_LEFT, OUTPUT);
		digitalWrite(USONIC_LEFT, LOW);
		delayMicroseconds(2);
		digitalWrite(USONIC_LEFT, HIGH);
		delayMicroseconds(15);
		digitalWrite(USONIC_LEFT, LOW);

		// usonic_left_sensor.output();
		// usonic_left_sensor.write(0);
		// wait_us(2);
		// usonic_left_sensor.write(1);
		// wait_us(15);
		// usonic_left_sensor.write(0);

		// usonic_left_sensor.input();
		// return 

		// The same pin is used to read back the returning signal, so must be set back to input
		pinMode(USONIC_LEFT, INPUT);
		return pulseIn(USONIC_LEFT, HIGH);
	}
	else if (left_sensor == false)
	{
		// Set the pin to output, bring it low, then high, then low to generate pulse
		pinMode(USONIC_RIGHT, OUTPUT);
		digitalWrite(USONIC_RIGHT, LOW);
		delayMicroseconds(2);
		digitalWrite(USONIC_RIGHT, HIGH);
		delayMicroseconds(15);
		digitalWrite(USONIC_RIGHT, LOW);

		// The same pin is used to read back the returning signal, so must be set back to input
		pinMode(USONIC_RIGHT, INPUT);
		return pulseIn(USONIC_RIGHT, HIGH);
	}

	return 0;

}

/**
 * @brief Calculate the distance from the infrared sensor data read from the I2C bus
 * Make sure the distance is correctly offset from the centre of the robot
 * 
 * @param cmd the data from the sensor
 * @param front_sensor whether to calculate the distance for the front or back sensor
 * True = front sensor
 * False = back sensor
 * @return float the distance measured by the sensor in cm
 */
float Sensors::calculate_infrared_distance(char cmd[2], bool front) 
{
	float distance = 0.0f;

	// distance = (cmd[0] * 16 + cmd[1])/16/powf(2,shift_bit);
	// distance = (cmd[0] * 16)/16/(float)pow(2,shift_bit);

	// cmd[0] = high bits, cmd[1] = low bits

	uint16_t merged = (cmd[0]<<4) | cmd[1];
	distance = (float)merged/64;


	// distance = (cmd[0]<<4);
	// distance = ((cmd[0]<<4) + ((cmd[1]>>(4+shift_bit))));
	// distance = (cmd[0] * 16);

	if (front == true)
	{
		distance = distance + FRONT_SENSOR_OFFSET;
	}
	else if (front == false)
	{
		distance = distance + REAR_SENSOR_OFFSET;
	}

	return distance;
}

/**
 * @brief Calculate the distance from the ultrasonic sensor data read from the I2C bus
 * Make sure the distance is correctly offset from the centre of the robot
 * 
 * @param usonic_duration the data from the sensor
 * @param left_sensor whether to calculate the distance for the left or right sensor
 * True = left sensor
 * False = right sensor
 * @return float the distance measured by the sensor in cm
 */
float Sensors::calculate_usonic_distance(float usonic_duration)
{
	float distance = 0;
	distance = usonic_duration / 29 / 2;
	distance = distance + LEFT_SENSOR_OFFSET;
	return distance;
}

/**
 * @brief Print the data from the infrared sensors to the serial monitor
 * 
 * @param infrared_distance the distance measured by the infrared sensors
 */
void Sensors::print_IR_data(float infrared_distance[2])
{
	Serial.println("-------------------------------");
	Serial.print("Front IR distance = ");
	Serial.println(infrared_distance[0]);
	Serial.print("Rear IR distance = ");
	Serial.println(infrared_distance[1]);
	Serial.println("-------------------------------");
}

/**
 * @brief Print the data from the ultrasonic sensors to the serial monitor
 * 
 * @param usonic_distance the distance measured by the ultrasonic sensors
 */
void Sensors::print_usonic_data(float usonic_distance[2])
{
	Serial.print("Left usonic distance =  ");
	Serial.print(usonic_distance[0]);
	Serial.println(" cm.");

	Serial.print("Right usonic distance =  ");
	Serial.print(usonic_distance[1]);
	Serial.println(" cm.");
}

/**
 * @brief Get the front IR distance after calling run_IR_sensors()
 * 
 * @return float The distance measured by the front sensor in cm
 */
float Sensors::get_front_IR_distance()
{
	return infrared_distance[0];
}

/**
 * @brief Get the back IR distance after calling run_IR_sensors()
 * 
 * @return float The distance measured by the back sensor in cm
 */
float Sensors::get_back_IR_distance()
{
	return infrared_distance[1];
}

/**
 * @brief Get the right usonic distance after calling run_usonic_sensors()
 * 
 * @return float The distance measured by the right sensor in cm
 */
float Sensors::get_left_usonic_distance()
{
	return usonic_distance[0];
}

/**
 * @brief Get the left usonic distance after calling run_usonic_sensors()
 * 
 * @return float The distance measured by the left sensor in cm
 */
float Sensors::get_right_usonic_distance()
{
	return usonic_distance[1];
}

/**
 * @brief Read data from the front infrared sensors and take multiple readings to calculate and average
 * 
 * @param num_sensor_readings - the number of readings to take and average
 * @return float              - the average distance measured by the front sensor in cm
 */
float Sensors::read_averaged_IR_sensor_front(int num_sensor_readings)
{
	this->average_infared_distance = 0.0f;
	for (int sensor_readings = 0; sensor_readings < num_sensor_readings; sensor_readings++)
	{
		read_IR_data(true);
		this->infrared_distance[0] = calculate_infrared_distance(cmd, true);

		this->average_infared_distance = this->average_infared_distance + this->infrared_distance[0];
	}

	this->average_infared_distance = (this->average_infared_distance/(float)num_sensor_readings);
	return this->average_infared_distance;
}
/**
 * @brief Read data from the back infrared sensors and take multiple readings to calculate and average
 * 
 * @param num_sensor_readings - the number of readings to take and average
 * @return float              - the average distance measured by the back sensor in cm
 */
float Sensors::read_averaged_IR_sensor_back(int num_sensor_readings)
{
	this->average_infared_distance = 0.0f;
	for (int sensor_readings = 0; sensor_readings < num_sensor_readings; sensor_readings++)
	{
		read_IR_data(false);
		this->infrared_distance[1] = calculate_infrared_distance(cmd, false);

		this->average_infared_distance = this->average_infared_distance + this->infrared_distance[1];
	}

	this->average_infared_distance = (this->average_infared_distance/(float)num_sensor_readings);
	return this->average_infared_distance;
}

/**
 * @brief Read data from the left ultrasonic sensors and take multiple readings to calculate and average
 * 
 * @param num_sensor_readings - the number of readings to take and average
 * @return float              - the average distance measured by the left sensor in cm
 */
float Sensors::read_averaged_usonic_sensor_left(int num_sensor_readings)
{
	this->average_usonic_distance = 0;
	for (int sensor_readings = 0; sensor_readings < num_sensor_readings; sensor_readings++)
	{
		this->usonic_duration[0] = read_usonic_data(true);
		this->usonic_distance[0] = calculate_usonic_distance(usonic_duration[0]);
		this->average_usonic_distance = this->average_usonic_distance + this->usonic_distance[0];
	}

	this->average_usonic_distance = (this->average_usonic_distance/(float)num_sensor_readings);
	return this->average_usonic_distance;
}
/**
 * @brief Read data from the right ultrasonic sensors and take multiple readings to calculate and average
 * 
 * @param num_sensor_readings - the number of readings to take and average
 * @return float              - the average distance measured by the right sensor in cm
 */
float Sensors::read_averaged_usonic_sensor_right(int num_sensor_readings)
{
	this->average_usonic_distance = 0;
	for (int sensor_readings = 0; sensor_readings < num_sensor_readings; sensor_readings++)
	{
		this->usonic_duration[1] = read_usonic_data(false);
		this->usonic_distance[1] = calculate_usonic_distance(usonic_duration[1]);
		this->average_usonic_distance = this->average_usonic_distance + this->usonic_distance[1];
	}

	this->average_usonic_distance = (this->average_usonic_distance/(float)num_sensor_readings);
	return this->average_usonic_distance;
}