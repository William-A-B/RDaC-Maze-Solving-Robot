#include "sensors.h"

mbed::I2C i2c(I2C_SDA, I2C_SCL);

const char mux_cmd_select_one = 0x01; // Mux select for front IR
const char mux_cmd_select_two = 0x02; // Mux select for rear IR
const char mux_addr = 0xEE;
const char bus_1 = 1;
const char shift_bit_register = 0x35;
const int shift_bit = 2;
char cmd[2];



/**
 * @brief Alighing robot perpendicular to wall
 * Measure straight forwards,
 * Measure x degrees to the left
 * Measure x degrees to the right
 * Compare the distances to calculate shortest distance to wall
 * Turn to face the wall at angle that is the shortest distance
*/

/**
 * @brief Scale sensor distances to be measured from the centre of the robot
 * 
 */


void Sensors::run_IR_sensors()
{
	//float infrared_distance[2] = {0};
	read_IR_data(true);
	infrared_distance[0] = calculate_infrared_distance(cmd);

	read_IR_data(false);
	infrared_distance[1] = calculate_infrared_distance(cmd);

	print_IR_data(infrared_distance);

    //return infrared_distance;
}

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

void Sensors::read_IR_data(bool front_sensor)
{
if (front_sensor == true)
{
	i2c.write(mux_addr, &mux_cmd_select_one, bus_1);

	cmd[0] = 0x5E;
	cmd[1] = 0x00;
	
	i2c.write(0x80, cmd, 1);
	wait_us(10000);
	i2c.read(0x80, cmd, 2);
}
else
{
	i2c.write(mux_addr, &mux_cmd_select_two, bus_1);

	cmd[0] = 0x5E;
	cmd[1] = 0x00;
	
	i2c.write(0x80, cmd, 1);
	wait_us(10000);
	i2c.read(0x80, cmd, 2);
}
}

int Sensors::read_usonic_data(bool left_sensor)
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

float Sensors::calculate_infrared_distance(char cmd[2]) 
{

	float distance = 0.0f;

	distance = (cmd[0] * 16 + cmd[1])/16/(float)pow(2,shift_bit);

	return distance;
}

int Sensors::calculate_usonic_distance(int usonic_duration)
{
	int distance = 0;
	distance = usonic_duration / 29 / 2;
	return distance;
}

void Sensors::print_IR_data(float infrared_distance[2])
{
	Serial.println("-------------------------------");
	Serial.print("Front IR distance = ");
	Serial.println(infrared_distance[0]);
	Serial.print("Rear IR distance = ");
	Serial.println(infrared_distance[1]);
	Serial.println("-------------------------------");
}

void Sensors::print_usonic_data(int usonic_distance[2])
{
	Serial.print("Left usonic distance =  ");
	Serial.print(usonic_distance[0]);
	Serial.println(" cm.");

	Serial.print("Right usonic distance =  ");
	Serial.print(usonic_distance[1]);
	Serial.println(" cm.");
}

/**
 * @brief 
 * 
 * @return float distance in cm
 */
float Sensors::get_front_IR_distance()
{
	return infrared_distance[0];
}

/**
 * @brief 
 * 
 * @return float distance in cm
 */
float Sensors::get_back_IR_distance()
{
	return infrared_distance[1];
}

/**
 * @brief 
 * 
 * @return int 
 */
int Sensors::get_left_usonic_distance()
{
	return usonic_distance[0];
}

int Sensors::get_right_usonic_distance()
{
	return usonic_distance[1];
}