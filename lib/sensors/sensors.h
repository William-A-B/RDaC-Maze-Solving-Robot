#ifndef SENSORS_H
#define SENSORS_H

/**
 * @file sensors.h
 * @author Y3905304
 * @brief Sensors class header file, contains all the functions and variables for the sensors class
 * 
 */

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"

// Offsets for the sensors so all the readings are effectively from the very centre of the robot
#define FRONT_SENSOR_OFFSET 7.7f
#define REAR_SENSOR_OFFSET 11.3f
#define LEFT_SENSOR_OFFSET 10.75f
#define RIGHT_SENSOR_OFFSET 10.75f

class Sensors
{
public:

    /**
     * @brief Wrapper to call all functions to read and calculate the infrared sensor data
     * 
     * Call get_front_IR_distance() and get_back_IR_distance() to get the distance
     * from the front and back IR sensors after this function has been called
     */
    void run_IR_sensors();

    /**
     * @brief Wrapper to call all functions to read and calculate the ultrasonic sensor data
     * 
     * Call get_left_usonic_distance() and get_right_usonic_distance() to get the distance
     * from the left and right usonic sensors after this function has been called
     */
    void run_usonic_sensors();

    /**
     * @brief Get the front IR distance after calling run_IR_sensors()
     * 
     * @return float The distance measured by the front sensor in cm
     */
    float get_front_IR_distance();

    /**
     * @brief Get the back IR distance after calling run_IR_sensors()
     * 
     * @return float The distance measured by the back sensor in cm
     */
    float get_back_IR_distance();

    /**
     * @brief Get the left usonic distance after calling run_usonic_sensors()
     * 
     * @return float The distance measured by the left sensor in cm
     */
    float get_right_usonic_distance();

    /**
     * @brief Get the right usonic distance after calling run_usonic_sensors()
     * 
     * @return float The distance measured by the right sensor in cm
     */
    float get_left_usonic_distance();

    /**
     * @brief Read data from the front infrared sensors and take multiple readings to calculate and average
     * 
     * @param num_sensor_readings - the number of readings to take and average
     * @return float              - the average distance measured by the front sensor in cm
     */
    float read_averaged_IR_sensor_front(int num_sensor_readings);

    /**
     * @brief Read data from the back infrared sensors and take multiple readings to calculate and average
     * 
     * @param num_sensor_readings - the number of readings to take and average
     * @return float              - the average distance measured by the back sensor in cm
     */
    float read_averaged_IR_sensor_back(int num_sensor_readings);

    /**
     * @brief Read data from the left ultrasonic sensors and take multiple readings to calculate and average
     * 
     * @param num_sensor_readings - the number of readings to take and average
     * @return float              - the average distance measured by the left sensor in cm
     */
    float read_averaged_usonic_sensor_left(int num_sensor_readings);
    
    /**
     * @brief Read data from the right ultrasonic sensors and take multiple readings to calculate and average
     * 
     * @param num_sensor_readings - the number of readings to take and average
     * @return float              - the average distance measured by the right sensor in cm
     */
    float read_averaged_usonic_sensor_right(int num_sensor_readings);

private:

    /**
     * @brief Read the data from the infrared sensors on the I2C bus
     * 
     * @param front_sensor whether to read the data for the front or back sensor
     * True = front sensor
     * False = back sensor
     */
    void read_IR_data(bool front_sensor);

    /**
     * @brief Read the data from the ultrasonic sensors on the I2C bus
     * 
     * @param left_sensor whether to read the data for the left or right sensor
     * True = left sensor
     * False = right sensor
     */
    float read_usonic_data(bool left_sensor);

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
    float calculate_infrared_distance(char cmd[2], bool front);

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
    float calculate_usonic_distance(float usonic_duration);

    /**
     * @brief Print the data from the infrared sensors to the serial monitor
     * 
     * @param infrared_distance the distance measured by the infrared sensors
     */
    void print_IR_data(float infrared_distance[2]);

    /**
     * @brief Print the data from the ultrasonic sensors to the serial monitor
     * 
     * @param usonic_distance the distance measured by the ultrasonic sensors
     */
    void print_usonic_data(float usonic_distance[2]);

    // Variables to store the data from the sensors
    float infrared_distance[2] = {0};
    int usonic_duration[2];
    float usonic_distance[2];
    float average_infared_distance;
    float average_usonic_distance;

};



#endif
