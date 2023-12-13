#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"

class Sensors
{
public:

    void run_IR_sensors();
    void run_usonic_sensors();
    float get_front_IR_distance();
    float get_back_IR_distance();
    int get_right_usonic_distance();
    int get_left_usonic_distance();
    float read_averaged_IR_sensor_front(int num_sensor_readings);
    float read_averaged_IR_sensor_back(int num_sensor_readings);
    int read_averaged_usonic_sensor_left(int num_sensor_readings);
    int read_averaged_usonic_sensor_right(int num_sensor_readings);

private:
    void read_IR_data(bool front_sensor);
    int read_usonic_data(bool left_sensor);
    float calculate_infrared_distance(char cmd[2]);
    int calculate_usonic_distance(int usonic_duration);
    void print_IR_data(float infrared_distance[2]);
    void print_usonic_data(int usonic_distance[2]);

    float infrared_distance[2] = {0};
    int usonic_duration[2], usonic_distance[2];
    float average_infared_distance;
    int average_usonic_distance;

};



#endif
