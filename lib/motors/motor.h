#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"

//#define MOTORSHIFT 0.515f // Motor calibration constant to correct speed of motors for a speed of 0.5f

// 0.48f - Move in straight line value
// #define MOTORSHIFT 0.4740f // Motor calibration constant to correct speed of motors for a speed on 0.75f
#define MOTORSHIFT 0.529f

// Correct value to give equal encoder distances for the values of
// left = 0.046125f
// right = 0.01722f
//#define MOTORSHIFT 0.4400030f 

#define LEFT_ENCODER_DISTANCE 0.046125f
// #define LEFT_ENCODER_DISTANCE 0.047500f
#define RIGHT_ENCODER_DISTANCE 0.01694f



// Radius of the wheels - 48.5mm or 4.85cm
#define WHEEL_DIAMETER 48.5

class Motor
{
public:

    Motor( ){};

    enum motor_direction
	{
		DIR_FORWARDS,
		DIR_BACKWARDS,
		DIR_CLOCKWISE,
		DIR_ANTICLOCKWISE,
	} current_direction;

    void drive(int time_to_drive_forwards);
    void set_direction(motor_direction current_direction);
    void attach_encoder_interrupts();
    void setup();
    void stop_driving();
    void turn_left(int angle);
    void turn_right(int angle);
    
    void set_speed(float speed);
    float get_speed();

    float calculate_distance_by_wheel_rotations_left();
    float calculate_distance_by_wheel_rotations_right();

    int get_wheel_rotations_left();
    int get_wheel_rotations_right();

    long int get_shaft_revolutions_left();
    long int get_shaft_revolutions_right();
    long int get_encoder_revolutions_left();
    long int get_encoder_revolutions_right();

    void set_distance_travelled_left(float distance_to_set);
    void increment_distance_travelled_left(float distance_to_increase);
    void set_distance_travelled_right(float distance_to_set);
    void increment_distance_travelled_right(float distance_to_increase);
    float get_distance_travelled_left();
    float get_distance_travelled_right();



    //void count_pulse_left();
    //void count_pulse_right();

private:

    void count_pulse_left();
    void count_pulse_right();

	int pinNumber;

    float motor_speed;

    
    // Shaft and encoder counters for left motor
    long int shaft_rev_A = 0;
    long int enc_count_A = 0;

    // Shaft and encoder counters for right motor
    long int shaft_rev_B = 0;
    long int enc_count_B = 0;

    // The absolute distance travelled based on the left motor encoder
    // In mm
    // Per 1000 on right wheel - 46cm length
    // 0.46mm per 1 encoder count
    // 1000 46.25cm = 0.4625mm
    // 0.46125 per 1 encoder count avg
    // 0.046125 cm
    float distance_travelled_left = 0;

    // The absolute distance travelled based on the right motor encoder
    // In mm
    // Per 1000 on right wheel - 43.5cm length
    // 0.435mm per 1 encoder revolution
    // 1013 and 17.8cm
    // 1008 17.2cm = 0.1706
    // 1007 17.5cm = 0.1738
    // 1002 17.25cm = 0.1722
    // 0.1722mm per 1 encoder count avg
    // 0.01722 cm
    float distance_travelled_right = 0;

    // 329 encoder counts for one left wheel rotation
    int wheel_rotations_left = 0;
    // 865 encoder counts for one right wheel rotation
    int wheel_rotations_right = 0;


};

#endif
