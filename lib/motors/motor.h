#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"

//#define MOTORSHIFT 0.515f // Motor calibration constant to correct speed of motors for a speed of 0.5f

#define MOTORSHIFT 0.48f // Motor calibration constant to correct speed of motors for a speed on 0.75f

void count_pulse_A();
void count_pulse_B();
void attach_encoder_interrupts();
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

    void drive_forwards(int time_to_drive_forwards);
    void set_direction(motor_direction current_direction);
    void calibrate();
    void stop_driving();
    void turn_left(int angle);
    void turn_right(int angle);
    
    void set_speed(float speed);
    float get_speed();

    void test();



    



    //void count_pulse_A();
    //void count_pulse_B();

private:
	int pinNumber;
    // // Shaft and encoder counters for left motor
    // long int ShaftRevA;
    // long int EncCountA;

    // // Shaft and encoder counters for right motor
    // long int ShaftRevB;
    // long int EncCountB;
    float motor_speed;

    


    // // Shaft and encoder counters for left motor
    // long int shaft_rev_A;
    // long int enc_count_A;

    // // Shaft and encoder counters for right motor
    // long int shaft_rev_B;
    // long int enc_count_B;


    

};

#endif
