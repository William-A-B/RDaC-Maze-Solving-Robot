#ifndef MOTOR_H
#define MOTOR_H

/**
 * @file motor.h
 * @author Y3905304
 * @brief Motor class header file, contains all the functions and variables for the motor class
 * 
 */

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"

// Motor calibration constant to correct speed of motors for a speed of 0.5f
#define MOTORSHIFT 0.550f

// Correct value to give equal encoder distances for the values of
// left = 0.046125f
// right = 0.01722f

#define LEFT_ENCODER_DISTANCE 0.046125f
#define RIGHT_ENCODER_DISTANCE 0.01694f

// Radius of the wheels - 48.5mm or 4.85cm
#define WHEEL_DIAMETER 48.5

class Motor
{
public:

    /** 
     * Constructor for the motor class, empty
    */
    Motor( ){};

    /**
     * @brief Enum to define the directions the motors and wheels should move in
     * 
     */
    enum motorDirection
	{
		DIR_FORWARDS,
		DIR_BACKWARDS,
		DIR_CLOCKWISE,
		DIR_ANTICLOCKWISE,
	} current_direction;

    /**
     * @brief Sets the motors to drive forwards at the current set speed
     * 
     * @param time_to_drive_forwards  - if the robot should drive forwards for a set time
     * If equal to 0, drives forwards until told to stop
     */
    void drive(int time_to_drive_forwards);

    /**
     * @brief Set the direction of the motors
     * 
     * @param current_direction the direction to set the current direction to
     * 0 - forwards
     * 1 - backwards
     * 2 - clockwise circle
     * 3 - anticlockwise circle
     */
    void setDirection(motorDirection current_direction);

    /**
     * @brief Sets up the motors defining the PWM frequency
     * Attaches the interrupts too
     */
    void setup();

    /**
     * @brief Stops the motors from moving completely
     * 
     */
    void stopDriving();
    
    /**
     * @brief Sets the speed of the motors
     * 
     * @param speed the speed to set the motors to
     */
    void setSpeed(float speed);

    /**
     * @brief Gets the speed of the motors
     * 
     * @return float the speed of the motors
     */
    float getSpeed();

    /**
     * @brief Gets the distance travelled by the left wheel
     * 
     * @return float the distance travelled by the left wheel
     */
    float getDistanceTravelledLeft();

    /**
     * @brief Get the distance travelled by the right wheel
     * 
     * @return float the distance travelled by the right wheel
     */
    float getDistanceTravelledRight();

private:

    /**
     * @brief Interrupt service routine to count the pulses for the left encoder
     */
    void countPulseLeft();

    /**
     * @brief Interrupt service routine to count the pulses for the right encoder
     */
    void countPulseRight();

    float motorSpeed;

    // Encoder counter for left motor
    long int encoderCountLeft = 0;

    // Encoder counter for right motor
    long int encoderCountRight = 0;

    // The absolute distance travelled based on the left motor encoder
    // In mm
    // Per 1000 on right wheel - 46cm length
    // 0.46mm per 1 encoder count
    // 1000 46.25cm = 0.4625mm
    // 0.46125 per 1 encoder count avg
    // 0.046125 cm
    float distanceTravelledLeft = 0;

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
    float distanceTravelledRight = 0;

};

#endif
