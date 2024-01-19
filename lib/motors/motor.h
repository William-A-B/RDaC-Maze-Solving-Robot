#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <mbed/mbed.h>
#include "pindef.h"

//#define MOTORSHIFT 0.515f // Motor calibration constant to correct speed of motors for a speed of 0.5f

// 0.48f - Move in straight line value
// #define MOTORSHIFT 0.4740f // Motor calibration constant to correct speed of motors for a speed on 0.75f

//#define MOTORSHIFT 0.529f
#define MOTORSHIFT 0.5460f // 0.545f

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

    enum motorDirection
	{
		DIR_FORWARDS,
		DIR_BACKWARDS,
		DIR_CLOCKWISE,
		DIR_ANTICLOCKWISE,
	} current_direction;

    void drive(int time_to_drive_forwards);
    void setDirection(motorDirection current_direction);
    void setup();
    void stopDriving();
    
    void setSpeed(float speed);
    float getSpeed();

    float getDistanceTravelledLeft();
    float getDistanceTravelledRight();

private:

    void countPulseLeft();
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
