#include "motor.h"

mbed::DigitalOut motorDirectionLeft(MOTOR_LEFT_DIRECTION);
mbed::DigitalOut motorDirectionRight(MOTOR_RIGHT_DIRECTION);

// A is left wheel
// B is right wheel
mbed::PwmOut motorPWMLeft(MOTOR_LEFT_PWM);
mbed::PwmOut motorPWMRight(MOTOR_RIGHT_PWM);

mbed::InterruptIn encoderLeft(MOTOR_LEFT_ENCODER);
//mbed::InterruptIn enc_A_left_secondary(MOTOR_LEFT_ENCODER_SECONDARY);
mbed::InterruptIn encoderRight(MOTOR_RIGHT_ENCODER);
//mbed::InterruptIn enc_B_right_secondary(MOTOR_RIGHT_ENCODER_SECONDARY);

mbed::DigitalIn encoderLeftIn(MOTOR_LEFT_ENCODER);
mbed::DigitalIn encoderLeftInSecond(MOTOR_LEFT_ENCODER_SECONDARY);
mbed::DigitalIn encoderRightIn(MOTOR_RIGHT_ENCODER);
mbed::DigitalIn encoderRightInSecond(MOTOR_RIGHT_ENCODER_SECONDARY);


void Motor::setup()
{
    // Set the period of the PWM
    motorPWMLeft.period_us(200);
    motorPWMRight.period_us(200);

    // Attach the interrupts for the encoders on the motors
    encoderLeft.rise(mbed::callback(this, &Motor::countPulseLeft));
    encoderRight.rise(mbed::callback(this, &Motor::countPulseRight));
}

/**
 * @param direction
 * 0 - forwards
 * 1 - backwards
 * 2 - clockwise circle
 * 3 - anticlockwise circle
 * */
void Motor::setDirection(motorDirection currentDirection)
{
    if (currentDirection == DIR_FORWARDS)
    {
        motorDirectionLeft = 0;
        motorDirectionRight = 1;
    }
    else if (currentDirection == DIR_BACKWARDS)
    {
        motorDirectionLeft = 1;
        motorDirectionRight = 0;
    }
    else if (currentDirection == DIR_CLOCKWISE)
    {
        motorDirectionLeft = 0;
        motorDirectionRight = 0;
    }
    else if (currentDirection == DIR_ANTICLOCKWISE)
    {
        motorDirectionLeft = 1;
        motorDirectionRight = 1;
    }
}

/**
 * @param timeToDriveForwards - How long to drive forwards for in ms
 * Sets the motors to drive in the direction set and at the speed set by the setSpeed function
 */
void Motor::drive(int timeToDriveForwards)
{
    if (timeToDriveForwards != 0)
    {
        motorPWMLeft.write(getSpeed() * MOTORSHIFT);
        motorPWMRight.write(getSpeed());
        wait_us(timeToDriveForwards * 1000);
        stopDriving();
    }
    else
    {
        motorPWMLeft.write(motorSpeed * MOTORSHIFT);
        motorPWMRight.write(motorSpeed);
    }
}

void Motor::stopDriving()
{
    motorPWMLeft.write(0.0f);
    motorPWMRight.write(0.0f);
}

void Motor::setSpeed(float speedToSet)
{
    motorSpeed = speedToSet;
    motorPWMLeft.write(speedToSet * MOTORSHIFT);
    motorPWMRight.write(speedToSet);
}

float Motor::getSpeed()
{
    return motorSpeed;
}

void Motor::countPulseLeft()
{
    if (encoderLeftIn != encoderLeftInSecond)
    {
        distanceTravelledLeft = distanceTravelledLeft + LEFT_ENCODER_DISTANCE;
    }
    else
    {
        distanceTravelledLeft = distanceTravelledLeft - LEFT_ENCODER_DISTANCE;
    }
    //distanceTravelledLeft = distanceTravelledLeft + 0.435;
    encoderCountLeft++;
}

void Motor::countPulseRight()
{
    if (encoderRightIn != encoderRightInSecond)
    {
        distanceTravelledRight = distanceTravelledRight - RIGHT_ENCODER_DISTANCE;
    }
    else
    {
        distanceTravelledRight = distanceTravelledRight + RIGHT_ENCODER_DISTANCE;
    }
    //distanceTravelledRight = distanceTravelledRight + 0.435;
    encoderCountRight++;
}

float Motor::getDistanceTravelledLeft()
{
    return distanceTravelledLeft;
}

float Motor::getDistanceTravelledRight()
{
    return distanceTravelledRight;
}


