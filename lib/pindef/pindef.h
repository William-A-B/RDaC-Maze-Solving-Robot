#ifndef PINDEF_H
#define PINDEF_H

#include <Arduino.h>
#include <mbed/mbed.h>

#define BAUDRATE 9600 // Baudrate for serial communication

#define MOTORSHIFT 0.5f // Motor calibration constant to correct speed of motors

#define MIN_IR_DIST 15.0f // Minimum distance for IR sensor to start detecting objects 
#define MIN_USONIC_DIST 12 // Minimum distance for USonic sensor to start detecting objects

#define MOTOR_LEFT_DIRECTION P0_4 // Left motor direction pin
#define MOTOR_RIGHT_DIRECTION P0_5 // Right motor direction pin

#define MOTOR_LEFT_PWM P0_27 // Left motor PWM pin
#define MOTOR_RIGHT_PWM P1_2 // Right motor PWM pin

#define MOTOR_LEFT_ENCODER P1_11 // Left motor encoder pin
#define MOTOR_LEFT_ENCODER_SECONDARY P0_29 // Left motor out of phase encoder pin (GPIO3)
#define MOTOR_RIGHT_ENCODER P1_12 // Right motor encoder pin
#define MOTOR_RIGHT_ENCODER_SECONDARY P0_28 // Right motor out of phase encoder pin (GPIO2)

#define IR_SENSOR_ADDRESS 0x80 // Address for IR sensor
#define IR_SENSOR_DISTANCE_REG 0x5E // Register address to read distance from
#define IR_SENSOR_SHIFT_REG 0x35 // Register address to read shift value from

#define USONIC_SENSOR_PIN_RIGHT P1_15 // Pin for right sensor
#define USONIC_SENSOR_PIN_LEFT P1_14 // Pin for left sensor

#define USONIC_LEFT D6 // Trigger for left sensor
#define USONIC_RIGHT D4 // Trigger for right sensor



#endif