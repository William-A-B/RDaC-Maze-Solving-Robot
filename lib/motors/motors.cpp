#include "motors.h"

mbed::DigitalOut motor_dir_A_left(MOTOR_LEFT_DIRECTION);
mbed::DigitalOut motor_dir_B_right(MOTOR_RIGHT_DIRECTION);

// A is left wheel
// B is right wheel
mbed::PwmOut motor_left_PWM(MOTOR_LEFT_PWM);
mbed::PwmOut motor_right_PWM(MOTOR_RIGHT_PWM);

mbed::InterruptIn enc_A_left(MOTOR_LEFT_ENCODER);
//mbed::InterruptIn enc_A_left_secondary(MOTOR_LEFT_ENCODER_SECONDARY);
mbed::InterruptIn enc_B_right(MOTOR_RIGHT_ENCODER);
//mbed::InterruptIn enc_B_right_secondary(MOTOR_RIGHT_ENCODER_SECONDARY);

mbed::DigitalIn encoder_left_in(MOTOR_LEFT_ENCODER);
mbed::DigitalIn encoder_left_in_secondary(MOTOR_LEFT_ENCODER_SECONDARY);
mbed::DigitalIn encoder_right_in(MOTOR_RIGHT_ENCODER);
mbed::DigitalIn encoder_right_in_secondary(MOTOR_RIGHT_ENCODER_SECONDARY);


// Shaft and encoder counters for left motor
long int shaft_rev_A = 0;
long int enc_count_A = 0;

// Shaft and encoder counters for right motor
long int shaft_rev_B = 0;
long int enc_count_B = 0;




float distance_travelled_left = 0;

// The absolute distance travelled based on the right motor encoder
// In mm
// Per 1000 on right wheel - 43.5cm length
// 0.435mm per 1 encoder revolution
float distance_travelled_right = 0;

// Motors aMotor;



void attach_encoder_interrupts()
{
    enc_A_left.rise(&count_pulse_A);
    enc_B_right.rise(&count_pulse_B);
    //enc_A_left.rise(&aMotor.test);
}

// void Motors::test()
// {
//     int i = 0;
//     i++;
// }

/**
 * @param direction
 * 0 - forwards
 * 1 - backwards
 * 2 - clockwise circle
 * 3 - anticlockwise circle
 * */
void Motors::set_direction(motor_direction current_direction)
{
    if (current_direction == DIR_FORWARDS)
    {
        motor_dir_A_left = 0;
        motor_dir_B_right = 1;
    }
    else if (current_direction == DIR_BACKWARDS)
    {
        motor_dir_A_left = 1;
        motor_dir_B_right = 0;
    }
    else if (current_direction == DIR_CLOCKWISE)
    {
        motor_dir_A_left = 0;
        motor_dir_B_right = 0;
    }
    else if (current_direction == DIR_ANTICLOCKWISE)
    {
        motor_dir_A_left = 1;
        motor_dir_B_right = 1;
    }
}

void Motors::calibrate()
{
    motor_left_PWM.period_us(200);
    motor_right_PWM.period_us(200);
}

/**
 * @param time_to_drive_forwards - How long to drive forwards for in ms
 * Sets the motors to drive in the direction set and at the speed set by the set_speed function
 */
void Motors::drive_forwards(int time_to_drive_forwards)
{
    // if (motor_speed == 0)
    // {
    //     throw (MotorSpeedInvalid);
    // }
    if (time_to_drive_forwards != 0)
    {
        motor_left_PWM.write(get_speed() * MOTORSHIFT);
        motor_right_PWM.write(get_speed());
        wait_us(time_to_drive_forwards * 1000);
        stop_driving();
    }
    else
    {
        motor_left_PWM.write(motor_speed * MOTORSHIFT);
        motor_right_PWM.write(motor_speed);
    }
}

void Motors::stop_driving()
{
    motor_left_PWM.write(0.0f);
    motor_right_PWM.write(0.0f);
}

void Motors::turn_left(int angle)
{
    stop_driving();

    mbed::Timer timer;
    timer.start();
    while (timer.elapsed_time().count() < 2.25 * 1000000)
    {
        set_direction(DIR_ANTICLOCKWISE);
        drive_forwards(0);
    }
    timer.stop();
    set_direction(DIR_FORWARDS);

    // long int current_shaft = shaft_rev_A;
    // while (shaft_rev_A < current_shaft + 150)
    // {
    //     motor_left_PWM.write(0.5*MOTORSHIFT);
    // }
}

void Motors::turn_right(int angle)
{
    stop_driving();

    long int current_shaft = shaft_rev_B;
    while (shaft_rev_B < current_shaft + 150)
    {
        motor_right_PWM.write(0.5);
    }
    stop_driving();
}

void Motors::set_speed(float speed_to_set)
{
    // float pwm_current = motor_speed;
    // float pwm_to_set = speed_to_set;

    motor_speed = speed_to_set;
    motor_left_PWM.write(speed_to_set * MOTORSHIFT);
    motor_right_PWM.write(speed_to_set);

    // if (pwm_current < pwm_to_set)
    // {
    //     pwm_current = pwm_current + 0.1f;
    //     motor_left_PWM.write(pwm_current);
    //     motor_right_PWM.write(pwm_current);
    // }

    // if (pwm_current > pwm_to_set)
    // {
    //     pwm_current = pwm_current - 0.1f;
    //     motor_left_PWM.write(pwm_current);
    //     motor_right_PWM.write(pwm_current);
    // }
}

float Motors::get_speed()
{
    return motor_speed;
}

void count_pulse_A()
{
    if (encoder_left_in != encoder_left_in_secondary)
    {
        distance_travelled_left = distance_travelled_left + 0.435;
    }
    else
    {
        distance_travelled_left = distance_travelled_left - 0.435;
    }
    //distance_travelled_left = distance_travelled_left + 0.435;
    enc_count_A++;
}

void count_pulse_B()
{
    if (encoder_right_in != encoder_right_in_secondary)
    {
        distance_travelled_right = distance_travelled_right - 0.435;
    }
    else
    {
        distance_travelled_right = distance_travelled_right + 0.435;
    }
    //distance_travelled_right = distance_travelled_right + 0.435;
    enc_count_B++;
}

// void count_pulse_B()
// {
//     if(enc_count_B%12 == 0)
//     {
//         if(enc_count_B%150 == 0)
//         {
//             shaft_rev_B++;
//         }
//     }
//     enc_count_B++;
// }

long int get_shaft_revolutions_left()
{
    return shaft_rev_A;
}

long int get_shaft_revolutions_right()
{
    return shaft_rev_B;
}

long int get_encoder_revolutions_left()
{
    return enc_count_A;
}

long int get_encoder_revolutions_right()
{
    return enc_count_B;
}

void set_distance_travelled_left(float distance_to_set)
{
    distance_travelled_left = distance_to_set;
}

void increment_distance_travelled_left(float distance_to_increase)
{
    distance_travelled_left = distance_travelled_left + distance_to_increase;
}

float get_distance_travelled_left()
{
    return distance_travelled_left;
}

void set_distance_travelled_right(float distance_to_set)
{
    distance_travelled_right = distance_to_set;
}

void increment_distance_travelled_right(float distance_to_increase)
{
    distance_travelled_right = distance_travelled_right + distance_to_increase;
}

float get_distance_travelled_right()
{
    return distance_travelled_right;
}


