#ifndef JOYSTICK_H
#define JOYSTICK_H

#define JOYSTICK_F P0_30
#define JOYSTICK_B P0_3


// Define struct to hold button push flags
struct JoystickState {
	bool left;
	bool right;
	bool forwards;
	bool backwards;
};

class Joystick
{
public:

    void joystickSetup();
    int checkJoystickPress();

private:
    
    JoystickState joystick;

    void joystickForwardsISR();
    void joystickBackwardsISR();

};

#endif
