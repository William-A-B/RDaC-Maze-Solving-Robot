#ifndef JOYSTICK_H
#define JOYSTICK_H


void DOWN_ISR();
void UP_ISR();
void RIGHT_ISR();
void LEFT_ISR();

void Joystick_setup();

void set_button_state(int state);

class Joystick
{
public:

    //Joystick();

    int check_button_press();

// private:
//     void DOWN_ISR();
//     void UP_ISR();
//     void RIGHT_ISR();
//     void LEFT_ISR();

};

#endif
