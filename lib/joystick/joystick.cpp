#include <Arduino.h>
#include <ARB.h>
#include "mbed/mbed.h"
#include "joystick.h"

mbed::InterruptIn joystickForwardsInterrupt(JOYSTICK_F);
mbed::InterruptIn joystickBackwardsInterrupt(JOYSTICK_B);

mbed::DigitalInOut joystickForwardsPin(JOYSTICK_F);
mbed::DigitalInOut joystickBackwardsPin(JOYSTICK_B);


void Joystick::joystickSetup() {

  // Setup ARB functionality
  ARBSetup(); 

  joystick.forwards = false;
  joystick.backwards = false;
  joystick.left = false;
  joystick.right = false;

  // Enable internal pull-ups on the pins
  joystickForwardsPin.write(HIGH);
  joystickBackwardsPin.write(HIGH);

  /* 
    Attach interrupts to PB pins
    The internal pull-ups pul the pins high when the button is open.
    When the button is closed it grounds the pin, so we want to be 
    looking for falling edges to see when the button has been pressed.
  */
  joystickForwardsInterrupt.fall(mbed::callback(this, &Joystick::joystickForwardsISR));
  joystickBackwardsInterrupt.fall(mbed::callback(this, &Joystick::joystickBackwardsISR));
}

/*
  This basic loop checks if a button press flag has been raised
  and if it has it prints out the name of that button and clears
  the flag
  Return directions corresponding to direction moved on Joystick
  0 - Forwards
  1 - Backwards
  2 - Left
  3 - Right
*/
int Joystick::checkJoystickPress() {
  if (joystick.forwards == true)
    return 0;
  else if (joystick.backwards == true)
    return 1;
  else if (joystick.left == true)
    return 2;
  else if (joystick.right == true)
    return 3;
  
  return -1;
}

/*
  It's best practice to keep ISRs as short as possible so these ISRs
  just set a flag if a button has been pressed so it can be acted upon
  outside of the ISR
*/
void Joystick::joystickForwardsISR()
{
  joystick.forwards = true;
  joystick.backwards = false;
  joystick.left = false;
  joystick.right = false;
}

void Joystick::joystickBackwardsISR()
{
  joystick.forwards = false;
  joystick.backwards = true;
  joystick.left = false;
  joystick.right = false;
}