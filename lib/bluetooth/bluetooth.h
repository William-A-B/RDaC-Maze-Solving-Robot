#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <ArduinoBLE.h>
#include <Arduino.h>
#include "mbed/mbed.h"


#define readCharUUID "718dec93-f1bf-4ba1-9086-4263db01f061"
#define writeCharUUID "7863504f-9ee4-411a-afd4-a7270ecfefb1"
#define customServiceUUID "26a4efc8-2905-4d2f-a2fc-9193d9171368"


const static char DEVICE_NAME[] = "William's Robot";




class Bluetooth
{
public:


    bool initialise_ble();

    bool connectToClient();

    bool disconnectFromClient();

    bool isClientConnected();

    void updateRobotLocationInfo(int bearing, float xCoordinate, float yCoordinate);

    void disableSendingData();

    void enableSendingData();

    void pingService(byte value);

    void pollBLE();
    
private:


};

#endif
