#include "bluetooth.h"

#define ROBOT_LOCATION_INFO_SERVICE_UUID "4b70a80a-c5dc-44a8-8246-c8b40808d405"
#define CLIENT_CONNECTION_SERVICE_UUID "1c18055d-48b3-40ab-8fda-f11194086e12"

#define BEARING_CHARACTERISTIC_UUID "2329585c-1c0c-4fb1-8f24-38880b2d650f"
#define X_COORD_CHARACTERISTIC_UUID "5206f37e-7860-49a0-9593-eb762f6b42ea"
#define Y_COORD_CHARACTERISTIC_UUID "18efaec4-47a1-4a6d-a1e7-ec2043a3a333"

#define BEARING_DESCRIPTOR_UUID "3de1fcd1-05f2-4b41-95d7-b4dea7d0afba"
#define X_COORD_DESCRIPTOR_UUID "a0f0d479-fe6a-425f-80c6-f70574575b16"
#define Y_COORD_DESCRIPTOR_UUID "804078f6-327d-4865-8d45-13b7ffb5aad0"

#define CLIENT_CONNECTION_CHARACTERISTIC_UUID "6fac34fe-e59e-4086-8491-127ee0139b13"

BLEDevice centralDevice;

BLEService ledService("180A");
BLEService robotLocationInfoService(ROBOT_LOCATION_INFO_SERVICE_UUID);
BLEService clientConnectedService(CLIENT_CONNECTION_SERVICE_UUID);

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
// BLEByteCharacteristic directionCharacteristic("1337", BLERead | BLEWrite);
BLEByteCharacteristic directionCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEIntCharacteristic bearingCharacteristic(BEARING_CHARACTERISTIC_UUID, BLERead | BLENotify);
BLEFloatCharacteristic xCoordinateCharacteristic(X_COORD_CHARACTERISTIC_UUID, BLERead | BLENotify);
BLEFloatCharacteristic yCoordinateCharacteristic(Y_COORD_CHARACTERISTIC_UUID, BLERead | BLENotify);

BLEDescriptor bearingDescriptor(BEARING_DESCRIPTOR_UUID, "Robot Bearing");
BLEDescriptor xCoordinateDescriptor(X_COORD_DESCRIPTOR_UUID, "Robot X Position");
BLEDescriptor yCoordinateDescriptor(Y_COORD_DESCRIPTOR_UUID, "Robot Y Position");

BLEByteCharacteristic clientConnectedCharacteristic(CLIENT_CONNECTION_CHARACTERISTIC_UUID, BLERead | BLEWrite);

// BLEDescriptor ledDescriptor("8888", "led");


bool Bluetooth::initialise_ble()
{
    // begin initialization
    if (!BLE.begin())
    {
        Serial.println("Could not start BLE!");

        return false;
    }

    // set advertised local name and service UUID:
    BLE.setLocalName(DEVICE_NAME);
    BLE.setAdvertisedService(ledService);

    // directionCharacteristic.addDescriptor(ledDescriptor);
    bearingCharacteristic.addDescriptor(bearingDescriptor);
    xCoordinateCharacteristic.addDescriptor(xCoordinateDescriptor);
    yCoordinateCharacteristic.addDescriptor(yCoordinateDescriptor);

    // add the characteristic to the service
    ledService.addCharacteristic(directionCharacteristic);

    robotLocationInfoService.addCharacteristic(bearingCharacteristic);
    robotLocationInfoService.addCharacteristic(xCoordinateCharacteristic);
    robotLocationInfoService.addCharacteristic(yCoordinateCharacteristic);

    clientConnectedService.addCharacteristic(clientConnectedCharacteristic);


    // add service
    BLE.addService(ledService);
    BLE.addService(robotLocationInfoService);
    BLE.addService(clientConnectedService);

    // set the initial value for the characteristic:
    directionCharacteristic.writeValue(0);
    bearingCharacteristic.writeValue(0);
    xCoordinateCharacteristic.writeValue(0.0f);
    yCoordinateCharacteristic.writeValue(0.0f);
    clientConnectedCharacteristic.writeValue(0x0);

    // start advertising
    BLE.advertise();

    Serial.println("Started BLE Robot");

    while (centralDevice == false)
    {
        // listen for BLE peripherals to connect:
        centralDevice = BLE.central();
        Serial.println("Searching for devices to connect");
        //wait_us(10);
        //this->pollBLE();
    }

    if (centralDevice == true)
    {
        return true;
    }
    else 
    {
        return false;
    }

}

void Bluetooth::listenForPeripherals()
{
    // listen for BLE peripherals to connect:
    centralDevice = BLE.central();
}

void Bluetooth::writeNumber(int number)
{
    directionCharacteristic.writeValue(number);
    directionCharacteristic.written();

}

void Bluetooth::updateRobotLocationInfo(int bearing, float xCoordinate, float yCoordinate)
{
    bearingCharacteristic.writeValue(bearing);
    xCoordinateCharacteristic.writeValue(xCoordinate);
    yCoordinateCharacteristic.writeValue(yCoordinate);
    this->pollBLE();
}

void Bluetooth::ledControl()
{
    // while the central is still connected to peripheral:
    while (centralDevice.connected()) {
        // if the remote device wrote to the characteristic,
        // use the value to control the LED:
        if (directionCharacteristic.written()) {
            switch (directionCharacteristic.value()) {   // any value other than 0
                case 1:
                    Serial.println("Forward");
                    digitalWrite(LED_BUILTIN, HIGH);            // will turn the LED on
                    break;
                case 2:
                    Serial.println("Left");
                    digitalWrite(LED_BUILTIN, HIGH);         // will turn the LED on
                    delay(500);
                    digitalWrite(LED_BUILTIN, LOW);         // will turn the LED off
                    delay(500);
                    digitalWrite(LED_BUILTIN, HIGH);      // will turn the LED on
                    delay(500);
                    digitalWrite(LED_BUILTIN, LOW);       // will turn the LED off
                    break;
                case 3:
                    Serial.println("Right");
                    digitalWrite(LED_BUILTIN, HIGH);         // will turn the LED on
                    delay(1000);
                    digitalWrite(LED_BUILTIN, LOW);         // will turn the LED off
                    delay(1000);
                    digitalWrite(LED_BUILTIN, HIGH);      // will turn the LED on
                    delay(1000);
                    digitalWrite(LED_BUILTIN, LOW);       // will turn the LED off
                    break;
                default:
                    Serial.println(F("Stop"));
                    digitalWrite(LED_BUILTIN, LOW);          // will turn the LED off
                    break;
            }
        }
    }
}

bool Bluetooth::isClientConnected()
{
    byte isClientConnected = 0x0;
    clientConnectedCharacteristic.readValue(isClientConnected);
    return (isClientConnected == 0x01);
}

bool Bluetooth::isConnected()
{
    return centralDevice.connected();
}

void Bluetooth::pollBLE()
{
    BLE.poll();
}


// BLEService motorControl();

// void Bluetooth::initialise_ble()
// {
//     // Initialise BLE
//     if(!BLE.begin()){
//         Serial.println("BLE init failed");
//     }else{
//         Serial.print("BLE address: ");
//         Serial.println(BLE.address()); // Print the address to serial so we know where to find it
//     }

//     // Set device Name
//     BLE.setDeviceName("Wills Robot");

//     // Set which service we are going to advertise
//     BLE.setAdvertisedService(motorControl);
// }



// BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// // Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
// BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// const int ledPin = LED_BUILTIN; // pin to use for the LED

// void setup() {
//   Serial.begin(9600);
//   while (!Serial);

//   // set LED pin to output mode
//   pinMode(ledPin, OUTPUT);

//   // begin initialization
//   if (!BLE.begin()) {
//     Serial.println("starting Bluetooth® Low Energy module failed!");

//     while (1);
//   }

//   // set advertised local name and service UUID:
//   BLE.setLocalName("LED");
//   BLE.setAdvertisedService(ledService);

//   // add the characteristic to the service
//   ledService.addCharacteristic(switchCharacteristic);

//   // add service
//   BLE.addService(ledService);

//   // set the initial value for the characeristic:
//   switchCharacteristic.writeValue(0);

//   // start advertising
//   BLE.advertise();

//   Serial.println("BLE LED Peripheral");
// }

// void loop() {
//   // listen for Bluetooth® Low Energy peripherals to connect:
//   BLEDevice central = BLE.central();

//   // if a central is connected to peripheral:
//   if (central) {
//     Serial.print("Connected to central: ");
//     // print the central's MAC address:
//     Serial.println(central.address());

//     // while the central is still connected to peripheral:
//     while (central.connected()) {
//       // if the remote device wrote to the characteristic,
//       // use the value to control the LED:
//       if (switchCharacteristic.written()) {
//         if (switchCharacteristic.value()) {   // any value other than 0
//           Serial.println("LED on");
//           digitalWrite(ledPin, HIGH);         // will turn the LED on
//         } else {                              // a 0 value
//           Serial.println(F("LED off"));
//           digitalWrite(ledPin, LOW);          // will turn the LED off
//         }
//       }
//     }

//     // when the central disconnects, print it out:
//     Serial.print(F("Disconnected from central: "));
//     Serial.println(central.address());
//   }
// }
