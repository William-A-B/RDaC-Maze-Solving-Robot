#include "bluetooth.h"

BLEService dataControl("180A");

BLEDevice centralDevice;

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic directionCharacteristic("1337", BLERead | BLEWrite);


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
    BLE.setAdvertisedService(dataControl);

    // add the characteristic to the service
    dataControl.addCharacteristic(directionCharacteristic);

    // add service
    BLE.addService(dataControl);

    // set the initial value for the characteristic:
    directionCharacteristic.writeValue(0);

    // start advertising
    BLE.advertise();

    Serial.println("Started BLE Robot");

    while (centralDevice == false)
    {
        // listen for BLE peripherals to connect:
        centralDevice = BLE.central();
        Serial.println("Searching for devices to connect");
        wait_us(10);
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
