# ESP32_BLE_WIFI_LORA
ESP32 Wireless Peripheral Modules Projects Examples

#Project1 (ESP32_BLE_C) :
# ESP32 LED Controller with BLE

This project implements an LED controller using an ESP32 microcontroller with Bluetooth Low Energy (BLE). The controller manages three LEDs (red, green, and blue) and communicates with a central BLE device (e.g., a smartphone) to toggle the LEDs and send notifications about their status.

## Features

- **BLE Service and Characteristic**: The ESP32 acts as a BLE peripheral device, advertising a custom LED service and characteristic.
- **LED Control**: The ESP32 controls three LEDs connected to specific pins, toggling them based on commands received via BLE.
- **Notifications**: The ESP32 sends notifications to the connected central device, informing it of the current LED status.

## Hardware Requirements

- ESP32 development board
- Red, green, and blue LEDs
- Resistors (appropriate values for LEDs)
- Breadboard and jumper wires

## Software Requirements

- Arduino IDE
- `ArduinoBLE` library

## Circuit Diagram

Connect the LEDs to the ESP32 as follows:

- **Red LED**: Connect the anode to pin 4 via a resistor and the cathode to ground.
- **Green LED**: Connect the anode to pin 17 via a resistor and the cathode to ground.
- **Blue LED**: Connect the anode to pin 16 via a resistor and the cathode to ground.

## Code Description

### Includes and Pin Definitions

```cpp
#include <ArduinoBLE.h>

#define REDled 4      // Pin 4 for the red LED
#define GREENled 17   // Pin 17 for the green LED
#define BLUEled 16    // Pin 16 for the blue LED
```

### BLE Service and Characteristic Setup

```cpp
BLEService ledService("181A"); // LED service
BLEStringCharacteristic ledCharacteristic("2A56", BLERead | BLENotify, 20); // LED characteristic (max 20 characters)
```

### Setup Function

The `setup()` function initializes the serial communication, configures the LED pins as outputs, and initializes the BLE service and characteristic.

```cpp
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize LED pins as outputs
  pinMode(REDled, OUTPUT);
  pinMode(GREENled, OUTPUT);
  pinMode(BLUEled, OUTPUT);

  // Initialize all LEDs as off
  digitalWrite(REDled, HIGH);
  digitalWrite(GREENled, HIGH);
  digitalWrite(BLUEled, HIGH);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Configure BLE device
  BLE.setLocalName("ESP32_LED_Controller");
  BLE.setAdvertisedService(ledService);
  ledService.addCharacteristic(ledCharacteristic);
  BLE.addService(ledService);

  // Start BLE advertising
  BLE.advertise();
  Serial.println("Waiting for a client connection to notify...");
}
```

### Loop Function

The `loop()` function checks for BLE central device connections and handles LED toggling and notifications.

```cpp
void loop() {
  // Check if a BLE central device is connected
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("Client connected!");

    // Check if service is available
    if (central.connected() && central.discoverAttributes()) {
      Serial.println("Services discovered!");

      // Toggle LEDs and send information via BLE and Serial
      while (central.connected()) {
        // Logic to toggle LEDs and send information
        digitalWrite(REDled, LOW);
        digitalWrite(GREENled, HIGH);
        digitalWrite(BLUEled, HIGH);
        if (!sendInformation("RED LED on")) {
          break; // If sending fails, disconnect
        }
        delay(3000);

        // Logic to toggle LEDs and send information
        digitalWrite(REDled, HIGH);
        digitalWrite(GREENled, LOW);
        digitalWrite(BLUEled, HIGH);
        if (!sendInformation("GREEN LED on")) {
          break; // If sending fails, disconnect
        }
        delay(3000);

        // Logic to toggle LEDs and send information
        digitalWrite(REDled, HIGH);
        digitalWrite(GREENled, HIGH);
        digitalWrite(BLUEled, LOW);
        if (!sendInformation("BLUE LED on")) {
          break; // If sending fails, disconnect
        }
        delay(3000);
      }
    } else {
      Serial.println("Failed to discover services. Disconnecting...");
      BLE.disconnect();
    }

    Serial.println("Client disconnected!");
  }
}
```

### Helper Function

The `sendInformation()` function sends a string of information via BLE using the `ledCharacteristic`.

```cpp
bool sendInformation(const char* info) {
  Serial.print("Sending information via BLE: ");
  Serial.println(info);
  bool status = ledCharacteristic.writeValue(info);
  if (!status) {
    Serial.println("Failed to send information via BLE!");
  }
  return status;
}
```

## Usage

1. **Upload the Code**: Use the Arduino IDE to upload the provided code to the ESP32.
2. **Power the ESP32**: Ensure the ESP32 is powered via USB or an external power source.
3. **Connect to BLE**: Use a BLE-compatible device (e.g., smartphone) to scan for and connect to the `ESP32_LED_Controller`.
4. **Monitor LEDs**: The LEDs will toggle between red, green, and blue, with the status sent to the connected BLE device.

This project demonstrates the use of BLE to control and monitor an ESP32's peripherals, providing a foundation for more advanced IoT applications.
