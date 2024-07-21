# ESP32_BLE_WIFI_LORA
ESP32 Wireless Peripheral Modules Projects Examples

# Project1 (ESP32_BLE_C) :
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
4. **Monitor LEDs**: The LEDs will toggle between red, green, and blue, with the status sent to the connected BLE device

# Project2 (ESP32_BLE_MICROPYTHON):
# ESP32 TFT Display with SD Card and BLE

This project uses an ESP32 microcontroller to interface with a TFT display, an SD card, and Bluetooth Low Energy (BLE). The ESP32 displays images stored on the SD card on the TFT display and communicates with a BLE central device (e.g., a smartphone) to receive commands for displaying images or listing available images.

## Features

- **TFT Display**: Displays images from the SD card.
- **SD Card**: Stores image files (`.jpg` and `.png`).
- **Bluetooth Low Energy (BLE)**: Receives commands to display images or list available images.

## Hardware Requirements

- ESP32 development board
- ILI9341 TFT display
- SD card and SD card module
- Resistors (if necessary)
- Breadboard and jumper wires

## Software Requirements

- Arduino IDE or any suitable MicroPython environment
- `ili9341` library for the TFT display
- `sdcard` library for SD card handling
- `bluetooth` library for BLE functionality

## Circuit Diagram

Connect the components as follows:

- **TFT Display**:
  - CS to pin 15
  - DC to pin 2
  - SCK to pin 14
  - MOSI to pin 13

- **SD Card**:
  - CS to pin 5
  - SCK to pin 18
  - MOSI to pin 23
  - MISO to pin 19

## Code Description

### Importing Libraries and Pin Definitions

```python
import machine
from machine import Pin, SPI
from sdcard import SDCard
import os
import time
from ili9341 import ILI9341
import bluetooth
from micropython import const

# Pin definitions
TFT_CS = const(15)
TFT_DC = const(2)
SD_CS = const(5)
```

### Initializing TFT Display and SD Card

```python
# Initialize SPI for TFT display
spi = SPI(2, baudrate=20000000, sck=Pin(14), mosi=Pin(13))
display = ILI9341(spi, cs=Pin(TFT_CS), dc=Pin(TFT_DC), rst=Pin(-1))
spi2 = SPI(1, baudrate=20000000, sck=Pin(18), mosi=Pin(23), miso=Pin(19))

# Function to initialize SD card and mount the filesystem
def init_sd():
    try:
        sd = SDCard(spi2, Pin(SD_CS))
        vfs = os.VfsFat(sd)
        os.mount(vfs, "/sd")
        return vfs
    except Exception as e:
        print("Failed to initialize SD card:", e)
        return None

# Initialize SD card
sd = init_sd()
```

### Setting Up Bluetooth

```python
# Initialize Bluetooth
ble = bluetooth.BLE()
ble.active(True)

# Set BLE device name
ble.config(name='ESP32_cla_test0')

# BLE service and characteristic
SERVICE_UUID = bluetooth.UUID('0000180A-0000-1000-8000-00805F9B34FB')
CHARACTERISTIC_UUID = bluetooth.UUID('00002A7E-0000-1000-8000-00805F9B34FB')
command_char = (CHARACTERISTIC_UUID, bluetooth.FLAG_WRITE)
```

### Functions for Image Display and Listing

```python
# Function to display image from SD card on the TFT display
def display_image(image_path):
    try:
        with open(image_path, 'rb') as f:
            img_data = f.read()
            display.fill(ILI9341.BLACK)  # Clear the screen
            display.image(0, 0, img_data)
            display.show()
        return True
    except OSError as e:
        print("Failed to open file:", e)
        return False

# Function to list images on the SD card
def list_images():
    try:
        return [f for f in os.listdir('/sd') if f.endswith('.jpg') or f.endswith('.png')]
    except OSError as e:
        print("Failed to list files:", e)
        return []
```

### Handling BLE Commands

```python
# Callback for handling writes to the BLE characteristic
def on_command_received(event):
    value = event.data
    try:
        command = value.decode('utf-8')
        if command.startswith('DI:'):
            image_name = command.split(':')[1]
            image_path = f'/sd/{image_name}'
            if display_image(image_path):
                print(f"Image {image_name} displayed on screen.")
                ble.gatts_notify(0, command_char, b"OK: Image displayed")
            else:
                print(f"Failed to display image {image_name}.")
                ble.gatts_notify(0, command_char, b"ERROR: Failed to display image")
        elif command == 'LI':
            images = list_images()
            images_str = ','.join(images)
            ble.gatts_notify(0, command_char, images_str.encode('utf-8'))
    except Exception as e:
        print("Failed to process command:", e)
        ble.gatts_notify(0, command_char, b"ERROR: Failed to process command")

# Configure BLE service
ble.config(rxbuf=1024)
srv = ble.gatts_register_services(((SERVICE_UUID, (command_char,)),))
ble.gatts_set_buffer(command_char, 100)
ble.gatts_write(command_char, b'command')

# Function for BLE event interrupts
def ble_irq(event, data):
    if event == 2:  # Write event
        on_command_received(data)

ble.irq(handler=ble_irq)

print("Waiting for BLE connection...")
```

### Main Loop

```python
# Main loop
while True:
    if ble.connections():
        print("Client connected via BLE!")
        time.sleep(1)
    else:
        print("Waiting for connection...")
        time.sleep(1)
```

## Usage

1. **Upload the Code**: Use the Arduino IDE or a suitable MicroPython environment to upload the provided code to the ESP32.
2. **Power the ESP32**: Ensure the ESP32 is powered via USB or an external power source.
3. **Connect to BLE**: Use a BLE-compatible device (e.g., smartphone) to scan for and connect to the `ESP32_cla_test0`.
4. **Send Commands**:
   - **Display Image**: Send the command `DI:imagename` to display an image (`imagename` should be the name of an image file on the SD card).
   - **List Images**: Send the command `LI` to list all image files available on the SD card.

This project demonstrates the integration of an ESP32 with a TFT display, an SD card, and BLE communication, providing a foundation for advanced IoT applications.
