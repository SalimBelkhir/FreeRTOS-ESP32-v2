# WELCOME TO FREERTOS ON ESP32 PROJECT

This project demonstrates how FreeRTOS works on the ESP32!  
This is your guide to getting the project up and running.

## Requirements

### Software
- ESP-IDF

### Hardware
- ESP32
- OLED SSD1307
- DHT22

## Getting Started

First, configure the project:
```bash
idf.py menuconfig
```

Navigate to **FreeRTOS Example Configuration** and choose which example you want to run:

1. Basic LED Blinking
2. Queue Communication
3. Binary Semaphore Communication
4. Mutex (Priority Inheritance)
5. Event Groups

## Build and Flash

Build the project:
```bash
idf.py build
```

Flash to your ESP32 and monitor the output:
```bash
idf.py flash monitor
```

Enjoy! 😊
