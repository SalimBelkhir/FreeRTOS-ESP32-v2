# WELCOME TO FREERTOS ON ESP32 PROJECT 
this project demonstares how the FreeRTOS project is working under ESP32 !

This is your guide to get the project working 

The project require :
## Software 
 ESP-IDF

## Hardware
 ESP32
 OLED SSD1307
 DHT22

First of all tap :
```
idf.py menuconfig
```

To run the project you need to choice whoch part of the project you need to run under FreeRTOS Example Configuration
1- Basic led blinking
2- Queue Communication
3- Binary Semaphore communication
4- Mutex (Priority Inheritance)
5- Event Groups

then write the command : 
```
idf.py build
```
and finally

```
idf.py flash monitor
```

Enjoy ;)
