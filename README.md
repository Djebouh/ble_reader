## ble_reader

This PlatformIO ESP32 project implements:
- a BLE client that polls **_Xiaomi_ MiFlora** and **_Parrot_ Pot** plant sensors
- a wifi MQTT reporting of the measured conditions

## Technical requirements

Hardware:
- ESP32
- Xiaomi Mi Flora or Parrot Pot devices

Software:
- WIFI
- MQTT server

The project relies on PlatformIO and on the espidf framework. I use PlatformIO in VSCode but it should work as well with other PlatformIO integrations.
It is inspired from [sidddy's project](https://github.com/sidddy/flora).

## Setup instructions

1) clone the project

2) In the include folder, copy config.clean.h into **config.h** and update settings according to your MQTT server

3) In the root folder, copy platformio.clean.ini into **platformio.ini** and settings according to your WIFI SSID and password

4) Compile & upload on your ISP32 

## High Level execution details

1> BLE Scan to list all devices in range and publishing ServiceUUID 0xfe95 (Mi Flora) or 39e1f900-84a8-11e2-afba-0002a5d5c51b (Parrot)
2> If devices are found, connect to Wifi and MQTT Server (a first event is logged in the topic ESP)
3> Connect to each device found in (1) and extract the plant conditions (and the battery of the sensor). Once the data is read for a given sensor, it is reported to the MQTT server (topics are MiFlora or ParrotPot)
4> Close Wifi and MQTT connections
5> Sleep for SLEEP_DURATION and then restart from (1)

Each step has a max duration, configured as below.

## Configuration

In **config.h** 
// sleep between to runs in seconds
- SLEEP_DURATION - sleep between to runs in seconds
- ITO_SERVICE - max duration of the step (3) above
- ITO_APP - max duration of steps 1 to 4
- RETRY - number of attemps to connect and extra data for a given device
- MQTT_HOST - IP of the MQTT server
- MQTT_PORT - Port of the MQTT server
- MQTT_URI - URI to connect to the MQTT server (redundant to others, but I am lazy at the moment)

In **platformio.ini**, enter the SSID and Password of your wifi

## DRAFT MODE
BLE classed from kolban are simply copied in the project as I had memory management issues that I barely fixed. The project is quite stable at the moment, and has been running for 2 months at home, but it needs more housekeeping
