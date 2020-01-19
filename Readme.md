This repository contains the firmware for smart home sensors.

## RoomClimate
The room climate sensor meassures temperature and humidity and sends the values to an OpenHab REST interface. The items are expected
to have the names `temperatureXXXXXXXXX` and `humidityXXXXXXXX` where the `XXXXXXXXX` are substituted by the MAC address of the devices.

A suited device (and the only one I tested it with is):
ESP8266 NodeMCU 12-F and a BME280 breakout board.

Firmware features:
- low energy consumption by using deep sleep
- reusing DHCP details to reduce time the device is awake
- reusing WIFI BSSID and channel information
- sending data to OpenHab REST interface
- using MAC address as identifier for OpenHab items to avoid individual firmware per sensor

Instruction on how to map the cryptic MAC address items to acutally readable OpenHab items can be read in this blog post:
[https://blog.papau.org/2020/01/19/temperature-sensors-openhab.html]
