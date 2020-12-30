# Arduino Temperature Sensor to RESTDB.io
Arduino project to monitor temperature with a DS18B20 probe and send the results to a RESTDB.io collection using HC-12 long-range radio

## Goal
The goal was to use the waterproof temperature sensor to measure sea water temperature and report this to a permanent record in a robust fashion, to be used for a simple web service and dashboard.

This version attempts to bypass the lack of GSM connectivity at the sensor location, by relaying measurements via HC-12 to a secondary Arduino that is Wifi connected to the internet.

## Hardware

* Sensor end:
  * Arduino MKR1000
    * HC-12 module
    * 100uF capacitor
* Send to cloud end:
  * Arduino MKR Wifi1010
    * HC-12 module
    * DS18B20 waterproof temperature probe
    * One 4.7kOhm resistor
    * 100uF capacitor
* Each end:
  * Breadboard or other circuit
  * Jump wires
  * Power source

### Set up installation

1. Solder pins on HC-12 modules, connect to breadboard
1. Connect antenna to HC-12 modules
1. Connect VCC, GND, PIN 14 (TX), PIN 13 (RX), PIN 1 of Arduino MKR board to (respectively) VCC, GND, RXD, TXD, SET of HC-12 
1. On the sensor end
    1. Connect GND and VCC to Gnd (black) and Vcc (red) of DS18B20
    1. Connect PIN2 to the signal wire (yellow) of DS18B20
    1. Connect the resistor between signal and Vcc of DS18B20
1. Set up code (below)
1. Load the software onto board
    1. `sensor-send-to-hc12.ino` on the sensor end
    1. `rcv-from-hc12-fwd-to-restdb.ino` on the wifi-enabled end
1. Set up permanent power source and make the installation rugged enough for outdoor use...

## Setup code

1. Set up your project on RESTDB.io, make a note of your API key and project name/server name
  1. Code expects RESTDB collection to be called "temperatures"
1. Install libraries (see below)
1. Define Wifi SSID and Password, RESTDB.io API key and RESTDB.io server name (project name) in secrets.h (in `rcv-from-hc12-fwd-to-restdb/` folder)
1. Adjust timeout `SEND_WAIT` as needed in either sketch
1. Adjust collection name in `client.post("/rest/temperatures");` if needed

### Code modifications
* You may need to change the `Serial1` pin assignments if you're using a board with a different serial port hardware.

### Required libraries
* Adafruit_SleepyDog_Library
* ArduinoHttpClient
* Arduino_JSON
* DallasTemperature
* OneWire
* SPI
* Time
* WiFi101
* WiFiUDP

### Library modifications
* You may need to move `libraries/Time/Time.h` to another file name to not conflict with `<time.h>`. I renamed it to `_Time.h` and included `"TimeLib.h"` in my project instead of `<Time.h>`, to get it to compile on my Mac.
* You may want to change the value of `kHttpWaitForDataDelay` set in `libraries/HttpClient/HttpClient.h` from `1000` to `30` (should be more than 20). This seems to improve robustness on bad GSM networks.
