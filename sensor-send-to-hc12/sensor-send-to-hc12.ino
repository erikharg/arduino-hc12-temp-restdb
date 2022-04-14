/*
  Temperature sensor to RESTDB.io collection
  APC220 version (part 1):
  Temperature sensor via APC220 Long Range Radio serial communication to remote APC220
  By: Erik Harg <erik@harg.no>

*/
// libraries
#include <Adafruit_SleepyDog.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"
#include "wiring_private.h"
#include "TimeLib.h"

// Initialize Dallas Temperature sensors
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

#define HC12_SET_PIN 1

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Timekeeping
unsigned long sendData = 0;         // next time we'll send data
unsigned long SEND_WAIT = 1800;     // how long to wait between submissions -- 1800 = 30 min
unsigned long LOOP_WAIT_MS = 2000;  // how long to wait between loops -- 2000 ms = 2 sec
unsigned long lastLoopMillis = 0; // time of last loop execution

// APC220 Communications initialize variables
String readBuffer = "";

void setup()
{
    int countdownMS = Watchdog.enable(16000); // 16s is max timeout

    // initialize serial communications and wait for port to open:
    Serial.begin(9600);
    delay(5000);
    // Setup APC220
    Serial1.begin(9600);
  
    // Starting service
    Serial.println("\n");
    Serial.println("\n");
    Serial.println("\n");
    Serial.println("\nStarting service!");
    Serial.print("Enabled the watchdog with max countdown of ");
    Serial.print(countdownMS, DEC);
    Serial.println(" milliseconds!");
    sensors.begin();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
    unsigned long currentMillis = millis();
    if(currentMillis - lastLoopMillis >= LOOP_WAIT_MS)
    {
      Watchdog.reset();
      digitalWrite(LED_BUILTIN, HIGH);
  
      time_t ticktime = now();
  
      // we should send data
      if (ticktime > sendData)
      {
          Serial.print("Send data at ");
          Serial.print(formatDateTime(ticktime) + "\n");
          sendDataNow();
      }
      //Serial.println("Loop done");
  
      Watchdog.reset();
      digitalWrite(LED_BUILTIN, LOW);
      lastLoopMillis = millis(); // set the timing for the next loop
      Watchdog.reset();
    }
}

void sendDataNow()
{
    String tempString = "";
    String voltageString = "";

    float tempVal = getTemp();
    Watchdog.reset();
    Serial.println("Got temp: " + String(tempVal));
    
    if (tempVal != DEVICE_DISCONNECTED_C && tempVal > -127.0f)
    {
        tempString = tempAsString(tempVal);

        int sensorValue = analogRead(ADC_BATTERY);
        // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
        float voltage = sensorValue * (4.3 / 1023.0);
        voltageString = formatVoltageAsString(voltage);
        Serial.print("Sending temperature data:");
        String sendString = tempString + ";" + voltageString;
        Serial.println(sendString);
        Serial1.write(sendString.c_str()); // Send over APC220 (Serial1)
        Serial.println("Sent, waiting for reply...");

        Watchdog.reset();
        
        delay(500); // wait for APC220 to reply
    
        // read data back from APC220
        Watchdog.reset();
        time_t looptime = now() + (time_t)10;
        while(!Serial1.available() && now() < looptime) 
        {
          Serial.println("now:" + formatDateTime(now()) + ", looptime:" + formatDateTime(looptime));
          delay(500);
        }
        Watchdog.reset();
        readBuffer = "";
        if (Serial1.available())
        {
            Serial.println("Got UART data from APC220");
            Watchdog.reset();
            while (Serial1.available())
            {
                int msg = Serial1.read();
                readBuffer += (char)msg;
                //Serial.write(msg);
            }
            Watchdog.reset();
            Serial.print("Received:");
            Serial.print(readBuffer + "\n");
            if (readBuffer.startsWith("ACK"))
            {
                Serial.println("Got response message:");  
                Serial.println(readBuffer);
                Serial.println("End of message");
            } else {
                Serial.println("Got unsupported message:");
                Serial.println(readBuffer);
                Serial.println("End of message");
            }
            Watchdog.reset();
            sendData = now() + SEND_WAIT; // wait this long until we send data again
        } else {
          Serial.println("Could not receive response!");
        }
        Serial.println("Waiting until " + formatDateTime(sendData) + " to send data again");
    }
}

float getTemp()
{    
    sensors.requestTemperatures(); // Send the command to get temperatures
    return sensors.getTempCByIndex(0);
}

String tempAsString(float tempC)
{
    String tempString = String(tempC);
    char tempChars[6];
    tempString.toCharArray(tempChars, 5);
    return String(tempChars);
}

String formatVoltageAsString(float voltage)
{
    String tempString = String(voltage);
    char vChars[4];
    tempString.toCharArray(vChars, 4);
    return String(vChars);
}

String formatDateTime(time_t t)
{
    int y = year(t);
    int mn = month(t);
    int d = day(t);
    int h = hour(t);
    int mi = minute(t);
    int s = second(t);

    String y_s = String(y);
    String mn_s = mn > 9 ? String(mn) : ("0" + String(mn));
    String d_s = d > 9 ? String(d) : ("0" + String(d));
    String h_s = h > 9 ? String(h) : ("0" + String(h));
    String mi_s = mi > 9 ? String(mi) : ("0" + String(mi));
    String s_s = s > 9 ? String(s) : ("0" + String(s));

    String retval = y_s + "-" + mn_s + "-" + d_s + " " + h_s + ":" + mi_s + ":" + s_s + " UTC";

    return retval;
}
