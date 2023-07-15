/*
  Temperature sensor to RESTDB.io collection
  E32-868T30D version (part 1):
  Temperature sensor via E32-868T30D LoRa radio serial communication to remote E32-868T30D
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

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Timekeeping
unsigned long sendData = 0;         // next time we'll send data
unsigned long SEND_WAIT = 3600;     // how long to wait between submissions -- 1800 = 30 min
unsigned long LOOP_WAIT_MS = 10000;  // how long to wait between loops -- 2000 ms = 2 sec
unsigned long lastLoopMillis = 0; // time of last loop execution

// Radio Communications initialize variables
unsigned radioWorkingMode = 0;
unsigned radioWakeupMode = 1;
unsigned radioPowerSavingMode = 2;
unsigned radioConfigMode = 3;
int mZeroPin = 3;
int mOnePin = 4;
int auxPin = 5;

byte readParams = 0xC1;
uint8_t cmd[3] = {readParams, readParams, readParams};

String readBuffer = "";

void setup()
{
    int countdownMS = Watchdog.enable(16000); // 16s is max timeout

    // initialize serial communications and wait for port to open:
    Serial.begin(9600);
    Serial.print("Enabled the watchdog with max countdown of ");
    Serial.print(countdownMS, DEC);
    Serial.println(" milliseconds!");
    
    delay(5000);
    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(mZeroPin, OUTPUT);
    pinMode(mOnePin, OUTPUT);
    pinMode(auxPin, INPUT);
    
    // Setup Radio
    Serial.println("Starting radio");
    Serial1.begin(9600, SERIAL_8N1);
    int radioReady = digitalRead(auxPin);
    Watchdog.reset();
    while(radioReady == LOW) {
      Serial.print(".");
      delay(100);
      radioReady = digitalRead(auxPin);
    }
    Serial.println("...");
    Watchdog.reset();
    setRadioMode(radioConfigMode);
    Serial.println("Getting radio config...");
    Serial1.write(cmd, 3);
    delay(100);
    
    if(Serial1.available())
    {
      int i = 0;
      while (Serial1.available())
      {
        int msg = Serial1.read();
        Serial.println("Got:");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(msg, HEX);
        Serial.print("\n");
        i++;
      }
    }
    setRadioMode(radioPowerSavingMode);
    Watchdog.reset();
    
    Serial.println("\nStarting service!");
    
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
        setRadioMode(radioWakeupMode);
        tempString = tempAsString(tempVal);

        int sensorValue = analogRead(ADC_BATTERY);
        // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
        float voltage = sensorValue * (4.3 / 1023.0);
        voltageString = formatVoltageAsString(voltage);
        Serial.print("Sending temperature data:");
        String sendString = tempString + ";" + voltageString;
        Serial.println(sendString);
        Serial1.write(sendString.c_str()); // Send over Radio (Serial1)
        Serial1.flush();
        
        Serial.println("Sent, waiting for reply...");

        Watchdog.reset();

        int radioReady  = digitalRead(auxPin);
        time_t looptime = now() + (time_t)10; // max 10s wait
        while(radioReady == LOW && now() < looptime) {
          Serial.print(".");
          delay(50);
          radioReady = digitalRead(auxPin);
        }
        //delay(250); // wait for Remote Radio to reply
        setRadioMode(radioPowerSavingMode);
    
        // read data back from Radio
        Watchdog.reset();
        looptime = now() + (time_t)10; // max 10s wait
        while(!Serial1.available() && now() < looptime) 
        {
          Serial.println("now:" + formatDateTime(now()) + ", looptime:" + formatDateTime(looptime));
          delay(500);
        }
        Watchdog.reset();
        readBuffer = "";
        if (Serial1.available())
        {
            Serial.println("Got UART data from Radio");
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
                sendData = now() + SEND_WAIT; // wait this long until we send data again
            } else {
                Serial.println("Got unsupported message:");
                Serial.println(readBuffer);
                Serial.println("End of message");
            }
            Watchdog.reset();            
        } else {
          Serial.println("Could not receive response!");
        }
        setRadioMode(radioPowerSavingMode);
        Serial.println("Waiting until " + formatDateTime(sendData) + " to send data again");
    }
}

void setRadioMode(unsigned radioMode)
{
  Serial.print("Setting radio mode M0=");
  Serial.print(getModeZero(radioMode) ? HIGH : LOW);
  Serial.print(", M1=");
  Serial.print(getModeOne(radioMode) ? HIGH : LOW);
  Serial.print("\n");
  digitalWrite(mZeroPin, getModeZero(radioMode) ? HIGH : LOW);
  digitalWrite(mOnePin,  getModeOne(radioMode)  ? HIGH : LOW);
  delay(500);
}

bool getModeZero(unsigned radioMode)
{
  return (bool)((radioMode >> 0) & 1);
}

bool getModeOne(unsigned radioMode)
{
  return (bool)((radioMode >> 1) & 1);
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
