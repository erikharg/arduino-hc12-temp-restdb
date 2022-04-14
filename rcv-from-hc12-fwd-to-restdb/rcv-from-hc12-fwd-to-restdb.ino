/*
  Temperature sensor to RESTDB.io collection
  APC220 version (part 2):
  Receive measurements from APC220 Long Range Radio serial communication, send to RESTDB.io via Wifi/internet
  By: Erik Harg <erik@harg.no>

*/

// libraries

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <ArduinoHttpClient.h>
#include <Arduino_JSON.h>
#include "TimeLib.h"
#include <Adafruit_SleepyDog.h>

#include "secrets.h"
// Please enter your sensitive data in secrets.h
// An example is provided in the repo (secrets_example.h)

// Initialize Wifi system variables
String ssid = String(SECRET_SSID);   // your network SSID (name)
String pass = String(SECRET_PASS);   // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS; // the Wifi radio's status
WiFiSSLClient wificlient;

// Initialize HTTP client with URL, port and path
String serverAddress = String(SERVER_ADDRESS);
int port = 443;
HttpClient client = HttpClient(wificlient, serverAddress, port);

// Variable for keeping the next values to send
JSONVar tempValues;
double lastTemp = 0.0;

// Variable for saving obtained response
String response = "";

// Messages for serial monitor response
String oktext = "OK";
String errortext = "ERROR";

// APC220 Communications initialize variables
String readBuffer = "";
int setPin = 1;

// Timekeeping
unsigned int localNTPPort = 2359;
IPAddress timeServer(216, 239, 35, 0); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48;        // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];    //buffer to hold incoming and outgoing packets
WiFiUDP Udp;                           // A UDP instance to let us send and receive packets over UDP

unsigned long sendData = 0;     // next time we'll send data
unsigned long SEND_WAIT = 3600; // how long to wait between submissions -- 3600 = 1h
unsigned long LOOP_WAIT_MS = 5000;  // how long to wait between loops -- 5000 ms = 5 sec
unsigned long lastLoopMillis = 0; // time of last loop execution

void setup()
{
    int countdownMS = Watchdog.enable(16000); // 16s is max timeout

    // initialize serial communications and wait for port to open:
    Serial.begin(9600);
    delay(1000);
    
    // Setup APC220
    Serial1.begin(9600);

    Serial.println("\n");
    Serial.println("\n");
    Serial.println("\n");
    Serial.println("\nStarting service!");
    Serial.print("Enabled the watchdog with max countdown of ");
    Serial.print(countdownMS, DEC);
    Serial.println(" milliseconds!");

    Serial.println("Testing connection with Wifi");
    Watchdog.reset();
    connectToWiFi();

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
  
      // Get time if current time is unrealistic
      if (ticktime < 10000) 
      {
          updateTimeKeeper();
          ticktime = now(); // use the newly set time as it is more accurate
      }
  
      // Get data from APC220
      Watchdog.reset();
      if(Serial1.available())
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
  
          int sampleNo = tempValues.length();
          if(sampleNo < 0) {
            sampleNo = 0;
          }
          Serial.print("Gathered sample (#" + String(sampleNo) + ") at ");
          Serial.print(formatDateTime(ticktime) + "\n");
          Serial.println("Sending data at " + formatDateTime(sendData));
          printWifiStatus();
          Watchdog.reset();
          JSONVar sample;
          char buf[sizeof(readBuffer)];
          readBuffer.toCharArray(buf, sizeof(buf));
          char *p = buf;
          char *str;
          int i = 0;
          bool validSample = false;
          double thisTemp = 0.0;
          Watchdog.reset();
          while ((str = strtok_r(p, ";", &p)) != NULL) // delimiter is the semicolon
          {
            char *end;
            double val = strtod(str, &end);
            if(str != end) {
              if(i == 0) 
              {
                sample["temperature"] = String(val);
                validSample = true;
                thisTemp = val;
              } else if (i == 1) 
              {
                sample["voltage"] = String(val);
              } else 
              {
                Serial.println("Extra data received [" + String(i) + "]: " + String(str));
              }  
            } else {
              Serial.println("Invalid value received [" + String(i) + "]: " + String(str));
            }
            
            i++;
          }
          if(sampleNo > 0 && validSample) {
            if(abs(thisTemp-lastTemp) > 10.0) {
              // Temp diff is too large, likely a measurement or transmission error
              validSample = false;
            } else {
              lastTemp = thisTemp;
            }
          }
          // If we got a valid temperature, and it survived the diff check above, store it...
          if(validSample) {
            sample["time"] = formatDateTime(ticktime);
            tempValues[sampleNo] = sample;
          }
          readBuffer = "";
          Watchdog.reset();
          Serial1.write("ACK");
      }
  
      if (ticktime > sendData && tempValues.length() > 0)
      {
          // check for Wifi
          Serial.print("Checking Wifi network at ");
          Serial.print(formatDateTime(ticktime) + "...\n");
          Watchdog.reset();
          connectToWiFi();
  
          Serial.print("Making HTTP POST request with ");
          Serial.print(tempValues.length());
          Serial.print(" samples:\n");
          String contentType = "application/json";
          String postData = JSON.stringify(tempValues);
          
          Serial.println("POST Data:");
          Serial.println(postData);
          Serial.println("End of POST Data");
  
          Watchdog.reset();
          client.beginRequest();
          Watchdog.reset();
          Serial.print(".");
          int httpretval = client.post("/rest/temperatures");
          Watchdog.reset();
          Serial.println("Started request: " + String(httpretval));
          Serial.print(".");
          client.sendHeader(HTTP_HEADER_CONTENT_TYPE, contentType);
          Watchdog.reset();
          Serial.print(".");
          client.sendHeader(HTTP_HEADER_CONTENT_LENGTH, postData.length());
          Watchdog.reset();
          Serial.print(".");
          client.sendHeader("x-apikey", X_API_KEY);
          Watchdog.reset();
          Serial.print(".");
          client.beginBody();
          Watchdog.reset();
          Serial.print(".");
          int bytesWritten = client.print(postData);
          Watchdog.reset();
          Serial.println("Wrote " + String(bytesWritten) + " bytes");
          Serial.print(".");
          client.endRequest();
          Watchdog.reset();
          Serial.print("OK\n");
  
          // read the status code and body of the response
          Serial.println("Getting response");
          // TODO: Fix timeout issue to re-enable Watchdog here
          //Watchdog.disable();
          
          int statusCode = client.responseStatusCode();
          Serial.print("Status code: ");
          Serial.println(statusCode);
          
          Watchdog.reset();
          String response = client.responseBody();
          Serial.print("Response: ");
          Serial.println(response);
          
          //Watchdog.enable(16000);
          
          if (statusCode >= 200 && statusCode < 300)
          {
              tempValues = JSONVar(); // empty the value array
              sendData = now() + SEND_WAIT; // wait this long until we send data again
          }
          Watchdog.reset();
  
          Serial.println("Waiting until " + formatDateTime(sendData) + " to send data again");
          disconnectFromWiFi();
      }
      
      //Serial.println("Loop done");
      Watchdog.reset();
      digitalWrite(LED_BUILTIN, LOW);
      lastLoopMillis = millis(); // set the timing for the next loop
      Watchdog.reset();
    }
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

void updateTimeKeeper()
{
    Watchdog.reset();
    Serial.println("Update timekeeper");
    connectToWiFi();
    Serial.println("WiFi connected, get time...");
    Udp.begin(localNTPPort);
    //Try up to this many times (UDP is uncertain...)
    int tries = 10;
    
    time_t pre = now();

    while(tries > 0) {
      Serial.print("Tries: ");
      Serial.print(tries);
      Serial.print("\n");
      tries--;
      Watchdog.reset();
      sendNTPpacket(timeServer); // send an NTP packet to a time server
      Watchdog.reset();
  
      // wait to see if a reply is available
      delay(1000);
      Watchdog.reset();
      if (Udp.parsePacket()) {
          Serial.println("UDP packet received");
  
          // We've received a packet, read the data from it
          Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
          //the timestamp starts at byte 40 of the received packet and is four bytes,
          // or two words, long. First, extract the two words:
  
          unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
          unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
          // combine the four bytes (two words) into a long integer
          // this is NTP time (seconds since Jan 1 1900):
          unsigned long secsSince1900 = highWord << 16 | lowWord;
          Serial.print("Seconds since Jan 1 1900 = ");
          Serial.println(secsSince1900);
          // now convert NTP time into everyday time:
          Serial.print("Unix time = ");
          // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
          const unsigned long seventyYears = 2208988800UL;
          // subtract seventy years:
          unsigned long epoch = secsSince1900 - seventyYears;

          // we got a valid date
          if(epoch > 10000)
          {
            setTime(epoch);
            Watchdog.reset();
  
            time_t pst = now();
            Serial.println("Time pre-Wifi:" + String((unsigned long)pre));
            Serial.println("Time postWifi:" + String((unsigned long)pst));

            // don't need another run
            tries = 0;
          }
      }
    }
    disconnectFromWiFi();
}

void printWifiStatus()
{
    // print status
    Serial.print("Status: ");
    Serial.println(WiFi.status());
    
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

// send an NTP request to the time server at the given address
// from https://www.arduino.cc/en/Tutorial/LibraryExamples/UdpNtpClient#code
void sendNTPpacket(IPAddress address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void connectToWiFi() {
  status = WiFi.status();
  Watchdog.reset();
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid.c_str(), pass.c_str());
    // wait up to 5 seconds for connection:
    time_t looptime = now() + (time_t)5;
    while(WiFi.status != WL_CONNECTED && now() < looptime) 
    {
      Serial.print(".");
      delay(200);
    }
    status = WiFi.status();
  }
  Serial.println("WiFi connected");
  printWifiStatus();
  Watchdog.reset();
}

void disconnectFromWiFi() {
  WiFi.end();
  status = WiFi.status();
  Watchdog.reset();
}
