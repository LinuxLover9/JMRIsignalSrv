/*
  CHK version 2024-12-14

  This sketch is for a MQTT signal server to be used with JMRI or other layout control.
  An ESP8266 with one or more SN74HC59N shift registers will drive the signal LEDs
  It's setup to use bi-polar Green/Red LEDs.
  For these bi-polar LEDs in series with a resistor are connected to 2 pins of the shift register.
  Therefore only 4 LEDs per shift register.
  I tested with a TI SN74HC595N which has a build in pull up. This means that with one pin high
  and the other low, there is 5 VDC between the pins. Depending on which pin is pulled high either
  green or red LED will light up.

  2024-12-16 adding incandessent simulation.
  2024-12-19 adding MQTT support for JMRI commands
             JMRI only knows lights to go ON /OFF
             Now we support the following messages
             toppic: JMRI/signal/light/set/<light-n>/{green|red|yellow|flashing}
             message: ON|OFF
  2024-12-23 reworking the dimming logic
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <time.h>
#include "SSID_access.h"


// ============================== GeneralDefinitions ============================ //
const char* myHostname ="HOsrv01";  // device identifier
const char* version = "241223";     // version is by date
#define DOnotDEBUG
#if defined DODEBUG
const boolean DEBUG = true;        // debugging variable
#else
const boolean DEBUG = false;        // debugging variable
#endif
const char* ssid = MY_SSID;         // SSID pulled from "SSID_access.h"
const char* password = SSID_PSW;    // password pulled from "SSID_access.h"
// boolean blinkOn = true;
// int blinks = 0;
// int blinkCnt = 0;
// int blinkDelay = 100;
// long bluePublish = 0;
long now = millis();
// long blinkTime = millis();
// String State = "on";
long cycleCnt = 0;                  // Statistics variable to check performance
const int cyclePeriod = 10000;      // count per 10 sec
long cycleStart = millis();
float cycleStats = 0;               // track the total of the cycleCounts per publish time

struct signal {                     // the structure of the signalHead variables
  String name;                      // username of the signal head
  String aspect;                    // aspect the light is to show
  String currentAspect;             // what are we showing that need dimming
  String targetAspect;              // what aspect do we change to
  String flashingAspect;            // save the aspect when flashing
  int pin;                          // state of the pins used
  boolean flash;                    // does the head need to flash
  uint8_t dimPattern;                   // contains the brightness pattern
  int dimIndex;                     // tracks the bit showing LED on ir off
  int dimStep;                      // time of next dim step
};
#if defined(DODEBUG)
long dimStepTime = 99;              // time interval between dimming steps. 1/8 of the time to fully dim, 1/16 of the time to switch light colour.
#else
long dimStepTime = 44;              // time interval between dimming steps. 1/8 of the time to fully dim, 1/16 of the time to switch light colour.
#endif
#define numSignalHeads 4            // four heads per shift register
struct signal SignalHead[numSignalHeads]={
  {.name="AMW-A", .aspect="RED", .currentAspect="DARK",  .targetAspect="RED", .pin=0, .flash=false, .dimPattern=255, .dimIndex=0, .dimStep=0},
  {.name="AMW-B", .aspect="RED", .currentAspect="DARK",  .targetAspect="RED", .pin=2, .flash=false, .dimPattern=255, .dimIndex=0, .dimStep=0},
  {.name="AMW-C", .aspect="RED", .currentAspect="DARK",  .targetAspect="RED", .pin=4, .flash=false, .dimPattern=255, .dimIndex=0, .dimStep=0},
  {.name="AMW-D", .aspect="RED", .currentAspect="DARK",  .targetAspect="RED", .pin=6, .flash=false, .dimPattern=255, .dimIndex=0, .dimStep=0}
};                                  // initialization of the signal heads
uint8_t signalPins = B10101010;     // this is the variable to push the pin values into the shift register
#if defined(DODEBUG)
  const int flashTime = 10000;      // flash time is set to 10 sec.
#else
  const int flashTime = 1000;       // flash time is set to 1 sec.
#endif
long lastFlashTime = 0;             // variable to keep track of the flash time
boolean flashOn = true;             // variable of the flash state.
byte mask = 0;                      // variable to mask pins
byte myPins = 3;                    // variable to set the pins of the head.
byte outherPins = B00111111;        // mask to retain other pin values.
uint8_t yellowCycle = B11101110;    // this is the pattern for the mix of green and red to make amber/yellow
boolean setGreen = true;            // are we to show green or red?
int internalCycle = 0;              // create a dimming cycle for the internal LED
const int dimBlue = 96;             // dim limit count down, cannot use the same as for the other LEDs

// =============================== Pin Definitions =============================== //
int dataPin = D6;                   // pin D6 on NodeMCU boards for data bits
int latchPin = D7;                  // pin D7 on NodeMCU boards for latching
int clockPin = D8;                  // pin D8 on NodeMCU boards for clock pulse

// ========================= function declarations =============================== //

void callback(char* topic, byte* payload, unsigned int length);
boolean reconnect();
WiFiClient espClient;
String utcTime();
void showTime();
boolean isGreen(uint8_t *yellowCycle);
void printBinary(byte inByte);
void publishAspect(int s);
void publishFlashing(int s);
void publishDebug(String message);
String trueAspect(signal signalHead);

// ===================================== OTA ==================================== //
//
long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

// ===================================== NTP ==================================== //
// IPAddress timeServerIP = "192.168.0.60";
// unsigned int localPort = 2390; // local port to listen for UDP packets
#define NTP_SERVER "pfsense.klomp.ca" // your time server
#define MY_TZ "EST+5EDT,M3.5.0/02,M10.5.0/03"
time_t now_t;                       // this are the seconds since Epoch (1970) - UTC
tm tm;                              // the structure tm holds time information in a more convenient way

// ============================== MQTT Definitions =============================== //
#define mqtt_server "mqtt.klomp.ca" // your MQTT server
#define mqtt_port 41883
// reconnect every 5 seconds
long lastReconnectAttempt = 0;
long nextPublish = 0;               // variale to keep track of the last publish message
#define publish_delay 600000        // 10 min between publishings
char* topicPrefix = (char*) "JMRI/signal/"; // topic prefix for MQTT communication
String message ="";
PubSubClient client(mqtt_server, mqtt_port, callback, espClient);


// =============================================================================== //
//                                Setup procedures                                 //
// =============================================================================== //

void setup() {
  // Serial.begin(115200);
  Serial.begin(9600);
  Serial.println("\nBooting");

  // blinkOn = true;
  // blinks = 1;

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  signalPins = B01010101;
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, signalPins);
  digitalWrite(latchPin, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.begin(MY_SSID, SSID_PSW);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(myHostname);

  // No authentication by default
  //ArduinoOTA.setPassword((const char *)"PwrSw01.OTA");
  ArduinoOTA.setPassword((const char *)OTA_PSW);

  ArduinoOTA.onStart([]() {
    Serial.println("Start OTA loading...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd OTA loading.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // setting up MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Init and get the time
  configTime(MY_TZ, NTP_SERVER);
  showTime();

} // end setup


// =============================================================================== //
//                                    Main loop                                    //
// =============================================================================== //

void loop() {
  now = millis();

  // stats
  cycleCnt++;                                                      // increase count for every loop
  if ((now - cycleStart) > cyclePeriod){
    cycleStart = now;
    cycleStats += cycleCnt/(cyclePeriod/1000);                     // how many cycles did we per second
    cycleCnt = 0;
  }
  ArduinoOTA.handle();

  // Check MQTT
  if (!client.connected()) {
    if ((now - lastReconnectAttempt) > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {                                           // Attempt to reconnect
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();                                                 // Client connected
  }

  // light processing
  if ((now - lastFlashTime) > flashTime){                         // do we show light or not
    lastFlashTime = now;
    flashOn = not flashOn;
  }
  if (flashOn){
    internalCycle--;
    if (internalCycle < 0){
      internalCycle = dimBlue;
      digitalWrite(LED_BUILTIN, LOW);                             // flash the build-in LED with this rate
    } else digitalWrite(LED_BUILTIN, HIGH);
  }

  setGreen = isGreen(&yellowCycle);                               // do we show green or red?

  for(int s=0; s<numSignalHeads; s++){                            // process for every signal head
    myPins = SignalHead[s].pin << (s*2);                          // get from aspect
    mask = 3 << (s*2);                                            // get the right mask
    if (SignalHead[s].currentAspect.indexOf("YELLOW") == 0){      // for Yellow we need to alternate colours
      if (setGreen) myPins = B10<< (s*2);                         // show green
      else myPins = B01<< (s*2);                                  // or red
    } else if (SignalHead[s].currentAspect.indexOf("DARK") == 0){ // for dark
      myPins = B11 << (s*2);                                      // dark for both pins high
    }

    if(SignalHead[s].flash) {                                     // do we need to flash?
      if(flashOn) SignalHead[s].targetAspect = SignalHead[s].aspect;
      else SignalHead[s].targetAspect = "DARK";
    }

    if ((SignalHead[s].currentAspect.compareTo(SignalHead[s].aspect) != 0) ||
        (SignalHead[s].currentAspect.compareTo(SignalHead[s].targetAspect) != 0)){ // do we need to dim?
      SignalHead[s].dimIndex++;
      if (SignalHead[s].dimIndex > 7) SignalHead[s].dimIndex=0;
      if ((SignalHead[s].dimPattern & (1 << SignalHead[s].dimIndex)) == 0) myPins = B11 << (s*2); // dimming

      if (now > SignalHead[s].dimStep){                           // time to dim more
        SignalHead[s].dimStep = now + dimStepTime;
        if ((SignalHead[s].currentAspect.compareTo("DARK") == 0) & (SignalHead[s].targetAspect.compareTo("DARK") != 0)){  // we need to brighten
          SignalHead[s].dimPattern = SignalHead[s].dimPattern*2 + 1;
          if (SignalHead[s].targetAspect.compareTo("GREEN")) SignalHead[s].pin = 1;
          else if (SignalHead[s].targetAspect.compareTo("RED")) SignalHead[s].pin = 2;
          myPins = SignalHead[s].pin << (s*2);                   // enable high pin

          if (SignalHead[s].dimPattern == 255){                   // were done
            SignalHead[s].currentAspect = SignalHead[s].targetAspect;
          }
        } else {                                                  // we need to darken
          SignalHead[s].dimPattern = SignalHead[s].dimPattern >> 1;
          if (SignalHead[s].dimPattern <= 0){                     // were done
            SignalHead[s].currentAspect = "DARK";                 // switch to brightning
            SignalHead[s].targetAspect = SignalHead[s].aspect;
            myPins = B11 << (s*2);                                // dark for both pins high
          }
        }
      }
        if (DEBUG && (s==3)) {
          Serial.print(SignalHead[s].currentAspect);
          Serial.print(">");
          Serial.print(SignalHead[s].targetAspect);
          Serial.print(" ");
          printBinary(SignalHead[s].dimPattern);
          Serial.print(" ");
          if (SignalHead[s].flash) Serial.print("F");
          else Serial.print("_");
          Serial.print(" ");
          printBinary(myPins);
          Serial.print(" ");
          printBinary(signalPins);
          Serial.println();
        }
    }

    outherPins = 0xFF ^ mask;                                     // mask for pins to retain
    signalPins = (signalPins & outherPins) | myPins ;             // insert pin states for current head
  }
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, signalPins);              // finally we write to shift register
  digitalWrite(latchPin, HIGH);

  // MQTT publish state
  if ((now > nextPublish) ){                       // is it time to publish?
    nextPublish = now + publish_delay;
    String payload = "";
    payload = utcTime();                                          // publish the current time
    String level = topicPrefix;
    level += myHostname;
    level += "/";
    level += "time";
    client.publish((char*)level.c_str(), (char*)payload.c_str());

    payload = String(cycleStats/(publish_delay/cyclePeriod));     // publish stats
    cycleStats = 0;
    level = topicPrefix;
    level += myHostname;
    level += "/stats";
    client.publish((char*)level.c_str(), (char*)payload.c_str());
  }
  if (DEBUG) delay(500);            // debug slow down so we can see what is happening
} // end of main loop


// =============================================================================== //
//                               MQTT processing                                   //
// =============================================================================== //

void callback(char* topic, byte* payload, unsigned int length) {
  if (DEBUG){
    Serial.print("Message arrived [");                              // show what we received
    Serial.print(topic);
    Serial.print("] '");
  }
  String pl ="";
  String tmpAspect;

  for (unsigned int i = 0; i < length; i++) {
    if (DEBUG) Serial.print((char)payload[i]);
    pl += (char)payload[i];
  }
  if (DEBUG) Serial.println("'");
  for(int s=0; s<numSignalHeads; s++){                             // do we receive a topic?
    String topicPub = topicPrefix;
    topicPub += SignalHead[s].name;
    String topicStr = topic;
    if ( topicStr.indexOf(SignalHead[s].name) > -1){
      if ( topicStr.indexOf("set") > -1 ){                                      // did we receive a set command?
        // payload OFF only for flashing and only when the aspect is changing
        if (topicStr.indexOf("flashing") > -1){
          if (topicStr.indexOf("flashing") > -1) {
            if (DEBUG) publishAspect(s);
            if (pl.compareTo("OFF") == 0) {
              // tmpAspect = SignalHead[s].aspect.substring(8);
              // publishDebug(tmpAspect);
              // SignalHead[s].aspect = tmpAspect;
              // SignalHead[s].targetAspect = tmpAspect;
              SignalHead[s].flash = false;
              // SignalHead[s].dimPattern = 255;
              // publishAspect(s);
              publishFlashing(s);
            } else {
              // SignalHead[s].targetAspect = SignalHead[s].aspect;
              // tmpAspect = "FLASHING";
              // tmpAspect += SignalHead[s].aspect;
              // publishDebug(tmpAspect);
              // SignalHead[s].aspect = tmpAspect;
              SignalHead[s].flash = true;
              // SignalHead[s].dimPattern = 255;
              // SignalHead[s].flashingAspect = SignalHead[s].targetAspect;
              // publishAspect(s);
              publishFlashing(s);
            }
          }
        } else {
          if (DEBUG){
            message = "reveived ON command:" ;
            message += topicStr;
            message += "=";
            message += pl;
            publishDebug(message);
          }
          if (pl.equalsIgnoreCase("GREEN") || ((topicStr.indexOf("green") >-1)&& (pl.compareTo("ON") == 0))){   // did we receive green aspect?
            SignalHead[s].aspect = "GREEN";
            SignalHead[s].targetAspect = "DARK";
            SignalHead[s].flash = false;
          } else if (pl.equalsIgnoreCase("RED") || ((topicStr.indexOf("red") >-1) && (pl.compareTo("ON") == 0)) ){
            SignalHead[s].aspect = "RED";
            SignalHead[s].targetAspect= "DARK";
            SignalHead[s].flash = false;
          } else if (pl.equalsIgnoreCase("YELLOW") || ((topicStr.indexOf("yellow") >-1) && (pl.compareTo("ON") == 0)) ){
            SignalHead[s].aspect = "YELLOW";
            SignalHead[s].targetAspect= "DARK";
            SignalHead[s].flash = false;
          } else if (pl.equalsIgnoreCase("DARK")){
            SignalHead[s].aspect = "DARK";
            SignalHead[s].targetAspect = "DARK";
            SignalHead[s].flash = false;
          } else if (pl.equalsIgnoreCase("FLASHINGGREEN")){
            SignalHead[s].aspect = "GREEN";
            SignalHead[s].targetAspect= "DARK";
            SignalHead[s].flash = true;
          } else if (pl.equalsIgnoreCase("FLASHINGRED")){
            SignalHead[s].aspect = "RED";
            SignalHead[s].targetAspect= "DARK";
            SignalHead[s].flash = true;
          } else if (pl.equalsIgnoreCase("FLASHINGYELLOW")){
            SignalHead[s].aspect = "YELLOW";
            SignalHead[s].targetAspect= "DARK";
            SignalHead[s].flash = true;
          }
          if (SignalHead[s].aspect.compareTo(SignalHead[s].currentAspect) != 0){
            client.publish(topicPub.c_str(), trueAspect(SignalHead[s]).c_str());
            publishAspect(s);
          }
        }
      } else if (pl.compareTo("?") == 0){                                    // did we receive a head query
        publishDebug("we're publishing on equest:");
        client.publish(topicPub.c_str(), trueAspect(SignalHead[s]).c_str());
        publishAspect(s);
      }else if (DEBUG) {
        message = "Command received, that I don't understand! : ";
        message += topicStr;
        message += "=";
        message += pl;
        publishDebug(message);
      }
    }
  }
} // end callback



boolean reconnect() {
  String topic = topicPrefix;
  topic += myHostname;
  Serial.print("Attempting MQTT connection from ");
  Serial.print(WiFi.localIP());
  Serial.print(" as ");
  String clientId = myHostname;
  Serial.println(myHostname);
  // Attempt to connect
  if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
    Serial.println("connected");

    // Once connected, publish an announcement...
    client.publish((char*)topic.c_str(), (char*)"Reconnected");
    Serial.println("connected to MQTT server");

    // ... and resubscribe
    topic = topicPrefix;
    topic += "#";
    client.subscribe((char*)topic.c_str());
  }
  return client.connected();
}


// =============================================================================== //
//         send an NTP request to the time server at the given address             //
// =============================================================================== //
String utcTime() {
  String utc = "";
  Serial.print("NTP time: ");
  Serial.println(time(&now_t));
  localtime_r(&now_t, &tm);           // update the structure tm with the current time
  utc += tm.tm_year + 1900;           // years since 1900
  utc += "-";
  utc += tm.tm_mon + 1;               // January = 0 (!)
  utc += "-";
  utc += tm.tm_mday;                  // day of month
  utc += " ";
  utc += tm.tm_hour;                  // hours since midnight  0-23
  utc += ":";
  utc += tm.tm_min;                    // minutes after the hour  0-59
  utc += ":";
  utc += tm.tm_sec;                    // seconds after the minute  0-61*
  utc += " ";
  if (tm.tm_isdst == 1)                // Daylight Saving Time flag
    utc += "EDT";
  else
    utc += "EST";
  return utc;
}

void showTime(){
  Serial.println(utcTime());
}


// when is green or red for yellow mix
boolean isGreen(uint8_t *yellowCycle){          // do we need to show green
  int bit7 = (*yellowCycle & B10000000)/128;    // get top bit of pattern
  *yellowCycle = *yellowCycle << 1;             // cycle all bits
  *yellowCycle = *yellowCycle | bit7;           // add bit to low end of the byte
  return(bit7 == 1);                            // if the bit is a 1 then it's green
}


void printBinary(byte inByte) {
  for (int b = 7; b >= 0; b--) {
    Serial.print(bitRead(inByte, b) ? '1' : '0');
  }
}


void publishAspect(int s){
  String pubTopic = topicPrefix;
  pubTopic += "light/";
  pubTopic += SignalHead[s].name;
  pubTopic += "/";

  String topicPub = pubTopic;
  String pubMsg = "OFF";
  topicPub += "green";
  if (SignalHead[s].aspect.indexOf("GREEN") > -1 ) pubMsg = "ON";
  client.publish((char*) topicPub.c_str(), (char*) pubMsg.c_str());

  topicPub = pubTopic;
  pubMsg = "OFF";
  topicPub += "yellow";
  if (SignalHead[s].aspect.indexOf("YELLOW") > -1 ) pubMsg = "ON";
  client.publish((char*) topicPub.c_str(), (char*) pubMsg.c_str());

  topicPub = pubTopic;
  pubMsg = "OFF";
  topicPub += "red";
  if (SignalHead[s].aspect.indexOf("RED") > -1 ) pubMsg = "ON";
  client.publish((char*) topicPub.c_str(), (char*) pubMsg.c_str());

  topicPub = pubTopic;
  pubMsg = "OFF";
  topicPub += "flashing";
  if (SignalHead[s].aspect.indexOf("FLASHING") > -1 ) pubMsg = "ON";
  client.publish((char*) topicPub.c_str(), (char*) pubMsg.c_str());
}


void publishFlashing(int s){
  String pubTopic = topicPrefix;
  pubTopic += "light/";
  pubTopic += SignalHead[s].name;
  pubTopic += "/";
  pubTopic += "flashing";
  String pubMsg = "OFF";
  if (SignalHead[s].flash) pubMsg = "ON";
  client.publish((char*) pubTopic.c_str(), (char*) pubMsg.c_str());
}


void publishDebug(String message){
  String topic = topicPrefix;
  topic += "DEBUG";
  if (DEBUG) client.publish((char*)topic.c_str(), (char*)message.c_str());
}


String trueAspect(signal signalHead){
  String ret ="";
  if (signalHead.flash) ret = "FLASHING";
  ret += signalHead.aspect;
  return(ret);
}
