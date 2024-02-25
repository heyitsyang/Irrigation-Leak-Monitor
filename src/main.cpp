/****************************
 *                          *
 * Irrigation Leak Detector *
 *                          *
 ****************************/
#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <ESPmDNS.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266mDNS.h>
#endif
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
// add the below libraries from the Library Manager
//#include <TelnetStream.h>
#include <PubSubClient.h>
#include <ezTime.h>

// local definitions
#include "esp_adc_cal.h"  // from LilyGo TZ example code
#include "prototypes.h"
#include "credentials.h"     // <<<<<<<  COMMENT THIS LINE OUT & ENTER YOUR CREDENTIALS BELOW - this contains stuff for my WIFI network, not yours

#define DEBUG 1     // comment line out to undefine - 0 is still defined
#ifdef DEBUG
  #define DEBUG_BEGIN(...) Serial.begin(__VA_ARGS__)
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
  #define DEBUG_FLUSH(...) Serial.flush(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(...) 
  #define DEBUG_PRINT(...) 
  #define DEBUG_PRINTLN(...) 
  #define DEBUG_PRINTF(...) 
  #define DEBUG_FLUSH(...) 
#endif

// name the device
#define DEVICE_HOST_NAME "irrig-leak"

// TIME SETTINGS
#define MY_TIMEZONE "America/New_York"               // <<<<<<< use Olson format: https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
#define TIMEZONE_EEPROM_OFFSET 0                     // location-to-timezone info - saved in case eztime server is down

#define VERSION "Ver 0.2 build 2024.02.1"

// GPIO PIN DEFINITIONS
#define BAT_ADC_PIN 12
#define BUILT_IN_LED_PIN 17
#define SLEEP_INHIBIT_PIN 21

#define VALVE_1_PIN 15
#define VALVE_2_PIN 16
#define VALVE_3_PIN 8
#define VALVE_4_PIN 18

#define FLOW_SENSOR_BLUE_PIN 9                       // Hunter HC100FLOW flow meter - ACTIVE LOW
#define FLOW_SENSOR_RED_PIN 11                       // not used for wake since we can only use one ext0 source

#define I2C_SCL_PIN 14    
#define I2C_SDA_PIN 13
#define PRESSURE_SENSOR_I2C_ADDR 0x28                // TE M3200 pressure sensor

// OPERATIONAL PARAMETERS & PREFERENCES
#define TOT_NUM_VALVES 4
#define PRESSURE_SENSOR_INSTALLED 1
#define PREFER_FAHRENHEIT 1  
#define FLOW_SETTLE_SECS 10                          // wait for empty pipe to fill and settle - normally set to 35
#define INACTIVITY_TIMEOUT_SECS 90                   // wait this long before sleeping after last watering zone - normally set to 90
#define HEARTBEAT_SLEEP_SECS  3600                   // seconds of sleep between wellness check wakeups - normally set to 3600
#define COMMS_EXCHANGE_SECS 180                      // wait this long for comms exchange before sleeping
#define uS_TO_S_FACTOR 1000000ULL                    // Conversion factor for micro seconds to seconds
#define MAX_PRESSURE 100                             // max rated pressure of pressure sensor
#define PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS 60000  // how often a pressure sensor error (timestmap) is published if error condition true

// MQTT
#define MQTT_MSG_BUFFER_SIZE 512                            // for MQTT message payload
#define MQTT_MAX_TOPIC_SIZE 1024                            // max topic string size(can be up to 65535)
#define MAX_MQTT_CONNECT_ATTTEMPTS 10


#define IRRIG_LWT_TOPIC "irrig_leak/status/LWT"             // MQTT Last Will & Testament
#define IRRIG_VERSION_TOPIC "irrig_leak/version"            // report software version at connect
#define IRRIG_WIFI_STRENGTH_TOPIC "irrig_leak/wifi_dbm"
#define IRRIG_BATTERY_VOLTS_TOPIC "irrig_leak/battery_volts"
#define IRRIG_BATTERY_PERCENT_TOPIC "irrig_leak/battery_percent"
#define IRRIG_IDLE_TIME_STAMP_TOPIC "irrig_leak/idle/time_stamp"
#define IRRIG_IDLE_PRESSURE_TOPIC "irrig_leak/idle/water_pressure"
#define IRRIG_IDLE_WATER_TEMPERATURE_TOPIC "irrig_leak/idle/water_temperature"
#define IRRIG_REPORT_TIME_STAMP_TOPIC "irrig_leak/report/time_stamp"
#define IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC "irrig_leak/report/tot_gals_all_zones"
#define IRRIG_VALVES_OFF_LEAK_TOPIC "irrig_leak/report/valve_leak"  // flow sensed when all valves are off & there should be none
#define IRRIG_GPM_TOPIC_PREFIX "irrig_leak/report/avg_gpm_zone"  // valve/zone number is appended to the end to create the complete topic
#define IRRIG_PSI_TOPIC_PREFIX "irrig_leak/report/avg_psi_zone"  // valve/zone number is appended to the end to create the complete topic

#define IRRIG_RECV_COMMAND_TOPIC "irrig_leak/cmd/#"

// MISC
#define READ_TEMPERATURE 0
#define READ_PRESSURE 1

RTC_DATA_ATTR int wakeup_level;                        // this is stored in RTC memory so it lasts through sleep

struct ZoneSummary
{
  u_int valveNum;
  u_int preMeasureGallons;
  u_int measuredZoneGallons;
  float averageGPM;
  float maxGPM;
  float averagePSI;
  float maxPSI;
  float minPSI;
  float waterTemperature;
//  byte filler;                                       // NVM requires even number of bytes for storage - uncomment only if necessary
};

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Timezone myTZ;

char mqttMsg[MQTT_MSG_BUFFER_SIZE];
char mqttTopic[MQTT_MAX_TOPIC_SIZE];
struct ZoneSummary zoneData[TOT_NUM_VALVES+1];           // array to keep flow data

uint32_t blinkMillis = 0;
int valveThisFlowTick = 0, valveLastFlowTick = -1;
esp_sleep_wakeup_cause_t wakeup_reason;
unsigned long lastReconnectAttempt = 0;
unsigned long lastPublish = 0, pressureLastRead = 0, lastValveSync = 0, lastPressErrReport = 0;
unsigned long flowTickNow, flowTickStart, flowTickPrev = 0, startSettling, flowTickDuration;
bool flowSettled = false;
unsigned long tempNow, lastPublishNow, pressureReadNow, mqttNow, lastPressErrReportNow;
byte sensorStatus;
float psiTminus0 = 0, psiTminus1 = 0, psiTminus2 = 0;         // psiTminus0 is the current pressure, psiTminus1 is the previous, psiTminus2 is the one before
float avgPressure, maxPressure, minPressure, currentPressure, temperature, runningTotPressure = 0; 
float instantGPM = 0, runningTotGPM = 0, avgGPM, maxGPM; 
unsigned int flowPulseCount = 0, flowPreMeasureCount = 0;;                             // one pulse = one gallon
unsigned int pressureReadInterval;
bool once;



/**********************
 *      SETUP
 **********************/
void setup()
{
  DEBUG_BEGIN(115200);

  pinMode(VALVE_1_PIN, INPUT);                // valve inputs
  pinMode(VALVE_2_PIN, INPUT);                // are active high
  pinMode(VALVE_3_PIN, INPUT);
  pinMode(VALVE_4_PIN, INPUT);
  pinMode(FLOW_SENSOR_BLUE_PIN, INPUT);       // flow inputs are
  pinMode(FLOW_SENSOR_RED_PIN, INPUT);        // active low w external pullup
  pinMode(BUILT_IN_LED_PIN, OUTPUT);
  pinMode(SLEEP_INHIBIT_PIN, INPUT_PULLUP);   // active low

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  DEBUG_PRINTF("\n\n\nIrrigation Leak Detector %s\n", VERSION);


  digitalWrite(BUILT_IN_LED_PIN, LOW);        // LED off

  DEBUG_PRINTLN("Size of struct ZoneSummary = " + String(sizeof(ZoneSummary)));    // anything written to flash must be even number of bytes
  
  if (digitalRead(SLEEP_INHIBIT_PIN) == LOW)
  {
    DEBUG_PRINTLN("\nSleep inhibit PIN active\n--------------------------------\n");
    wakeup_reason = ESP_SLEEP_WAKEUP_EXT0;   // simulate this so loop will run
  }
  else
    wakeup_reason = esp_sleep_get_wakeup_cause();

  digitalWrite(BUILT_IN_LED_PIN, HIGH);             // LED on
  flowPulseCount = 0;
  flowTickDuration = 0;
  flowTickPrev = 0;
  avgPressure = 0;
  maxPressure = 0;
  minPressure = 0;
  runningTotPressure = 0;
  avgGPM = 0;
  runningTotGPM = 0;
  instantGPM = 0;
  startSettling = millis();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:                                               // wait FLOW_SETTLE_TIME then count pulses until timeout
    {
      print_wakeup_reason(wakeup_reason);
      while (true)
      {
        if (digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW)
        {
          currentPressure = readPressureSensor(READ_PRESSURE);
          if (maxPressure < currentPressure)
            maxPressure = currentPressure;
          if (minPressure > currentPressure)
            minPressure = currentPressure;
          valveThisFlowTick = getActiveValve();
          if (valveThisFlowTick != valveLastFlowTick)                         // if zone has changed
          {
            once = true;
            startSettling = millis();
            flowPulseCount = 1;
            flowPreMeasureCount = 0;                                          // initialize varialbles
            avgPressure = currentPressure;
            maxPressure = currentPressure;
            minPressure = currentPressure;
            runningTotPressure = currentPressure;
            instantGPM = 0;
            maxGPM = 0;
            avgGPM = 0;
            runningTotGPM = 0;
          }
          else
          {
            flowPulseCount++;
            runningTotPressure = runningTotPressure + currentPressure;
            avgPressure = runningTotPressure/flowPulseCount;
          }

          flowTickStart = millis();   // establish start time

          digitalWrite(BUILT_IN_LED_PIN, HIGH);        // LED on
          while(digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW)      // wait for pulse to go low
          {
            flowTickNow = millis();
            if ((flowTickNow - flowTickStart) > (INACTIVITY_TIMEOUT_SECS * 1000))   // in case magnet stops on LOW
            { 
              DEBUG_PRINTLN(F("\nINACTIVITY_TIMEOUT_SECS while active low FLOW_SENSOR_BLUE_PIN = LOW"));
              exchangeComms(wakeup_reason);
              GoToSleep();
              break;             // break out of loop in case SLEEP_INHIBIT_PIN is set
            }
            yield();
          }
          digitalWrite(BUILT_IN_LED_PIN, LOW);        // LED off

          if((millis() - startSettling) > (FLOW_SETTLE_SECS * 1000))   // settling time reached
          {
            flowTickDuration = (flowTickStart - flowTickPrev);
            if(once)                                                // only execute once per zone
            {
              once = false;
              flowPreMeasureCount = flowPulseCount;                    // save count
              flowPulseCount = 1;
              runningTotGPM = 0;
              runningTotPressure = currentPressure;
              DEBUG_PRINTF("\nFLOW MEASUREMENT STARTED, %d gallons flowed pre-measurement\n", flowPreMeasureCount);
            }
            
            if (flowTickDuration > 0)                    // flowTickPrev is zero on 1st time thru
            {
              instantGPM = (60 * 1000)/flowTickDuration;  // instantaneos gallons per minute
              if (instantGPM > maxGPM)
                maxGPM = instantGPM;
              runningTotGPM = runningTotGPM + instantGPM;
              avgGPM = runningTotGPM/flowPulseCount;
              zoneData[valveThisFlowTick].averageGPM = avgGPM;
              zoneData[valveThisFlowTick].maxGPM = maxGPM;
              zoneData[valveThisFlowTick].valveNum = valveThisFlowTick;
              zoneData[valveThisFlowTick].measuredZoneGallons = flowPulseCount;
              zoneData[valveThisFlowTick].preMeasureGallons = flowPreMeasureCount;
              zoneData[valveThisFlowTick].averagePSI = avgPressure;
              zoneData[valveThisFlowTick].maxPSI = maxPressure;
              zoneData[valveThisFlowTick].minPSI = minPressure;
              zoneData[valveThisFlowTick].waterTemperature = readPressureSensor(READ_TEMPERATURE);
            }

            DEBUG_PRINTF("valveThisFlowTick = %d  flowPulseCount = %d, currentPressure =  %.2f avgPressure = %.2f  maxPressure = %.2f  minPressure = %.2f  runningTotPressure = %.2f\n",
                         valveThisFlowTick, flowPulseCount, currentPressure, avgPressure, maxPressure, minPressure, runningTotPressure);
            DEBUG_PRINTF("                  flowTickDuration = %lu instantGPM = %.2f  avgGPM = %.2f  runningTotGPM = %.2f \n", flowTickDuration, instantGPM, avgGPM, runningTotGPM);
          }

          flowTickPrev = flowTickStart;
          valveLastFlowTick = valveThisFlowTick;
        } 
        else // flow signal is HIGH, do nothing except check for timeout & continue loop
        {
          flowTickNow = millis();
          if ((flowTickNow - flowTickStart) > (INACTIVITY_TIMEOUT_SECS * 1000))  // in case magnet stops on HIGH
          {
            DEBUG_PRINTLN(F("\nINACTIVITY_TIMEOUT_SECS while active low FLOW_SENSOR_BLUE_PIN = HIGH"));
            exchangeComms(wakeup_reason);
            GoToSleep();
            break;        // break out of loop in case SLEEP_INHIBIT_PIN is set
          }
          yield();
        }


      }
      break;
    }
  
    case ESP_SLEEP_WAKEUP_TIMER:                                                // send idle time info and go back to sleep
    {
      // send wellness check-in info to MQTT
      print_wakeup_reason(wakeup_reason);
      DEBUG_PRINTF("Awakened on I/O level = %d\n", wakeup_level);
      exchangeComms(wakeup_reason);
      DEBUG_PRINTLN(F("--------------------------------\n"));
      GoToSleep();
      break;
    }

    default: 
      print_wakeup_reason(wakeup_reason);
  }

  DEBUG_PRINTLN(F("SHOULD NEVER REACH THIS"));
  GoToSleep();
}

/*********************** 
 *       LOOP
 ***********************/
void loop()
{ }  // never reaches here }




/*********************************
 *        SUBROUTINES
 *********************************/

/*
 * exchangeComms
 */
void exchangeComms(esp_sleep_wakeup_cause_t w_reason)
{
  unsigned long startTick, pauseTick;

  digitalWrite(BUILT_IN_LED_PIN, HIGH);        // LED on
  setup_wifi();
  setup_OTA();
  waitForSync();  //sync the time
  setInterval(0);  //do not periodically sync NTP
  myTZ.setLocation(F(MY_TIMEZONE)); 
  DEBUG_PRINTF("Got local time: %s\n", myTZ.dateTime("[H:i:s.v]").c_str());
  connectMQTT();

  mqttClient.publish(IRRIG_VERSION_TOPIC, VERSION, true); // report firmware version
  DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VERSION_TOPIC, VERSION);

  sprintf(mqttMsg, "%d", WiFi.RSSI());
  mqttClient.publish(IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg, true);
  DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_STRENGTH_TOPIC , "true");

  switch (w_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
    {
      mqttClient.publish(IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
      DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());

      sendBatteryStatus();
      sendTotalsReport();
      
      // wait a bit for potential OTA & MQTT exchanges
      startTick = millis();
      DEBUG_PRINTF("\nWaiting %d seconds for any OTA & MQTT traffic...\n", COMMS_EXCHANGE_SECS);
      while ((millis() - startTick) < (COMMS_EXCHANGE_SECS * 1000))
      {
        mqttClient.loop();
        digitalWrite(BUILT_IN_LED_PIN, LOW);        // LED off
        pauseTick = millis();
        while( (millis() - pauseTick) < 400 );
        digitalWrite(BUILT_IN_LED_PIN, HIGH);        // LED on
        pauseTick = millis();
        while( (millis() - pauseTick) < 200 );
        ArduinoOTA.handle();                        // remember program never returns from OTA
      }
      
      break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
    {
      mqttClient.publish(IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
      DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
      
      sendBatteryStatus();
      sendPressureSensorStatus();
      
      break;
    }
    default:
      DEBUG_PRINTF("WAKE REASON = %d - NO COMMS EXCHANGED\n", w_reason);
  }
  return;
}


/*
 * setup_wifi
 */
void setup_wifi()
{
  u_int j;
  unsigned long pauseTick;
  
  // Connect to a WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOST_NAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);   // set to maximum possible (draws 150mA)
  DEBUG_PRINTLN(WiFi.macAddress());

  DEBUG_PRINT(F("\nWaiting for WiFi "));
  while (WiFi.waitForConnectResult() != WL_CONNECTED)   // connection happens in a different thread - just waiting for results here
  {
    DEBUG_PRINT(F("."));
    for(j=0; j<60; j++)     // blink LED for 60s while trying to connect
    {
      digitalWrite(BUILT_IN_LED_PIN, LOW);        // LED off
      pauseTick = millis();
      while( (millis() - pauseTick) < 500 );
      digitalWrite(BUILT_IN_LED_PIN, HIGH);        // LED on
      pauseTick = millis();
      while( (millis() - pauseTick) < 500 );
    }
    ESP.restart();   // restart effectively starts sleep mode (initial state) & all data is lost
  }

  //randomSeed(micros());

  DEBUG_PRINTLN(F(""));
  DEBUG_PRINT(F("WiFi connected to "));
  DEBUG_PRINTLN(WIFI_SSID);
  DEBUG_PRINT(F("IP address: "));
  DEBUG_PRINTLN(WiFi.localIP());
}


/*
 * setupOTA
 */
void setup_OTA()
{
  // Port defaults to 8266
  //  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(DEVICE_HOST_NAME);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";  // U_FS

    // NOTE: if updating FS this is the place to unmount FS using FS.end()

    DEBUG_PRINTLN("OTA Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_PRINTLN(F("\nOTA End"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_PRINTF("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_PRINTF("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      DEBUG_PRINTLN(F("OTA Auth Failed"));
    else if (error == OTA_BEGIN_ERROR)
      DEBUG_PRINTLN(F("OTA Begin Failed"));
    else if (error == OTA_CONNECT_ERROR)
      DEBUG_PRINTLN(F("OTA Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR)
      DEBUG_PRINTLN(F("OTA Receive Failed"));
    else if (error == OTA_END_ERROR)
      DEBUG_PRINTLN(F("OTA End Failed"));
  });
  ArduinoOTA.begin();
}

/*
 * connectMQTT
 */
void connectMQTT()
{
  int connectAttemptCount = 0;
  mqttClient.setBufferSize(MQTT_MSG_BUFFER_SIZE);
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(callback);
  while(connectAttemptCount < MAX_MQTT_CONNECT_ATTTEMPTS)
  {
    if (!mqttClient.connected())
    {
      mqttNow = millis();
      if (mqttNow - lastReconnectAttempt > 1000)
      {
        Serial.printf("[%s] Waiting for MQTT...\n", myTZ.dateTime(RFC3339).c_str());
        lastReconnectAttempt = mqttNow;
        connectAttemptCount++;
        // Attempt to reconnect
        if (reconnect())
        {
          mqttClient.publish(IRRIG_LWT_TOPIC, "Connected", true); // let broker know we're connected
          DEBUG_PRINTF("\n%s MQTT SENT: %s/Connected\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_LWT_TOPIC);
          lastReconnectAttempt = 0;
          return;
        }
      }
    }
  }
  DEBUG_PRINTLN("ERROR-----Max MQTT connect attempts exceeded");
}

/*
 * MQTT callback
 */
void callback(char *topic, byte *payload, unsigned int length)
{
  // handle MQTT message arrival
  bool cmdValid = false;
  strncpy(mqttMsg, (char *)payload, length);
  mqttMsg[length] = (char)NULL; // terminate the string
  DEBUG_PRINTF("\n%s MQTT RECVD: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), topic, mqttMsg);
}

/*
 * MQTT reconnect
 */
boolean reconnect()
{

  // PubSubClient::connect(const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage)
  if (mqttClient.connect(DEVICE_HOST_NAME, MQTT_USER_NAME, MQTT_PASSWORD, IRRIG_LWT_TOPIC, 2, true, "Disconnected"))
  {
    DEBUG_PRINT(F("MQTT connected to "));
    DEBUG_PRINTLN(F(MQTT_SERVER));
  }
  return mqttClient.connected();
}


/*
 * sendTotals
 */
void sendTotalsReport()
{
  int i;
  unsigned int valveOffLeakGals = 0, galsAllZones = 0;

  for(i=0; i<(TOT_NUM_VALVES+1); i++)
  {
    // send GPM
    sprintf(mqttTopic, "%s_%d", IRRIG_GPM_TOPIC_PREFIX, i);

    sprintf(mqttMsg, "%.2f", zoneData[i].averageGPM);
    mqttClient.publish(mqttTopic, mqttMsg, true);       // send average GPM
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"preMeasureGallons\": \"%d\", \"measuredZoneGallons\": \"%d\", \"maxGPM\": \"%.2f\"}", 
                       zoneData[i].valveNum, zoneData[i].preMeasureGallons, zoneData[i].measuredZoneGallons, zoneData[i].maxGPM);
    sprintf(mqttTopic, "%s_%d%s", IRRIG_GPM_TOPIC_PREFIX, i, "/attributes");
    mqttClient.publish(mqttTopic, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    // send PSI & temperature
    sprintf(mqttTopic, "%s_%d", IRRIG_PSI_TOPIC_PREFIX, i);

    sprintf(mqttMsg, "%.2f", zoneData[i].averagePSI);
    mqttClient.publish(mqttTopic, mqttMsg, true);       // send average PSI
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"maxPSI\": \"%.2f\", \"minPSI\": \"%.2f\", \"waterTemperature\": \"%.2f\"}", 
                       zoneData[i].valveNum, zoneData[i].maxPSI, zoneData[i].minPSI, zoneData[i].waterTemperature);
    sprintf(mqttTopic, "%s_%d%s", IRRIG_PSI_TOPIC_PREFIX, i, "/attributes");
    mqttClient.publish(mqttTopic, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    galsAllZones = galsAllZones + zoneData[i].preMeasureGallons + zoneData[i].measuredZoneGallons;

    if (zoneData[i].valveNum == 0)   // valNum == 0 means no valves are ON, so any flow must be leak
      valveOffLeakGals = valveOffLeakGals + zoneData[i].preMeasureGallons + zoneData[i].measuredZoneGallons;  // accumulate leak gals
  }

  sprintf(mqttMsg, "%d", galsAllZones);
  mqttClient.publish(IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC , mqttMsg, true);
  DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC, mqttMsg);

  if (valveOffLeakGals > 0)
  {
    sprintf(mqttMsg, "%d", valveOffLeakGals);
    mqttClient.publish(IRRIG_VALVES_OFF_LEAK_TOPIC, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VALVES_OFF_LEAK_TOPIC, mqttMsg);
  }
  else
    mqttClient.publish(IRRIG_VALVES_OFF_LEAK_TOPIC, "0", true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VALVES_OFF_LEAK_TOPIC, "0");
}

/*
 * sendPressureSensorStatus
 */
void sendPressureSensorStatus()
{
  float idleWaterPressure, idleWaterTemperature;
  
  if (PRESSURE_SENSOR_INSTALLED)
  {
    idleWaterPressure = readPressureSensor(READ_PRESSURE);
    idleWaterTemperature = readPressureSensor(READ_TEMPERATURE);
    sprintf(mqttMsg, "%.2f", idleWaterPressure);
    mqttClient.publish(IRRIG_IDLE_PRESSURE_TOPIC, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_PRESSURE_TOPIC, mqttMsg);
    sprintf(mqttMsg, "%.2f", idleWaterTemperature);
    mqttClient.publish(IRRIG_IDLE_WATER_TEMPERATURE_TOPIC, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_WATER_TEMPERATURE_TOPIC, mqttMsg);
  }
}

/*
 * sendBatteryStatus
 */
void sendBatteryStatus()
{
  float voltage = 0.0;
  uint8_t percentage = 0;

  voltage = ((float)((readADC_Cal(analogRead(BAT_ADC_PIN))) * 2))/1000;

  percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
  if (voltage >= 4.20) percentage = 100;
  if (voltage <= 3.30) percentage = 0;  // orig 3.5
  
  sprintf(mqttMsg, "%.2f", voltage);
  mqttClient.publish(IRRIG_BATTERY_VOLTS_TOPIC, mqttMsg, true);  
  DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_BATTERY_VOLTS_TOPIC, mqttMsg);
  sprintf(mqttMsg, "%d", percentage);
  mqttClient.publish(IRRIG_BATTERY_PERCENT_TOPIC, mqttMsg, true);  
  DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_BATTERY_PERCENT_TOPIC, mqttMsg);
}

/*
 * GoToSleep
 */
void GoToSleep()
{
    if(digitalRead(SLEEP_INHIBIT_PIN) == HIGH)  // if sleep not inhibited (active low)
    {
      // First we configure when to wakeup next
      digitalWrite(BUILT_IN_LED_PIN, LOW);
      esp_sleep_enable_timer_wakeup(HEARTBEAT_SLEEP_SECS * uS_TO_S_FACTOR);          // periodic wellness checkin
      wakeup_level = (!digitalRead(FLOW_SENSOR_BLUE_PIN));                               // set new wakeup level to opposite of current
      esp_sleep_enable_ext0_wakeup((gpio_num_t)FLOW_SENSOR_BLUE_PIN, wakeup_level);      // wake if state changes
      DEBUG_PRINTF("%s Going to sleep now for %ds. Will wake on I/O level %d\nZzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz\n\n", 
                    myTZ.dateTime("[H:i:s.v]").c_str(), HEARTBEAT_SLEEP_SECS, wakeup_level);
      DEBUG_FLUSH();
      esp_deep_sleep_start();
      DEBUG_PRINTLN(F("This will never be printed"));
    }
    else
      yield();
}


/*
 * readADC_Cal
 */
uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}


/*
 * getActiveValve
 */
int getActiveValve()                     // zero is the first valve/zone
{
  if (digitalRead(VALVE_1_PIN) == HIGH)
    return(1);
  if (digitalRead(VALVE_2_PIN) == HIGH)
    return(2);
  if (digitalRead(VALVE_3_PIN) == HIGH)
    return(3);
  if (digitalRead(VALVE_4_PIN) == HIGH)
    return(4);
  return(0);
}


/*
 * print_wake_reason
 */
void print_wakeup_reason(esp_sleep_wakeup_cause_t w_reason)
{
  switch (w_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : 
    {
      DEBUG_PRINTLN(F("\nWakeup caused by external signal using RTC_IO"));
      DEBUG_PRINTF("Awakened on I/O level = %d\n", wakeup_level);
      break;
    }
    case ESP_SLEEP_WAKEUP_EXT1 :
    {
      DEBUG_PRINTLN(F("\nWakeup caused by external signal using RTC_CNTL"));
      break;
    }
    case ESP_SLEEP_WAKEUP_TIMER :
    {
      DEBUG_PRINTLN(F("\nWakeup caused by timer"));
      break;
    }
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
    {
      DEBUG_PRINTLN(F("\nWakeup caused by touchpad"));
      break;
    }
    case ESP_SLEEP_WAKEUP_ULP :
    {
      DEBUG_PRINTLN(F("\nWakeup caused by ULP program"));
      break;
    }
    default :
    {
      DEBUG_PRINTF("\nWakeup was not caused by deep sleep: %d\n", w_reason);
      break;
    }
  }
}


/*
 * readPressureSensor
 */
float readPressureSensor(int pressOrtemp)
{
  // Read sensor
  if (PRESSURE_SENSOR_INSTALLED)
  {
    sensorStatus = 0xFF; // set to non-zero for initial while() test
    pressureReadNow = millis();
    if ((unsigned long)(pressureReadNow - pressureLastRead) > pressureReadInterval)
    {
      pressureLastRead = millis();
      while (sensorStatus != 0) // continue reading until valid
      {
        Wire.requestFrom(PRESSURE_SENSOR_I2C_ADDR, 4); // request 4 bytes  - if optional 3rd argument false = don't share i2c bus
        int n = Wire.available();
        if (n == 4)
        {
          sensorStatus = 0;  // set to zero to exit while loop when read is successful
          uint16_t rawP; // pressure data from sensor
          uint16_t rawT; // temperature data from sensor

          rawP = (uint16_t)Wire.read(); // upper 8 bits
          rawP <<= 8;
          rawP |= (uint16_t)Wire.read(); // lower 8 bits
          rawT = (uint16_t)Wire.read();  // upper 8 bits
          rawT <<= 8;
          rawT |= (uint16_t)Wire.read(); // lower 8 bits

          sensorStatus = rawP >> 14; // The status is 0, 1, 2 or 3
          rawP &= 0x3FFF;            // keep 14 bits, remove status bits

          rawT >>= 5; // the lowest 5 bits are not used

          psiTminus0 = ((rawP - 1000.0) / (15000.0 - 1000.0)) * MAX_PRESSURE;
          temperature = ((rawT - 512.0) / (1075.0 - 512.0)) * 55.0;
        }
        else
        {
          lastPressErrReportNow = millis();
          if ((unsigned long)(lastPressErrReportNow - lastPressErrReport) > (unsigned long)PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS)
          {
            DEBUG_PRINTLN(F("Error reading pressure sensor"));
            // mqttClient.publish(PRESSURE_SENSOR_FAULT_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
            // DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), PRESSURE_SENSOR_FAULT_TOPIC, myTZ.dateTime(RFC3339).c_str());
            lastPressErrReport = millis();
          }
          while ((millis() - lastPressErrReportNow) < pressureReadInterval)   // prevent cycling too fast
            yield();
          break;
        }
      }
    }

  }
  else
    return(0);

  if (pressOrtemp == READ_TEMPERATURE)
  {
    if (PREFER_FAHRENHEIT)
      return( (temperature * 9/5) + 32 );
    else
      return(temperature);
  }
  else
    return(psiTminus0);
}