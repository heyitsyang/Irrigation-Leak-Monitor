/****************************
 *                          *
 * Irrigation Leak Detector *
 *                          *
 ****************************/

/* To Do List
   - add watering duration for each zone to report
*/

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
#include "prototypes.h"
#include "credentials.h"     // <<<<<<<  COMMENT THIS LINE OUT & ENTER YOUR CREDENTIALS BELOW - this contains stuff for my WIFI network, not yours

#define DEBUG 1     // comment line out to undefine - setting to 0 is still considered defined
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

#define VERSION "Ver 0.3 build 2026.06.1"

// GPIO PIN DEFINITIONS
#define BUILT_IN_LED_PIN 17

#define VALVE_1_PIN 15
#define VALVE_2_PIN 16
#define VALVE_3_PIN 8
#define VALVE_4_PIN 18

#define FLOW_SENSOR_BLUE_PIN 9                       // Hunter HC100FLOW flow meter - ACTIVE LOW
#define FLOW_SENSOR_RED_PIN 11

#define I2C_SCL_PIN 13
#define I2C_SDA_PIN 14
#define PRESSURE_SENSOR_I2C_ADDR 0x28                // TE M3200 pressure sensor

// OPERATIONAL PARAMETERS & PREFERENCES
#define FLOW_GALS_PER_PULSE  1
#define TOT_NUM_VALVES 4
#define PRESSURE_SENSOR_INSTALLED 1
#define PREFER_FAHRENHEIT 1
#define FLOW_SETTLE_SECS 10                          // wait for empty pipe to fill and settle - normally set to 35
#define INACTIVITY_TIMEOUT_SECS 90                   // wait this long after last pulse before ending session - normally set to 90
#define HEARTBEAT_SECS 3600                          // seconds between wellness check-in publishes - normally set to 3600
#define MAX_PRESSURE 100                             // max rated pressure of pressure sensor
#define PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS 60000  // how often a pressure sensor error is published if error condition persists

// MQTT
#define MQTT_MSG_BUFFER_SIZE 512                            // for MQTT message payload
#define MQTT_MAX_TOPIC_SIZE 1024                            // max topic string size(can be up to 65535)
#define MAX_MQTT_CONNECT_ATTTEMPTS 10

#define IRRIG_LWT_TOPIC "irrig_leak/status/LWT"             // MQTT Last Will & Testament
#define IRRIG_VERSION_TOPIC "irrig_leak/version"            // report software version at connect
#define IRRIG_WIFI_STRENGTH_TOPIC "irrig_leak/wifi_dbm"
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
};

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Timezone myTZ;

char mqttMsg[MQTT_MSG_BUFFER_SIZE];
char mqttTopic[MQTT_MAX_TOPIC_SIZE];
struct ZoneSummary zoneData[TOT_NUM_VALVES+1];           // array to keep flow data

uint32_t blinkMillis = 0;
int valveThisFlowPulse = 0, valveLastFlowPulse = -1;
unsigned long lastReconnectAttempt = 0;
unsigned long lastPublish = 0, pressureLastRead = 0, lastPressErrReport = 0;
unsigned long millisNow, millisStart = 0, millisPrev = 0, startSettling, millisElapsed;
unsigned long pressureReadNow, mqttNow, lastPressErrReportNow;
byte sensorStatus;
float psiTminus0 = 0, psiTminus1 = 0, psiTminus2 = 0;
float avgPressure, maxPressure, minPressure, currentPressure, temperature, runningTotPressure = 0;
float instantGPM = 0, runningTotGPM = 0, avgGPM, maxGPM;
unsigned int flowPulseCount = 0, flowPreMeasurePulseCount = 0;
unsigned int pressureReadInterval;
bool once;

bool sessionActive = false;
unsigned long lastHeartbeatMs = 0;


/*
 * resetSessionData - clears per-irrigation-session accumulators
 */
void resetSessionData()
{
  memset(zoneData, 0, sizeof(zoneData));
  flowPulseCount = 0;
  flowPreMeasurePulseCount = 0;
  millisElapsed = 0;
  millisPrev = 0;
  avgPressure = 0;
  maxPressure = 0;
  minPressure = 0;
  runningTotPressure = 0;
  avgGPM = 0;
  runningTotGPM = 0;
  instantGPM = 0;
  valveLastFlowPulse = -1;
  once = false;
}


/*
 * publishSessionReport - send end-of-session data to MQTT broker
 */
void publishSessionReport()
{
  mqttClient.publish(IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
  DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
  sendTotalsReport();
  resetSessionData();
  millisStart = millis();  // reset so inactivity timeout doesn't re-fire immediately
}


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

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  DEBUG_PRINTF("\n\n\nIrrigation Leak Detector %s\n", VERSION);
  DEBUG_PRINTLN("Size of struct ZoneSummary = " + String(sizeof(ZoneSummary)));

  digitalWrite(BUILT_IN_LED_PIN, HIGH);       // LED on during init

  resetSessionData();
  millisStart = millis();

  setup_wifi();
  setup_OTA();
  waitForSync();  // sync the time; ezTime will re-sync periodically on its own schedule
  myTZ.setLocation(F(MY_TIMEZONE));
  DEBUG_PRINTF("Got local time: %s\n", myTZ.dateTime("[H:i:s.v]").c_str());
  connectMQTT();

  mqttClient.publish(IRRIG_VERSION_TOPIC, VERSION, true);
  DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VERSION_TOPIC, VERSION);

  sprintf(mqttMsg, "%d", WiFi.RSSI());
  mqttClient.publish(IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg, true);
  DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg);

  lastHeartbeatMs = millis();
  DEBUG_PRINTLN(F("\nSetup complete. Entering main loop."));
  DEBUG_PRINTLN(F("================================\n"));
}


/***********************
 *       LOOP
 ***********************/
void loop()
{
  ArduinoOTA.handle();

  // Reconnect WiFi if dropped
  if (WiFi.status() != WL_CONNECTED)
  {
    DEBUG_PRINTLN(F("WiFi lost, reconnecting..."));
    setup_wifi();
  }

  // Reconnect MQTT if dropped
  if (!mqttClient.connected())
    connectMQTT();
  mqttClient.loop();

  // Periodic heartbeat
  if ((unsigned long)(millis() - lastHeartbeatMs) >= (HEARTBEAT_SECS * 1000UL))
  {
    lastHeartbeatMs = millis();
    DEBUG_PRINTLN(F("\n--- Heartbeat ---"));
    mqttClient.publish(IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
    sprintf(mqttMsg, "%d", WiFi.RSSI());
    mqttClient.publish(IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg, true);
    sendPressureSensorStatus();
  }

  // Flow detection
  if (digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW)   // pulse is active when LOW
  {
    if (!sessionActive)
    {
      sessionActive = true;
      resetSessionData();
      startSettling = millis();
      DEBUG_PRINTLN(F("\n--- Irrigation session started ---"));
    }

    currentPressure = readPressureSensor(READ_PRESSURE);
    if (maxPressure < currentPressure)
      maxPressure = currentPressure;
    if (minPressure > currentPressure)
      minPressure = currentPressure;
    valveThisFlowPulse = getActiveValve();

    if (valveThisFlowPulse != valveLastFlowPulse)  // zone has changed
    {
      once = true;
      startSettling = millis();
      flowPulseCount = 1;
      flowPreMeasurePulseCount = 0;
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
      avgPressure = runningTotPressure / flowPulseCount;
    }

    millisStart = millis();

    digitalWrite(BUILT_IN_LED_PIN, HIGH);                // LED on during pulse
    while (digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW)     // wait for pulse to end
    {
      millisNow = millis();
      if ((millisNow - millisStart) > (INACTIVITY_TIMEOUT_SECS * 1000))  // magnet stuck on LOW
      {
        DEBUG_PRINTLN(F("\nINACTIVITY_TIMEOUT_SECS while FLOW_SENSOR_BLUE_PIN stuck LOW"));
        publishSessionReport();
        sessionActive = false;
        return;
      }
      yield();
    }
    digitalWrite(BUILT_IN_LED_PIN, LOW);                 // LED off after pulse

    if ((millis() - startSettling) > (FLOW_SETTLE_SECS * 1000))
    {
      millisElapsed = (millisStart - millisPrev);
      if (once)
      {
        once = false;
        flowPreMeasurePulseCount = flowPulseCount;
        flowPulseCount = 1;
        runningTotGPM = 0;
        runningTotPressure = currentPressure;
        DEBUG_PRINTF("\nFLOW MEASUREMENT STARTED, %d gallons flowed during pre-measurement settling time\n", flowPreMeasurePulseCount);
      }

      if (millisPrev > 0)
      {
        instantGPM = (60 * 1000) / millisElapsed;
        if (instantGPM > maxGPM)
          maxGPM = instantGPM;
        runningTotGPM = runningTotGPM + instantGPM;
        avgGPM = runningTotGPM / flowPulseCount;
        zoneData[valveThisFlowPulse].averageGPM = avgGPM;
        zoneData[valveThisFlowPulse].maxGPM = maxGPM;
        zoneData[valveThisFlowPulse].valveNum = valveThisFlowPulse;
        zoneData[valveThisFlowPulse].measuredZoneGallons = flowPulseCount;
        zoneData[valveThisFlowPulse].preMeasureGallons = flowPreMeasurePulseCount * FLOW_GALS_PER_PULSE;
        zoneData[valveThisFlowPulse].averagePSI = avgPressure;
        zoneData[valveThisFlowPulse].maxPSI = maxPressure;
        zoneData[valveThisFlowPulse].minPSI = minPressure;
        zoneData[valveThisFlowPulse].waterTemperature = readPressureSensor(READ_TEMPERATURE);
      }

      DEBUG_PRINTF("valveThisFlowPulse = %d  flowPulseCount = %d, currentPressure =  %.2f avgPressure = %.2f  maxPressure = %.2f  minPressure = %.2f  runningTotPressure = %.2f\n",
                   valveThisFlowPulse, flowPulseCount, currentPressure, avgPressure, maxPressure, minPressure, runningTotPressure);
      DEBUG_PRINTF("                  millisElapsed = %lu instantGPM = %.2f  avgGPM = %.2f  runningTotGPM = %.2f \n", millisElapsed, instantGPM, avgGPM, runningTotGPM);
    }

    millisPrev = millisStart;
    valveLastFlowPulse = valveThisFlowPulse;
  }
  else  // no flow — check for session inactivity timeout
  {
    if (sessionActive && ((millis() - millisStart) > (INACTIVITY_TIMEOUT_SECS * 1000UL)))
    {
      DEBUG_PRINTLN(F("\nINACTIVITY_TIMEOUT_SECS - irrigation session ended"));
      publishSessionReport();
      sessionActive = false;
    }
    yield();
  }
}


/*********************************
 *        SUBROUTINES
 *********************************/

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
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    DEBUG_PRINT(F("."));
    for (j = 0; j < 90; j++)     // blink LED for 90s while trying to connect
    {
      digitalWrite(BUILT_IN_LED_PIN, LOW);
      pauseTick = millis();
      while ((millis() - pauseTick) < 500);
      digitalWrite(BUILT_IN_LED_PIN, HIGH);
      pauseTick = millis();
      while ((millis() - pauseTick) < 500);
    }
    ESP.restart();
  }

  DEBUG_PRINTLN(F(""));
  DEBUG_PRINT(F("WiFi connected to "));
  DEBUG_PRINTLN(WIFI_SSID);
  DEBUG_PRINT(F("IP address: "));
  DEBUG_PRINTLN(WiFi.localIP());
}


/*
 * setup_OTA
 */
void setup_OTA()
{
  ArduinoOTA.setHostname(DEVICE_HOST_NAME);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";  // U_FS

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
  while (connectAttemptCount < MAX_MQTT_CONNECT_ATTTEMPTS)
  {
    if (!mqttClient.connected())
    {
      mqttNow = millis();
      if (mqttNow - lastReconnectAttempt > 1000)
      {
        Serial.printf("[%s] Waiting for MQTT...\n", myTZ.dateTime(RFC3339).c_str());
        lastReconnectAttempt = mqttNow;
        connectAttemptCount++;
        if (reconnect())
        {
          mqttClient.publish(IRRIG_LWT_TOPIC, "Connected", true);
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
  strncpy(mqttMsg, (char *)payload, length);
  mqttMsg[length] = (char)NULL;
  DEBUG_PRINTF("\n%s MQTT RECVD: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), topic, mqttMsg);
}


/*
 * MQTT reconnect
 */
boolean reconnect()
{
  if (mqttClient.connect(DEVICE_HOST_NAME, MQTT_USER_NAME, MQTT_PASSWORD, IRRIG_LWT_TOPIC, 2, true, "Disconnected"))
  {
    DEBUG_PRINT(F("MQTT connected to "));
    DEBUG_PRINTLN(F(MQTT_SERVER));
  }
  return mqttClient.connected();
}


/*
 * sendTotalsReport
 */
void sendTotalsReport()
{
  int i;
  unsigned int valveOffLeakGals = 0, galsAllZones = 0;

  for (i = 0; i < (TOT_NUM_VALVES + 1); i++)
  {
    // send GPM
    sprintf(mqttTopic, "%s_%d", IRRIG_GPM_TOPIC_PREFIX, i);
    sprintf(mqttMsg, "%.2f", zoneData[i].averageGPM);
    mqttClient.publish(mqttTopic, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"preMeasureGallons\": \"%d\", \"measuredZoneGallons\": \"%d\", \"maxGPM\": \"%.2f\"}",
                     zoneData[i].valveNum, zoneData[i].preMeasureGallons, zoneData[i].measuredZoneGallons, zoneData[i].maxGPM);
    sprintf(mqttTopic, "%s_%d%s", IRRIG_GPM_TOPIC_PREFIX, i, "/attributes");
    mqttClient.publish(mqttTopic, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    // send PSI & temperature
    sprintf(mqttTopic, "%s_%d", IRRIG_PSI_TOPIC_PREFIX, i);
    sprintf(mqttMsg, "%.2f", zoneData[i].averagePSI);
    mqttClient.publish(mqttTopic, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"maxPSI\": \"%.2f\", \"minPSI\": \"%.2f\", \"waterTemperature\": \"%.2f\"}",
                     zoneData[i].valveNum, zoneData[i].maxPSI, zoneData[i].minPSI, zoneData[i].waterTemperature);
    sprintf(mqttTopic, "%s_%d%s", IRRIG_PSI_TOPIC_PREFIX, i, "/attributes");
    mqttClient.publish(mqttTopic, mqttMsg, true);
    DEBUG_PRINTF("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    galsAllZones = galsAllZones + zoneData[i].preMeasureGallons + zoneData[i].measuredZoneGallons;

    if (zoneData[i].valveNum == 0)  // valNum == 0 means no valves ON, so any flow must be a leak
      valveOffLeakGals = valveOffLeakGals + zoneData[i].preMeasureGallons + zoneData[i].measuredZoneGallons;
  }

  sprintf(mqttMsg, "%d", galsAllZones);
  mqttClient.publish(IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC, mqttMsg, true);
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
 * getActiveValve
 */
int getActiveValve()                     // zero means no valve is active
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
 * readPressureSensor
 */
float readPressureSensor(int pressOrtemp)
{
  if (PRESSURE_SENSOR_INSTALLED)
  {
    sensorStatus = 0xFF; // set to non-zero for initial while() test
    pressureReadNow = millis();
    if ((unsigned long)(pressureReadNow - pressureLastRead) > pressureReadInterval)
    {
      pressureLastRead = millis();
      while (sensorStatus != 0) // continue reading until valid
      {
        Wire.requestFrom(PRESSURE_SENSOR_I2C_ADDR, 4);
        int n = Wire.available();
        if (n == 4)
        {
          sensorStatus = 0;
          uint16_t rawP;
          uint16_t rawT;

          rawP = (uint16_t)Wire.read();
          rawP <<= 8;
          rawP |= (uint16_t)Wire.read();
          rawT = (uint16_t)Wire.read();
          rawT <<= 8;
          rawT |= (uint16_t)Wire.read();

          sensorStatus = rawP >> 14;
          rawP &= 0x3FFF;

          rawT >>= 5;

          psiTminus0 = ((rawP - 1000.0) / (15000.0 - 1000.0)) * MAX_PRESSURE;
          temperature = ((rawT - 512.0) / (1075.0 - 512.0)) * 55.0;
        }
        else
        {
          lastPressErrReportNow = millis();
          if ((unsigned long)(lastPressErrReportNow - lastPressErrReport) > (unsigned long)PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS)
          {
            DEBUG_PRINTLN(F("Error reading pressure sensor"));
            lastPressErrReport = millis();
          }
          while ((millis() - lastPressErrReportNow) < pressureReadInterval)
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
      return((temperature * 9 / 5) + 32);
    else
      return(temperature);
  }
  else
    return(psiTminus0);
}
