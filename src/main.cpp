/****************************
 *                          *
 * Irrigation Leak Detector *
 *                          *
 ****************************/

#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <WiFiMulti.h>
#else
  #include <ESP8266WiFi.h>
#endif
#include <ArduinoOTA.h>
#include <Wire.h>
// add the below libraries from the Library Manager
#include <PubSubClient.h>
#include <ezTime.h>

// local definitions
#include "prototypes.h"
#include "credentials.h"     // <<<<<<<  COMMENT THIS LINE OUT & ENTER YOUR CREDENTIALS BELOW - this contains stuff for my WIFI network, not yours

#define DEBUG_VALVES 1     // comment line out to undefine - setting to 0 is still considered defined

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
#define FLOW_SETTLE_SECS 35                          // wait for empty pipe to fill and settle
#define INACTIVITY_TIMEOUT_SECS 90                   // wait this long after last pulse before ending session - normally set to 90
#define HEARTBEAT_SECS 3600                          // seconds between wellness check-in publishes - normally set to 3600
#define MAX_PRESSURE 100                             // max rated pressure of pressure sensor
#define PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS 60000  // how often a pressure sensor error is published if error condition persists
#define PRESSURE_READ_INTERVAL_MS 0                   // minimum ms between I2C reads (0 = read every call)

// MQTT
#define MQTT_MSG_BUFFER_SIZE 512                            // for MQTT message payload
#define MQTT_MAX_TOPIC_SIZE 1024                            // max topic string size(can be up to 65535)
#define MAX_MQTT_CONNECT_ATTTEMPTS 10

#define IRRIG_LWT_TOPIC "irrig_leak/status/LWT"             // MQTT Last Will & Testament
#define IRRIG_VERSION_TOPIC "irrig_leak/version"            // report software version at connect
#define IRRIG_WIFI_STRENGTH_TOPIC "irrig_leak/wifi_dbm"     // signal strength of the active WiFi network
#define IRRIG_WIFI_SSID_TOPIC "irrig_leak/wifi_ssid"
#define IRRIG_WIFI_PRIMARY_DBM_TOPIC   "irrig_leak/wifi_primary_dbm"    // for ShenCentral SSID
#define IRRIG_WIFI_SECONDARY_DBM_TOPIC "irrig_leak/wifi_secondary_dbm"  // for irrig-leak SSID
#define IRRIG_IDLE_TIME_STAMP_TOPIC "irrig_leak/idle/time_stamp"
#define IRRIG_IDLE_PRESSURE_TOPIC "irrig_leak/idle/water_pressure"
#define IRRIG_IDLE_WATER_TEMPERATURE_TOPIC "irrig_leak/idle/water_temperature"
#define IRRIG_REPORT_TIME_STAMP_TOPIC "irrig_leak/report/time_stamp"
#define IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC "irrig_leak/report/tot_gals_all_zones"
#define IRRIG_VALVES_OFF_LEAK_TOPIC "irrig_leak/report/valve_leak"  // flow sensed when all valves are off & there should be none
#define IRRIG_GPM_TOPIC_PREFIX "irrig_leak/report/avg_gpm_zone"  // valve/zone number is appended to the end to create the complete topic
#define IRRIG_PSI_TOPIC_PREFIX "irrig_leak/report/avg_psi_zone"  // valve/zone number is appended to the end to create the complete topic
#define IRRIG_RUN_DURATION_TOPIC_PREFIX "irrig_leak/report/run_dur_zone"  // valve/zone number is appended to the end to create the complete topic

#define IRRIG_RECV_COMMAND_TOPIC "irrig_leak/cmd/#"
// MIS
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
  unsigned long runDurationMs;
};

WiFiClient espClient;
WiFiMulti wifiMulti;
PubSubClient mqttClient(espClient);
Timezone myTZ;

char mqttMsg[MQTT_MSG_BUFFER_SIZE];
char mqttTopic[MQTT_MAX_TOPIC_SIZE];
struct ZoneSummary zoneData[TOT_NUM_VALVES+1];           // array to keep flow data

int valveThisFlowPulse = 0, valveLastFlowPulse = -1;
unsigned long lastReconnectAttempt = 0;
unsigned long sensorStuckUntilMs = 0;
unsigned long pressureLastRead = 0, lastPressErrReport = 0;
unsigned long millisNow, millisStart = 0, millisPrev = 0, startSettling, millisElapsed;
unsigned long zoneStartMs = 0;
unsigned long pressureReadNow, mqttNow, lastPressErrReportNow;
byte sensorStatus;
float psiTminus0 = 0;
float avgPressure, maxPressure, minPressure, currentPressure, temperature, runningTotPressure = 0;
float instantGPM = 0, runningTotGPM = 0, avgGPM, maxGPM;
unsigned int flowPulseCount = 0, flowPreMeasurePulseCount = 0;
bool once;

bool sessionActive = false;
bool connectedOK = false;
unsigned long lastHeartbeatMs = 0;





/**********************
 *      SETUP
 **********************/
void setup()
{
  Serial.begin(115200);

  pinMode(VALVE_1_PIN, INPUT);                // valve inputs
  pinMode(VALVE_2_PIN, INPUT);                // are active high
  pinMode(VALVE_3_PIN, INPUT);
  pinMode(VALVE_4_PIN, INPUT);
  pinMode(FLOW_SENSOR_BLUE_PIN, INPUT);       // flow inputs are
  pinMode(FLOW_SENSOR_RED_PIN, INPUT);        // active low w external pullup
  pinMode(BUILT_IN_LED_PIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  Serial.printf("\n\n\nIrrigation Leak Detector %s\n", VERSION);
  Serial.println("Size of struct ZoneSummary = " + String(sizeof(ZoneSummary)));

  digitalWrite(BUILT_IN_LED_PIN, HIGH);       // LED on during init

  resetSessionData();
  millisStart = millis();

  setup_wifi();
  setup_OTA();
  waitForSync();  // sync the time; ezTime will re-sync periodically on its own schedule
  myTZ.setLocation(F(MY_TIMEZONE));
  Serial.printf("Got local time: %s\n", myTZ.dateTime("[H:i:s.v]").c_str());
  connectMQTT();
  connectedOK = (WiFi.status() == WL_CONNECTED && mqttClient.connected());
  digitalWrite(BUILT_IN_LED_PIN, connectedOK ? LOW : HIGH);

  mqttClient.loop();  // keep connection alive after waitForSync() blocking call
  mqttClient.publish(IRRIG_VERSION_TOPIC, VERSION, true);
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VERSION_TOPIC, VERSION);

  sprintf(mqttMsg, "%d", WiFi.RSSI());
  mqttClient.publish(IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg, true);
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg);

  mqttClient.publish(IRRIG_WIFI_SSID_TOPIC, WiFi.SSID().c_str(), true);
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_SSID_TOPIC, WiFi.SSID().c_str());

  {
    CandidateRSSI candidates = scanAndLogWifi();
    if (candidates.primary != INT8_MIN) {
      sprintf(mqttMsg, "%d", candidates.primary);
      mqttClient.publish(IRRIG_WIFI_PRIMARY_DBM_TOPIC, mqttMsg, true);
      Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_PRIMARY_DBM_TOPIC, mqttMsg);
    }
    if (candidates.secondary != INT8_MIN) {
      sprintf(mqttMsg, "%d", candidates.secondary);
      mqttClient.publish(IRRIG_WIFI_SECONDARY_DBM_TOPIC, mqttMsg, true);
      Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_SECONDARY_DBM_TOPIC, mqttMsg);
    }
  }

  // Initialize run duration topics to 0 so HA never shows Unknown on fresh boot
  for (int i = 1; i <= TOT_NUM_VALVES; i++)
  {
    sprintf(mqttTopic, "%s_%d", IRRIG_RUN_DURATION_TOPIC_PREFIX, i);
    mqttClient.publish(mqttTopic, "0", true);
    Serial.printf("%s MQTT SENT: %s/0\n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic);
  }

  lastHeartbeatMs = millis();
  Serial.println(F("\nSetup complete. Entering main loop."));
  Serial.println(F("================================\n"));
}


/***********************
 *       LOOP
 ***********************/
void loop()
{
  ArduinoOTA.handle();

  // Reconnect WiFi/MQTT if dropped; LED driven at drop/connect events
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(F("WiFi lost, reconnecting..."));
    if (connectedOK) { connectedOK = false; digitalWrite(BUILT_IN_LED_PIN, HIGH); }
    wifiMulti.run();
  }
  if (!mqttClient.connected())
  {
    if (connectedOK) { connectedOK = false; digitalWrite(BUILT_IN_LED_PIN, HIGH); }
    connectMQTT();   // reconnect() inside sets connectedOK=true and LED LOW on success
  }
  mqttClient.loop();

  // Periodic heartbeat
  if ((unsigned long)(millis() - lastHeartbeatMs) >= (HEARTBEAT_SECS * 1000UL))
  {
    lastHeartbeatMs = millis();
    Serial.println(F("\n--- Heartbeat ---"));
    mqttClient.publish(IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
    Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
    sprintf(mqttMsg, "%d", WiFi.RSSI());
    mqttClient.publish(IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg, true);

    mqttClient.publish(IRRIG_WIFI_SSID_TOPIC, WiFi.SSID().c_str(), true);
    Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_SSID_TOPIC, WiFi.SSID().c_str());

    {
      CandidateRSSI candidates = scanAndLogWifi();
      if (candidates.primary != INT8_MIN) {
        sprintf(mqttMsg, "%d", candidates.primary);
        mqttClient.publish(IRRIG_WIFI_PRIMARY_DBM_TOPIC, mqttMsg, true);
        Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_PRIMARY_DBM_TOPIC, mqttMsg);
      }
      if (candidates.secondary != INT8_MIN) {
        sprintf(mqttMsg, "%d", candidates.secondary);
        mqttClient.publish(IRRIG_WIFI_SECONDARY_DBM_TOPIC, mqttMsg, true);
        Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_SECONDARY_DBM_TOPIC, mqttMsg);
      }
    }

    sendPressureSensorStatus();
  }

  // Flow detection
  if (digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW && millis() >= sensorStuckUntilMs)   // pulse is active when LOW
  {
    if (!sessionActive)
    {
      sessionActive = true;
      resetSessionData();
      zoneStartMs = millis();
      startSettling = millis();
      Serial.println(F("\n--- Irrigation session started ---"));
    }

    currentPressure = readPressureSensor(READ_PRESSURE);
    if (maxPressure < currentPressure)
      maxPressure = currentPressure;
    if (minPressure > currentPressure)
      minPressure = currentPressure;
    valveThisFlowPulse = getActiveValve();

    if (valveThisFlowPulse != valveLastFlowPulse)  // zone has changed
    {
      Serial.printf("\nZone %d detected (was %d)\n", valveThisFlowPulse, valveLastFlowPulse);
      if (valveLastFlowPulse >= 0)
        zoneData[valveLastFlowPulse].runDurationMs += millis() - zoneStartMs;
      zoneStartMs = millis();
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

    {
      unsigned long blinkStart = millis();
      bool blinkActive = true;
      digitalWrite(BUILT_IN_LED_PIN, connectedOK ? HIGH : LOW);   // blink state
      while (digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW)
      {
        ArduinoOTA.handle();
        millisNow = millis();
        if (blinkActive && (millisNow - blinkStart) >= 500)
        {
          blinkActive = false;
          digitalWrite(BUILT_IN_LED_PIN, connectedOK ? LOW : HIGH);  // return to rest
        }
        if ((millisNow - millisStart) > (INACTIVITY_TIMEOUT_SECS * 1000))  // magnet stuck on LOW
        {
          Serial.println(F("\nINACTIVITY_TIMEOUT_SECS while FLOW_SENSOR_BLUE_PIN stuck LOW"));
          digitalWrite(BUILT_IN_LED_PIN, connectedOK ? LOW : HIGH);  // restore rest state
          sensorStuckUntilMs = millis() + (INACTIVITY_TIMEOUT_SECS * 1000UL);
          publishSessionReport();
          sessionActive = false;
          return;
        }
        yield();
      }
      digitalWrite(BUILT_IN_LED_PIN, connectedOK ? LOW : HIGH);    // restore rest state
    }

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
        Serial.printf("\nFLOW MEASUREMENT STARTED, %d gallons flowed during pre-measurement settling time\n", flowPreMeasurePulseCount);
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

      Serial.printf("valveThisFlowPulse = %d  flowPulseCount = %d, currentPressure =  %.2f avgPressure = %.2f  maxPressure = %.2f  minPressure = %.2f  runningTotPressure = %.2f\n",
                   valveThisFlowPulse, flowPulseCount, currentPressure, avgPressure, maxPressure, minPressure, runningTotPressure);
      Serial.printf("                  millisElapsed = %lu instantGPM = %.2f  avgGPM = %.2f  runningTotGPM = %.2f \n", millisElapsed, instantGPM, avgGPM, runningTotGPM);
    }

    millisPrev = millisStart;
    valveLastFlowPulse = valveThisFlowPulse;
  }
  else  // no flow — check for session inactivity timeout
  {
    if (sessionActive && ((millis() - millisStart) > (INACTIVITY_TIMEOUT_SECS * 1000UL)))
    {
      Serial.println(F("\nINACTIVITY_TIMEOUT_SECS - irrigation session ended"));
      publishSessionReport();
      sessionActive = false;
    }
    yield();

#ifdef DEBUG_VALVES
    {
      static unsigned long lastValvePollPrintMs = 0;
      static int lastValvePollResult = -99;
      int polledValve = getActiveValve();
      if (polledValve != lastValvePollResult || (millis() - lastValvePollPrintMs) >= 5000)
      {
        Serial.printf("[valve poll] active valve = %d\n", polledValve);
        lastValvePollResult = polledValve;
        lastValvePollPrintMs = millis();
      }
    }
#endif
  }
}


/*********************************
 *        SUBROUTINES
 *********************************/

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
  if (valveLastFlowPulse >= 0)
    zoneData[valveLastFlowPulse].runDurationMs += millis() - zoneStartMs;
  mqttClient.publish(IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
  sendTotalsReport();
  resetSessionData();
  millisStart = millis();  // reset so inactivity timeout doesn't re-fire immediately
}


/*
 * scanAndLogWifi - scan visible APs, log all to serial, return RSSI for each configured candidate
 */
CandidateRSSI scanAndLogWifi()
{
  CandidateRSSI result = { INT8_MIN, INT8_MIN };

  int n = WiFi.scanNetworks(false, true);
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == PRIMARY_WIFI_SSID && WiFi.RSSI(i) > result.primary)
      result.primary = (int8_t)WiFi.RSSI(i);
  }
  WiFi.scanDelete();

  // Hidden SSIDs return "" in broadcast scan; directed probe makes them respond with SSID populated
  int m = WiFi.scanNetworks(false, true, false, 300, 0, SECONDARY_WIFI_SSID);
  for (int i = 0; i < m; i++) {
    if (WiFi.SSID(i) == SECONDARY_WIFI_SSID && WiFi.RSSI(i) > result.secondary)
      result.secondary = (int8_t)WiFi.RSSI(i);
  }
  WiFi.scanDelete();

  // Synchronous scan can temporarily disconnect; yield until reconnected (or 5s timeout)
  if (WiFi.status() != WL_CONNECTED)
  {
    unsigned long recoveryStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - recoveryStart) < 5000)
      yield();
  }

  return result;
}


/*
 * setup_wifi
 */
void setup_wifi()
{
  unsigned long pauseTick;

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOST_NAME);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);   // set to maximum possible (draws 150mA)
  Serial.println(WiFi.macAddress());

  wifiMulti.addAP(PRIMARY_WIFI_SSID,   PRIMARY_WIFI_PASSWORD);
  wifiMulti.addAP(SECONDARY_WIFI_SSID, SECONDARY_WIFI_PASSWORD);

  scanAndLogWifi();   // serial-only; MQTT not yet connected

  Serial.print(F("\nWaiting for WiFi "));
  while (wifiMulti.run() != WL_CONNECTED)
  {
    Serial.print(F("."));
    pauseTick = millis();
    while ((millis() - pauseTick) < 90000)   // wait 90s solid-ON then restart
      yield();
    ESP.restart();
  }

  Serial.println(F(""));
  Serial.print(F("WiFi connected to "));
  Serial.println(WiFi.SSID());
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
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

    Serial.println("OTA Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("\nOTA End"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println(F("OTA Auth Failed"));
    else if (error == OTA_BEGIN_ERROR)
      Serial.println(F("OTA Begin Failed"));
    else if (error == OTA_CONNECT_ERROR)
      Serial.println(F("OTA Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println(F("OTA Receive Failed"));
    else if (error == OTA_END_ERROR)
      Serial.println(F("OTA End Failed"));
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
    ArduinoOTA.handle();
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
          Serial.printf("\n%s MQTT SENT: %s/Connected\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_LWT_TOPIC);
          lastReconnectAttempt = 0;
          return;
        }
      }
    }
  }
  Serial.println("ERROR-----Max MQTT connect attempts exceeded");
}


/*
 * MQTT callback
 */
void callback(char *topic, byte *payload, unsigned int length)
{
  strncpy(mqttMsg, (char *)payload, length);
  mqttMsg[length] = (char)NULL;
  Serial.printf("\n%s MQTT RECVD: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), topic, mqttMsg);
}


/*
 * MQTT reconnect
 */
boolean reconnect()
{
  if (mqttClient.connect(DEVICE_HOST_NAME, MQTT_USER_NAME, MQTT_PASSWORD, IRRIG_LWT_TOPIC, 2, true, "Disconnected"))
  {
    connectedOK = true;
    digitalWrite(BUILT_IN_LED_PIN, LOW);    // LED OFF immediately on successful connect
    Serial.print(F("MQTT connected to "));
    Serial.println(F(MQTT_SERVER));
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
    Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"preMeasureGallons\": \"%d\", \"measuredZoneGallons\": \"%d\", \"maxGPM\": \"%.2f\"}",
                     zoneData[i].valveNum, zoneData[i].preMeasureGallons, zoneData[i].measuredZoneGallons, zoneData[i].maxGPM);
    sprintf(mqttTopic, "%s_%d%s", IRRIG_GPM_TOPIC_PREFIX, i, "/attributes");
    mqttClient.publish(mqttTopic, mqttMsg, true);
    Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    // send PSI & temperature
    sprintf(mqttTopic, "%s_%d", IRRIG_PSI_TOPIC_PREFIX, i);
    sprintf(mqttMsg, "%.2f", zoneData[i].averagePSI);
    mqttClient.publish(mqttTopic, mqttMsg, true);
    Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"maxPSI\": \"%.2f\", \"minPSI\": \"%.2f\", \"waterTemperature\": \"%.2f\"}",
                     zoneData[i].valveNum, zoneData[i].maxPSI, zoneData[i].minPSI, zoneData[i].waterTemperature);
    sprintf(mqttTopic, "%s_%d%s", IRRIG_PSI_TOPIC_PREFIX, i, "/attributes");
    mqttClient.publish(mqttTopic, mqttMsg, true);
    Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    // send run duration (zones 1-4 only)
    if (i > 0)
    {
      sprintf(mqttTopic, "%s_%d", IRRIG_RUN_DURATION_TOPIC_PREFIX, i);
      sprintf(mqttMsg, "%.1f", zoneData[i].runDurationMs / 60000.0f);
      mqttClient.publish(mqttTopic, mqttMsg, true);
      Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);
    }

    galsAllZones = galsAllZones + zoneData[i].preMeasureGallons + zoneData[i].measuredZoneGallons;

    if (zoneData[i].valveNum == 0)  // valNum == 0 means no valves ON, so any flow must be a leak
      valveOffLeakGals = valveOffLeakGals + zoneData[i].measuredZoneGallons;
  }

  sprintf(mqttMsg, "%d", galsAllZones);
  mqttClient.publish(IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC, mqttMsg, true);
  Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC, mqttMsg);

  if (valveOffLeakGals > 0)
  {
    sprintf(mqttMsg, "%d", valveOffLeakGals);
    mqttClient.publish(IRRIG_VALVES_OFF_LEAK_TOPIC, mqttMsg, true);
    Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VALVES_OFF_LEAK_TOPIC, mqttMsg);
  }
  else
    mqttClient.publish(IRRIG_VALVES_OFF_LEAK_TOPIC, "0", true);
    Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VALVES_OFF_LEAK_TOPIC, "0");
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
    Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_PRESSURE_TOPIC, mqttMsg);
    sprintf(mqttMsg, "%.2f", idleWaterTemperature);
    mqttClient.publish(IRRIG_IDLE_WATER_TEMPERATURE_TOPIC, mqttMsg, true);
    Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_WATER_TEMPERATURE_TOPIC, mqttMsg);
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
    if ((unsigned long)(pressureReadNow - pressureLastRead) > PRESSURE_READ_INTERVAL_MS)
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
            Serial.println(F("Error reading pressure sensor"));
            lastPressErrReport = millis();
          }
          while ((millis() - lastPressErrReportNow) < PRESSURE_READ_INTERVAL_MS)
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
