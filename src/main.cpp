/****************************
 *                          *
 * Irrigation Leak Detector *
 *                          *
 ****************************/

#include <Arduino.h>
#include <atomic>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif
#include <ArduinoOTA.h>
#include <Wire.h>
// add the below libraries from the Library Manager
#include <PubSubClient.h>
#include <ezTime.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

// local definitions
#include "prototypes.h"
#include "credentials.h"     // <<<<<<<  COMMENT THIS LINE OUT & ENTER YOUR CREDENTIALS BELOW - this contains stuff for my WIFI network, not yours

// name the device
#define DEVICE_HOST_NAME "irrig-leak"

// TIME SETTINGS
#define MY_TIMEZONE "America/New_York"               // <<<<<<< use Olson format: https://en.wikipedia.org/wiki/List_of_tz_database_time_zones

#define VERSION "Ver 0.4 build 2026.06.0"

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
#define HEARTBEAT_SECS 1800                          // seconds between wellness check-in publishes
#define MAX_PRESSURE 100                             // max rated pressure of pressure sensor
#define PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS 60000  // how often a pressure sensor error is published if error condition persists
#define PRESSURE_READ_INTERVAL_MS 0                   // minimum ms between I2C reads (0 = read every call)
#define PRESSURE_SENSOR_INVALID -99.0f               // sentinel returned when sensor is unavailable

// MQTT
#define MQTT_MSG_BUFFER_SIZE 512                            // for MQTT message payload
#define MQTT_MAX_TOPIC_SIZE 1024                            // max topic string size(can be up to 65535)
#define MAX_MQTT_CONNECT_ATTTEMPTS 10

#define IRRIG_LWT_TOPIC "irrig_leak/status/LWT"             // MQTT Last Will & Testament
#define IRRIG_VERSION_TOPIC "irrig_leak/version"            // report software version at connect
#define IRRIG_WIFI_STRENGTH_TOPIC "irrig_leak/wifi_dbm"     // signal strength of the active WiFi network
#define IRRIG_WIFI_SSID_TOPIC "irrig_leak/wifi_ssid"
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
#define READ_TEMPERATURE 1  // pass to readPressureSensor() to return temperature
#define READ_PRESSURE 0     // pass to readPressureSensor() to return pressure

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

AsyncWebServer server(80);

WiFiClient espClient;
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
unsigned int validPressureReadCount = 0;
float instantGPM = 0, runningTotGPM = 0, avgGPM, maxGPM;
unsigned int flowPulseCount = 0, flowPreMeasurePulseCount = 0;
bool once;

bool sessionActive = false;
bool connectedOK = false;
unsigned long lastHeartbeatMs = 0;
std::atomic<bool> webSerialPromptRequested{false};

#define PRECONNECT_LOG_BUF_SIZE 2048
static char preConnectLog[PRECONNECT_LOG_BUF_SIZE];
static size_t preConnectLogLen = 0;
static bool preConnectLogReplayed = false;


void LOG(const char* fmt, ...)
{
  char buf[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  WebSerial.print(buf);
  if (!preConnectLogReplayed) {
    size_t len = strnlen(buf, sizeof(buf));
    size_t space = PRECONNECT_LOG_BUF_SIZE - preConnectLogLen;
    if (len > space) len = space;
    if (len > 0) {
      memcpy(preConnectLog + preConnectLogLen, buf, len);
      preConnectLogLen += len;
    }
  }
}


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

  digitalWrite(BUILT_IN_LED_PIN, HIGH);       // LED on during init

  resetSessionData();
  millisStart = millis();

  setup_wifi();
  WebSerial.onMessage([](uint8_t* data, size_t len) {
    webSerialPromptRequested = true;
  });
  WebSerial.begin(&server);
  server.begin();
  setup_OTA();
  LOG("\n\n\nIrrigation Leak Detector %s\n", VERSION);
  LOG("Size of struct ZoneSummary = %d\n", sizeof(ZoneSummary));
  waitForSync();  // sync the time; ezTime will re-sync periodically on its own schedule
  myTZ.setLocation(F(MY_TIMEZONE));
  LOG("Got local time: %s\n", myTZ.dateTime("[H:i:s.v]").c_str());
  connectMQTT();
  connectedOK = (WiFi.status() == WL_CONNECTED && mqttClient.connected());
  digitalWrite(BUILT_IN_LED_PIN, connectedOK ? LOW : HIGH);

  mqttClient.loop();  // keep connection alive after waitForSync() blocking call
  mqttClient.publish(IRRIG_VERSION_TOPIC, VERSION, true);
  LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VERSION_TOPIC, VERSION);

  sprintf(mqttMsg, "%d", WiFi.RSSI());
  mqttClient.publish(IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg, true);
  LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg);

  mqttClient.publish(IRRIG_WIFI_SSID_TOPIC, WiFi.SSID().c_str(), true);
  LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_SSID_TOPIC, WiFi.SSID().c_str());

  // Initialize run duration topics to 0 so HA never shows Unknown on fresh boot
  for (int i = 1; i <= TOT_NUM_VALVES; i++)
  {
    sprintf(mqttTopic, "%s_%d", IRRIG_RUN_DURATION_TOPIC_PREFIX, i);
    mqttClient.publish(mqttTopic, "0", true);
    LOG("%s MQTT SENT: %s/0\n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic);
  }

  lastHeartbeatMs = millis();
  LOG("\nSetup complete. Entering main loop.\n");
  LOG("================================\n\n");
}


/***********************
 *       LOOP
 ***********************/
void loop()
{
  WebSerial.loop();

  {
    static bool wsWasConnected = false;
    bool wsNowConnected = WebSerial.getConnectionCount();
    if (!wsWasConnected && wsNowConnected && preConnectLogLen > 0) {
      const size_t chunkSize = 512;
      size_t offset = 0;
      while (offset < preConnectLogLen) {
        size_t chunk = (preConnectLogLen - offset < chunkSize) ? (preConnectLogLen - offset) : chunkSize;
        WebSerial.write((const uint8_t*)(preConnectLog + offset), chunk);
        offset += chunk;
      }
      preConnectLogLen = 0;
      preConnectLogReplayed = true;
    }
    wsWasConnected = wsNowConnected;
  }

  if (webSerialPromptRequested) {
    webSerialPromptRequested = false;
    LOG("irrig-leak> %s | WiFi %s %ddBm | MQTT %s | zone %s\n",
      myTZ.dateTime("[Y-m-d H:i:s]").c_str(),
      WiFi.SSID().c_str(), WiFi.RSSI(),
      mqttClient.connected() ? "OK" : "LOST",
      sessionActive ? "ACTIVE" : "idle");
  }

  ArduinoOTA.handle();

  // Reconnect WiFi/MQTT if dropped; LED driven at drop/connect events
  if (WiFi.status() != WL_CONNECTED)
  {
    LOG("WiFi lost, reconnecting...\n");
    if (connectedOK) { connectedOK = false; digitalWrite(BUILT_IN_LED_PIN, HIGH); }
    WiFi.reconnect();
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
    LOG("\n--- Heartbeat ---\n");
    mqttClient.publish(IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
    LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
    sprintf(mqttMsg, "%d", WiFi.RSSI());
    mqttClient.publish(IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg, true);

    mqttClient.publish(IRRIG_WIFI_SSID_TOPIC, WiFi.SSID().c_str(), true);
    LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_SSID_TOPIC, WiFi.SSID().c_str());

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
      LOG("\n--- Irrigation session started ---\n");
    }

    currentPressure = readPressureSensor(READ_PRESSURE);
    if (currentPressure != PRESSURE_SENSOR_INVALID)
    {
      if (maxPressure == PRESSURE_SENSOR_INVALID || maxPressure < currentPressure)
        maxPressure = currentPressure;
      if (minPressure == PRESSURE_SENSOR_INVALID || minPressure > currentPressure)
        minPressure = currentPressure;
    }
    valveThisFlowPulse = getActiveValve();

    if (valveThisFlowPulse != valveLastFlowPulse)  // zone has changed
    {
      LOG("\nZone %d detected (was %d)\n", valveThisFlowPulse, valveLastFlowPulse);
      if (valveLastFlowPulse >= 0)
        zoneData[valveLastFlowPulse].runDurationMs += millis() - zoneStartMs;
      zoneStartMs = millis();
      once = true;
      startSettling = millis();
      flowPulseCount = 1;
      flowPreMeasurePulseCount = 0;
      validPressureReadCount = 0;
      avgPressure = PRESSURE_SENSOR_INVALID;
      maxPressure = PRESSURE_SENSOR_INVALID;
      minPressure = PRESSURE_SENSOR_INVALID;
      runningTotPressure = 0;
      if (currentPressure != PRESSURE_SENSOR_INVALID)
      {
        avgPressure = currentPressure;
        maxPressure = currentPressure;
        minPressure = currentPressure;
        runningTotPressure = currentPressure;
        validPressureReadCount = 1;
      }
      instantGPM = 0;
      maxGPM = 0;
      avgGPM = 0;
      runningTotGPM = 0;
    }
    else
    {
      flowPulseCount++;
      if (currentPressure != PRESSURE_SENSOR_INVALID)
      {
        runningTotPressure += currentPressure;
        validPressureReadCount++;
        avgPressure = runningTotPressure / validPressureReadCount;
      }
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
          LOG("\nINACTIVITY_TIMEOUT_SECS while FLOW_SENSOR_BLUE_PIN stuck LOW\n");
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
        validPressureReadCount = 0;
        runningTotPressure = 0;
        avgPressure = PRESSURE_SENSOR_INVALID;
        if (currentPressure != PRESSURE_SENSOR_INVALID)
        {
          runningTotPressure = currentPressure;
          validPressureReadCount = 1;
          avgPressure = currentPressure;
        }
        LOG("\nFLOW MEASUREMENT STARTED, %d gallons flowed during pre-measurement settling time\n", flowPreMeasurePulseCount);
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

      LOG("valveThisFlowPulse = %d  flowPulseCount = %d, currentPressure =  %.2f avgPressure = %.2f  maxPressure = %.2f  minPressure = %.2f  runningTotPressure = %.2f\n",
                   valveThisFlowPulse, flowPulseCount, currentPressure, avgPressure, maxPressure, minPressure, runningTotPressure);
      LOG("                  millisElapsed = %lu instantGPM = %.2f  avgGPM = %.2f  runningTotGPM = %.2f \n", millisElapsed, instantGPM, avgGPM, runningTotGPM);
    }

    millisPrev = millisStart;
    valveLastFlowPulse = valveThisFlowPulse;
  }
  else  // no flow — check for session inactivity timeout
  {
    if (sessionActive && ((millis() - millisStart) > (INACTIVITY_TIMEOUT_SECS * 1000UL)))
    {
      LOG("\nINACTIVITY_TIMEOUT_SECS - irrigation session ended\n");
      publishSessionReport();
      sessionActive = false;
    }
    yield();

    {
      static int lastValvePollResult = getActiveValve();
      int polledValve = getActiveValve();
      if (polledValve != lastValvePollResult)
      {
        LOG("[valve poll] active valve = %d\n", polledValve);
        lastValvePollResult = polledValve;
      }
    }
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
  avgPressure = PRESSURE_SENSOR_INVALID;
  maxPressure = PRESSURE_SENSOR_INVALID;
  minPressure = PRESSURE_SENSOR_INVALID;
  runningTotPressure = 0;
  validPressureReadCount = 0;
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
  LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
  sendTotalsReport();
  resetSessionData();
  millisStart = millis();  // reset so inactivity timeout doesn't re-fire immediately
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

  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print(F("\nWaiting for WiFi "));
  pauseTick = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    if ((millis() - pauseTick) >= 90000)
      ESP.restart();
    Serial.print(F("."));
    delay(500);
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

    LOG("OTA Start updating %s\n", type.c_str());
  });
  ArduinoOTA.onEnd([]() {
    LOG("\nOTA End\n");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    LOG("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    LOG("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      LOG("OTA Auth Failed\n");
    else if (error == OTA_BEGIN_ERROR)
      LOG("OTA Begin Failed\n");
    else if (error == OTA_CONNECT_ERROR)
      LOG("OTA Connect Failed\n");
    else if (error == OTA_RECEIVE_ERROR)
      LOG("OTA Receive Failed\n");
    else if (error == OTA_END_ERROR)
      LOG("OTA End Failed\n");
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
        LOG("[%s] Waiting for MQTT...\n", myTZ.dateTime(RFC3339).c_str());
        lastReconnectAttempt = mqttNow;
        connectAttemptCount++;
        if (reconnect())
        {
          mqttClient.publish(IRRIG_LWT_TOPIC, "Connected", true);
          LOG("\n%s MQTT SENT: %s/Connected\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_LWT_TOPIC);
          lastReconnectAttempt = 0;
          return;
        }
      }
    }
  }
  LOG("ERROR-----Max MQTT connect attempts exceeded\n");
}


/*
 * MQTT callback
 */
void callback(char *topic, byte *payload, unsigned int length)
{
  strncpy(mqttMsg, (char *)payload, length);
  mqttMsg[length] = (char)NULL;
  LOG("\n%s MQTT RECVD: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), topic, mqttMsg);
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
    LOG("MQTT connected to %s\n", MQTT_SERVER);
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
    LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"preMeasureGallons\": \"%d\", \"measuredZoneGallons\": \"%d\", \"maxGPM\": \"%.2f\"}",
                     zoneData[i].valveNum, zoneData[i].preMeasureGallons, zoneData[i].measuredZoneGallons, zoneData[i].maxGPM);
    sprintf(mqttTopic, "%s_%d%s", IRRIG_GPM_TOPIC_PREFIX, i, "/attributes");
    mqttClient.publish(mqttTopic, mqttMsg, true);
    LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    // send PSI & temperature — skip entirely if sensor was unavailable this session
    if (zoneData[i].averagePSI != PRESSURE_SENSOR_INVALID)
    {
      sprintf(mqttTopic, "%s_%d", IRRIG_PSI_TOPIC_PREFIX, i);
      sprintf(mqttMsg, "%.2f", zoneData[i].averagePSI);
      mqttClient.publish(mqttTopic, mqttMsg, true);
      LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

      if (zoneData[i].waterTemperature != PRESSURE_SENSOR_INVALID)
        sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"maxPSI\": \"%.2f\", \"minPSI\": \"%.2f\", \"waterTemperature\": \"%.2f\"}",
                         zoneData[i].valveNum, zoneData[i].maxPSI, zoneData[i].minPSI, zoneData[i].waterTemperature);
      else
        sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"maxPSI\": \"%.2f\", \"minPSI\": \"%.2f\"}",
                         zoneData[i].valveNum, zoneData[i].maxPSI, zoneData[i].minPSI);
      sprintf(mqttTopic, "%s_%d%s", IRRIG_PSI_TOPIC_PREFIX, i, "/attributes");
      mqttClient.publish(mqttTopic, mqttMsg, true);
      LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);
    }
    else
      LOG("Skipping PSI/temp MQTT publish for zone %d — sensor unavailable\n", i);

    // send run duration (zones 1-4 only)
    if (i > 0)
    {
      sprintf(mqttTopic, "%s_%d", IRRIG_RUN_DURATION_TOPIC_PREFIX, i);
      sprintf(mqttMsg, "%.1f", zoneData[i].runDurationMs / 60000.0f);
      mqttClient.publish(mqttTopic, mqttMsg, true);
      LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);
    }

    galsAllZones = galsAllZones + zoneData[i].preMeasureGallons + zoneData[i].measuredZoneGallons;

    if (zoneData[i].valveNum == 0)  // valNum == 0 means no valves ON, so any flow must be a leak
      valveOffLeakGals = valveOffLeakGals + zoneData[i].measuredZoneGallons;
  }

  sprintf(mqttMsg, "%d", galsAllZones);
  mqttClient.publish(IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC, mqttMsg, true);
  LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC, mqttMsg);

  if (valveOffLeakGals > 0)
  {
    sprintf(mqttMsg, "%d", valveOffLeakGals);
    mqttClient.publish(IRRIG_VALVES_OFF_LEAK_TOPIC, mqttMsg, true);
    LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VALVES_OFF_LEAK_TOPIC, mqttMsg);
  }
  else
    mqttClient.publish(IRRIG_VALVES_OFF_LEAK_TOPIC, "0", true);
    LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_VALVES_OFF_LEAK_TOPIC, "0");
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
    LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_PRESSURE_TOPIC, mqttMsg);
    sprintf(mqttMsg, "%.2f", idleWaterTemperature);
    mqttClient.publish(IRRIG_IDLE_WATER_TEMPERATURE_TOPIC, mqttMsg, true);
    LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_WATER_TEMPERATURE_TOPIC, mqttMsg);
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
            LOG("Error reading pressure sensor\n");
            lastPressErrReport = millis();
          }
          psiTminus0 = PRESSURE_SENSOR_INVALID;
          temperature = PRESSURE_SENSOR_INVALID;
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
    if (temperature == PRESSURE_SENSOR_INVALID)
      return PRESSURE_SENSOR_INVALID;
    if (PREFER_FAHRENHEIT)
      return((temperature * 9 / 5) + 32);
    else
      return(temperature);
  }
  else
    return(psiTminus0);
}
