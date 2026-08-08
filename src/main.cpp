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

#define VERSION "Ver 0.4 build 2026.08.1"

// GPIO PIN DEFINITIONS
#define BUILT_IN_LED_PIN 21                         // yellow
#define STATUS_LED_PIN 7                            // blue

#define VALVE_1_PIN 4
#define VALVE_2_PIN 3
#define VALVE_3_PIN 2
#define VALVE_4_PIN 1

#define FLOW_SENSOR_BLUE_PIN 43                       // Hunter HC100FLOW flow meter - ACTIVE LOW
#define FLOW_SENSOR_RED_PIN 44

#define I2C_SCL_PIN 6
#define I2C_SDA_PIN 5
#define I2C_BUS_FREQ_HZ 100000                       // 100kHz standard mode (M3200 supports up to 400kHz)
#define PRESSURE_SENSOR_I2C_ADDR 0x28                // TE M3200 pressure sensor

// OPERATIONAL PARAMETERS & PREFERENCES
#define FLOW_GALS_PER_PULSE  1
#define FLOW_PULSE_DEBOUNCE_MS 50                    // blank sensor after each pulse to suppress reed switch bounce (max flow 34 GPM = 1 pulse per 1765ms)
#define MIN_LEAK_GALS 2                              // minimum zone-0 gallons before issuing a leak notification
#define TOT_NUM_VALVES 4
#define PRESSURE_SENSOR_INSTALLED 1
#define PREFER_FAHRENHEIT 1
#define INACTIVITY_TIMEOUT_SECS 90                   // wait this long after last pulse before ending session - normally set to 90
#define GPM_HIST_BIN_WIDTH 0.25f                     // GPM histogram resolution
#define GPM_HIST_BINS 600                            // 0.25 GPM bins across 0-150 GPM; median is taken over the whole run
                                                     // range must exceed any real reading - samples above it clamp into the top bin and skew the median
#define HEARTBEAT_SECS 1800                          // seconds between wellness check-in publishes
#define VALVE_AC_SAMPLE_MS 25                        // valve sense window - must exceed one mains cycle (16.7ms @ 60Hz) to bridge 24VAC zero crossings
#define WIFI_DIAGNOSTICS 0                           // set to 0 to drop the boot-time AP scan and status decoding
#define MAX_PRESSURE 100                             // max rated pressure of pressure sensor
#define PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS 60000  // how often a pressure sensor error is published if error condition persists
#define PRESSURE_READ_INTERVAL_MS 0                   // minimum ms between I2C reads (0 = read every call)
#define PRESSURE_SENSOR_INVALID -99.0f               // sentinel returned when sensor is unavailable
#define M3200_STATUS_NORMAL 0                        // top 2 bits of the pressure word: valid data
#define M3200_STATUS_STALE 2                         // no new conversion since last read - benign, retry
#define M3200_STATUS_FAULT 3                         // diagnostic fault - will not clear by re-reading

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
#define IRRIG_GPM_TOPIC_PREFIX "irrig_leak/report/median_gpm_zone"  // valve/zone number is appended to the end to create the complete topic
#define IRRIG_PSI_TOPIC_PREFIX "irrig_leak/report/avg_psi_zone"  // valve/zone number is appended to the end to create the complete topic
#define IRRIG_RUN_DURATION_TOPIC_PREFIX "irrig_leak/report/run_dur_zone"  // valve/zone number is appended to the end to create the complete topic

#define IRRIG_RECV_COMMAND_TOPIC "irrig_leak/cmd/#"
#define READ_TEMPERATURE 1  // pass to readPressureSensor() to return temperature
#define READ_PRESSURE 0     // pass to readPressureSensor() to return pressure

struct ZoneSummary
{
  u_int valveNum;
  u_int measuredZoneGallons;
  float medianGPM;
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
unsigned long millisNow, millisStart = 0, millisPrev = 0, millisElapsed;
unsigned long zoneStartMs = 0;
unsigned long pressureReadNow, mqttNow, lastPressErrReportNow;
byte sensorStatus;
float psiTminus0 = 0;
float avgPressure, maxPressure, minPressure, currentPressure, temperature, runningTotPressure = 0;
unsigned int validPressureReadCount = 0;
float instantGPM = 0, zoneMedianGPM, maxGPM;
unsigned int flowPulseCount = 0;

// GPM histogram for the current zone. A median over the whole run needs unbounded storage
// if kept as samples, so bin instead: fixed 400 bytes, and unlike a ring buffer of recent
// pulses it does not quietly redefine "average GPM" as "recent GPM".
uint16_t gpmHistogram[GPM_HIST_BINS];
unsigned int gpmSampleCount = 0;

bool leakAlertSent = false;           // interim alert fires once per session

bool sessionActive = false;
bool connectedOK = false;
unsigned long lastHeartbeatMs = 0;
std::atomic<bool> webSerialPromptRequested{false};

#define PRECONNECT_LOG_BUF_SIZE 2048
static char preConnectLog[PRECONNECT_LOG_BUF_SIZE];
static size_t preConnectLogLen = 0;
static bool preConnectLogReplayed = false;


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
  digitalWrite(BUILT_IN_LED_PIN, HIGH);       // onboard LED is active LOW - HIGH = off
  pinMode(STATUS_LED_PIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_BUS_FREQ_HZ);

  digitalWrite(STATUS_LED_PIN, HIGH);       // LED on during init

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
  digitalWrite(STATUS_LED_PIN, connectedOK ? LOW : HIGH);

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

  // Publish idle state immediately on boot so HA has current data without waiting for the first heartbeat
  mqttClient.publish(IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
  LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_IDLE_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
  sendPressureSensorStatus();

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
    // Pressure/temperature are read live here so the sensor can be polled on demand with no
    // flow at all - the per-pulse log only fires when the reed switch trips, which is no use
    // for a static bench reading such as a zero-pressure check.
    float promptPSI = readPressureSensor(READ_PRESSURE);
    float promptTemp = readPressureSensor(READ_TEMPERATURE);
    LOG("irrig-leak> %s | WiFi %s %ddBm | MQTT %s | zone %s | %.2f PSI | %.2f%c\n",
      myTZ.dateTime("[Y-m-d H:i:s]").c_str(),
      WiFi.SSID().c_str(), WiFi.RSSI(),
      mqttClient.connected() ? "OK" : "LOST",
      sessionActive ? "ACTIVE" : "idle",
      promptPSI, promptTemp, PREFER_FAHRENHEIT ? 'F' : 'C');
  }

  ArduinoOTA.handle();

  // Reconnect WiFi/MQTT if dropped; LED driven at drop/connect events
  if (WiFi.status() != WL_CONNECTED)
  {
    LOG("WiFi lost, reconnecting...\n");
    if (connectedOK) { connectedOK = false; digitalWrite(STATUS_LED_PIN, HIGH); }
    WiFi.reconnect();
  }
  if (!mqttClient.connected())
  {
    if (connectedOK) { connectedOK = false; digitalWrite(STATUS_LED_PIN, HIGH); }
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
    LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_WIFI_STRENGTH_TOPIC, mqttMsg);

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
      LOG("\nZone %d detected (was %d) [pins: 1=%d 2=%d 3=%d 4=%d]\n",
          valveThisFlowPulse, valveLastFlowPulse,
          digitalRead(VALVE_1_PIN), digitalRead(VALVE_2_PIN),
          digitalRead(VALVE_3_PIN), digitalRead(VALVE_4_PIN));
      if (valveLastFlowPulse >= 0)
      {
        zoneData[valveLastFlowPulse].runDurationMs += millis() - zoneStartMs;
        zoneData[valveLastFlowPulse].medianGPM = medianGPM();   // flush outgoing zone's rate
      }
      zoneStartMs = millis();
      flowPulseCount = 1;
      resetGPMHistogram();
      // The interval into this pulse spans the valve changeover, not flow through the new
      // zone - it produced 4 GPM from a 14s gap and 94 GPM from a 637ms one. Clearing
      // millisPrev makes this pulse contribute gallons but no rate sample, exactly as the
      // first pulse of a session already does.
      millisPrev = 0;
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
      zoneMedianGPM = 0;
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
      digitalWrite(STATUS_LED_PIN, connectedOK ? HIGH : LOW);   // blink state
      while (digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW)
      {
        ArduinoOTA.handle();
        millisNow = millis();
        if (blinkActive && (millisNow - blinkStart) >= 500)
        {
          blinkActive = false;
          digitalWrite(STATUS_LED_PIN, connectedOK ? LOW : HIGH);  // return to rest
        }
        if ((millisNow - millisStart) > (INACTIVITY_TIMEOUT_SECS * 1000))  // magnet stuck on LOW
        {
          LOG("\nINACTIVITY_TIMEOUT_SECS while FLOW_SENSOR_BLUE_PIN stuck LOW\n");
          digitalWrite(STATUS_LED_PIN, connectedOK ? LOW : HIGH);  // restore rest state
          sensorStuckUntilMs = millis() + (INACTIVITY_TIMEOUT_SECS * 1000UL);
          publishSessionReport();
          sessionActive = false;
          return;
        }
        yield();
      }
      digitalWrite(STATUS_LED_PIN, connectedOK ? LOW : HIGH);    // restore rest state
    }
    sensorStuckUntilMs = millis() + FLOW_PULSE_DEBOUNCE_MS;  // suppress reed switch bounce on rising edge

    millisElapsed = (millisPrev > 0) ? (millisStart - millisPrev) : 0;

    // Gallons are recorded on EVERY pulse, unconditionally. Previously this lived inside a
    // settle-window gate, so a run shorter than the window recorded nothing at all.
    // Accumulate rather than assign flowPulseCount: if a zone is ever revisited within a
    // session, assigning would reset its total back to 1 and lose everything before it.
    zoneData[valveThisFlowPulse].measuredZoneGallons += FLOW_GALS_PER_PULSE;
    zoneData[valveThisFlowPulse].averagePSI = avgPressure;
    zoneData[valveThisFlowPulse].maxPSI = maxPressure;
    zoneData[valveThisFlowPulse].minPSI = minPressure;
    zoneData[valveThisFlowPulse].waterTemperature = readPressureSensor(READ_TEMPERATURE);

    // Rate is a separate concern from volume. The first pulse of a zone has no preceding
    // interval, so it contributes gallons but no rate sample.
    if (millisPrev > 0 && millisElapsed > 0)
    {
      instantGPM = 60000.0f / (float)millisElapsed;
      if (instantGPM > maxGPM)
        maxGPM = instantGPM;
      addGPMSample(instantGPM);
      zoneMedianGPM = medianGPM();                 // median ignores the pipe-fill spike without a window
      zoneData[valveThisFlowPulse].medianGPM = zoneMedianGPM;
      zoneData[valveThisFlowPulse].maxGPM = maxGPM;
    }


    // Flow and pressure are unrelated measurements - the reed switch counts gallons, the
    // M3200 reads PSI - so they get a line each. runningTotPressure is deliberately not
    // logged: it is only the numerator of avgPressure, and a SUM of pressures is not a
    // physical quantity (unlike gallons, PSI is intensive - averageable, not addable).
    LOG("flow:  zone = %d  gallons = %d  millisElapsed = %lu  instantGPM = %.2f  medianGPM = %.2f  maxGPM = %.2f\n",
                 valveThisFlowPulse, flowPulseCount, millisElapsed, instantGPM, zoneMedianGPM, maxGPM);
    LOG("press: currentPressure = %.2f  avgPressure = %.2f  maxPressure = %.2f  minPressure = %.2f  temperature = %.2f%c\n",
                 currentPressure, avgPressure, maxPressure, minPressure,
                 zoneData[valveThisFlowPulse].waterTemperature, PREFER_FAHRENHEIT ? 'F' : 'C');

    // A stuck-open valve passes full zone flow continuously, so the session never times out
    // and the end-of-session report never fires. Alert as soon as the volume says "leak".
    if (zoneData[0].measuredZoneGallons > MIN_LEAK_GALS && !leakAlertSent)
    {
      leakAlertSent = true;
      zoneData[0].medianGPM = medianGPM();     // flush the in-progress rate before reporting
      publishLeakTopic(true, "LEAK ALERT (mid-session) ");
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
 * LOG - printf-style logging mirrored to Serial and WebSerial; also captures output
 *       into preConnectLog until a WebSerial client attaches and it gets replayed
 */
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


/*
 * resetSessionData - clears per-irrigation-session accumulators
 */
void resetSessionData()
{
  memset(zoneData, 0, sizeof(zoneData));
  flowPulseCount = 0;
  millisElapsed = 0;
  millisPrev = 0;
  avgPressure = PRESSURE_SENSOR_INVALID;
  maxPressure = PRESSURE_SENSOR_INVALID;
  minPressure = PRESSURE_SENSOR_INVALID;
  runningTotPressure = 0;
  validPressureReadCount = 0;
  zoneMedianGPM = 0;
  instantGPM = 0;
  maxGPM = 0;
  valveLastFlowPulse = -1;
  resetGPMHistogram();
  leakAlertSent = false;
}


/*
 * resetGPMHistogram / addGPMSample / medianGPM
 *
 * The zone average is a median rather than a mean so the pipe-fill spike at the start of a
 * run is stepped over as a handful of outliers, with no settle window to tune and no
 * sensitivity to flow rate. It also shrugs off a dropped reed pulse, which doubles an
 * interval and halves that one sample - a mean would absorb that error.
 *
 * Binning instead of storing samples keeps the median over the WHOLE run at fixed cost; a
 * ring buffer of recent pulses would quietly turn "average GPM" into "recent GPM".
 */
void resetGPMHistogram()
{
  memset(gpmHistogram, 0, sizeof(gpmHistogram));
  gpmSampleCount = 0;
}


void addGPMSample(float gpm)
{
  int bin = (int)(gpm / GPM_HIST_BIN_WIDTH);
  if (bin < 0)
    bin = 0;
  if (bin >= GPM_HIST_BINS)
    bin = GPM_HIST_BINS - 1;          // clamp: anything above range lands in the top bin
  if (gpmHistogram[bin] < 0xFFFF)
    gpmHistogram[bin]++;
  gpmSampleCount++;
}


float medianGPM()
{
  if (gpmSampleCount == 0)
    return(0);

  unsigned int half = gpmSampleCount / 2;
  unsigned int running = 0;
  for (int bin = 0; bin < GPM_HIST_BINS; bin++)
  {
    running += gpmHistogram[bin];
    if (running > half)
      return((bin + 0.5f) * GPM_HIST_BIN_WIDTH);   // bin centre
  }
  return(0);
}


/*
 * publishLeakTopic - the single source of truth for valves-off flow.
 *
 * Zone 0 is not a zone, it is a fault condition, so it is reported here rather than as a
 * fifth member of the per-zone series. State carries GALLONS: volume is monotonic within a
 * session, is what the loss actually costs, and unlike a rate it cannot read zero while
 * water is still moving (a rate needs two pulses in the same zone visit to exist at all).
 * Rate goes in the attributes, where it still conveys urgency - 40 GPM is a burst, 2 GPM a
 * seep.
 *
 * Called mid-session the moment the volume crosses the threshold, because a stuck-open
 * valve flows continuously and the inactivity timeout that ends a session never fires.
 * The attribute gallons figure is ungated so sub-threshold transition noise stays visible
 * for tuning even while the state reads 0.
 */
void publishLeakTopic(bool isLeak, const char* context)
{
  sprintf(mqttMsg, "%d", isLeak ? zoneData[0].measuredZoneGallons : 0);
  mqttClient.publish(IRRIG_VALVES_OFF_LEAK_TOPIC, mqttMsg, true);
  LOG("%s %sMQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), context, IRRIG_VALVES_OFF_LEAK_TOPIC, mqttMsg);

  sprintf(mqttMsg, "{\"gallons\": \"%d\", \"medianGPM\": \"%.2f\", \"maxGPM\": \"%.2f\"}",
                   zoneData[0].measuredZoneGallons,
                   isLeak ? zoneData[0].medianGPM : 0.0f,
                   isLeak ? zoneData[0].maxGPM : 0.0f);
  sprintf(mqttTopic, "%s%s", IRRIG_VALVES_OFF_LEAK_TOPIC, "/attributes");
  mqttClient.publish(mqttTopic, mqttMsg, true);
  LOG("%s %sMQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), context, mqttTopic, mqttMsg);
}


/*
 * publishSessionReport - send end-of-session data to MQTT broker
 */
void publishSessionReport()
{
  if (valveLastFlowPulse >= 0)
  {
    zoneData[valveLastFlowPulse].runDurationMs += millis() - zoneStartMs;
    zoneData[valveLastFlowPulse].medianGPM = medianGPM();   // flush the final zone's rate
  }
  mqttClient.publish(IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
  LOG("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());
  sendTotalsReport();
  resetSessionData();
  millisStart = millis();  // reset so inactivity timeout doesn't re-fire immediately
}


/*
 * onWiFiEvent - the driver's disconnect reason code is far more specific than
 *               WiFi.status(); it names the exact stage that failed.
 *               Reason codes: 201=NO_AP_FOUND, 15=4WAY_HANDSHAKE_TIMEOUT (bad password),
 *               2=AUTH_EXPIRE, 3/8=ASSOC/STA_LEAVING, 205=CONNECTION_FAIL
 */
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  switch (event)
  {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      LOG("\n  WiFi disconnected, reason=%d\n", info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      LOG("\n  associated with AP, awaiting IP\n");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      LOG("\n  got IP\n");
      break;
    default:
      LOG("\n  wifi event %d\n", (int)event);   // catch-all: silence here hides the real failure
      break;
  }
}


/*
 * wifiStatusName - decode WiFi.status() into something readable in the log
 */
const char* wifiStatusName(int status)
{
  switch (status)
  {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL (AP not seen - antenna or range)";
    case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED (usually a bad password)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    default:                 return "UNKNOWN";
  }
}


/*
 * scanForNetworks - list visible APs so we can tell a deaf radio from a refused login.
 *                   Diagnostic only; gate off with WIFI_DIAGNOSTICS once WiFi is stable.
 */
void scanForNetworks()
{
  LOG("Scanning for networks...\n");
  int found = WiFi.scanNetworks();
  if (found <= 0)
  {
    LOG("  no APs found - check that the external U.FL antenna is attached\n");
    return;
  }

  bool targetSeen = false;
  for (int i = 0; i < found; i++)
  {
    bool isTarget = (WiFi.SSID(i) == WIFI_SSID);
    if (isTarget) targetSeen = true;
    LOG("  %2d) %-32s %4d dBm  ch%-3d %s\n", i + 1, WiFi.SSID(i).c_str(),
        WiFi.RSSI(i), WiFi.channel(i), isTarget ? "<== target" : "");
  }
  if (!targetSeen)
    LOG("  '%s' NOT among the %d visible APs\n", WIFI_SSID, found);
  WiFi.scanDelete();
}


/*
 * setup_wifi
 */
void setup_wifi()
{
  unsigned long pauseTick;

#if WIFI_DIAGNOSTICS
  WiFi.onEvent(onWiFiEvent);            // register before mode() so STA_START is not missed
#endif
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);               // don't let NVS-cached credentials shadow the ones below
  WiFi.disconnect(true, true);          // clear any stale config left in flash by factory firmware (do not delete this line)
  delay(100);
  WiFi.setHostname(DEVICE_HOST_NAME);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);   // set to maximum possible (draws 150mA)
  LOG("MAC address: %s\n", WiFi.macAddress().c_str());

#if WIFI_DIAGNOSTICS
  scanForNetworks();
#endif

  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  wl_status_t beginResult = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  LOG("WiFi.begin(\"%s\") returned %s\n", WIFI_SSID, wifiStatusName(beginResult));

  Serial.print(F("\nWaiting for WiFi "));
  pauseTick = millis();
  unsigned long lastStatusTick = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    if ((millis() - pauseTick) >= 90000)
      ESP.restart();
    Serial.print(F("."));
#if WIFI_DIAGNOSTICS
    if ((millis() - lastStatusTick) >= 5000)     // periodic so the dots stay readable
    {
      lastStatusTick = millis();
      LOG("\n  status=%s\n", wifiStatusName(WiFi.status()));
    }
#endif
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
    digitalWrite(STATUS_LED_PIN, LOW);    // LED OFF immediately on successful connect
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
  unsigned int galsAllZones = 0;

  // Zone-0 classification is by VOLUME alone. The flow meter sits upstream of the whole
  // valve manifold, so a closing valve stops flow through the meter immediately - the
  // lateral drains downstream of a closed valve and never crosses it. There is no real
  // bleed to excuse, only a gallon or two of transition noise between zones. A stuck-open
  // valve passes full zone flow continuously, so volume alone separates the two.
  // The old "was any zone 1-4 active this session" guard is deliberately gone: it dismissed
  // ALL zone-0 flow whenever a zone had run, which is exactly what hid a stuck valve.
  bool zone0IsLeak = (zoneData[0].measuredZoneGallons > MIN_LEAK_GALS);

  // A report must be a synchronized snapshot: zones that did not run this session must not
  // carry stale PSI from an earlier run. Read the sensor once here so every idle zone reports
  // the same "PSI at time of non-run" instead of last week's numbers or a fake 0.00.
  float idlePSI = readPressureSensor(READ_PRESSURE);
  float idleTemperature = readPressureSensor(READ_TEMPERATURE);

  galsAllZones = zoneData[0].measuredZoneGallons;   // zone-0 water still crossed the meter

  // Zones 1-4 only - zone 0 is reported through IRRIG_VALVES_OFF_LEAK_TOPIC instead.
  for (i = 1; i <= TOT_NUM_VALVES; i++)
  {
    bool zoneRan = (zoneData[i].measuredZoneGallons > 0);
    float reportPSI  = zoneRan ? zoneData[i].averagePSI : idlePSI;
    float reportMaxPSI = zoneRan ? zoneData[i].maxPSI : idlePSI;
    float reportMinPSI = zoneRan ? zoneData[i].minPSI : idlePSI;
    float reportTemperature = zoneRan ? zoneData[i].waterTemperature : idleTemperature;

    // send GPM
    sprintf(mqttTopic, "%s_%d", IRRIG_GPM_TOPIC_PREFIX, i);
    sprintf(mqttMsg, "%.2f", zoneData[i].medianGPM);
    mqttClient.publish(mqttTopic, mqttMsg, true);
    LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    // Report the loop index, not zoneData[i].valveNum: valveNum is only assigned for a zone
    // that actually saw flow, so idle zones would otherwise all report valve 0.
    sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"measuredZoneGallons\": \"%d\", \"maxGPM\": \"%.2f\"}",
                     i, zoneData[i].measuredZoneGallons, zoneData[i].maxGPM);
    sprintf(mqttTopic, "%s_%d%s", IRRIG_GPM_TOPIC_PREFIX, i, "/attributes");
    mqttClient.publish(mqttTopic, mqttMsg, true);
    LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    // send PSI & temperature — skip entirely if sensor was unavailable
    if (reportPSI != PRESSURE_SENSOR_INVALID)
    {
      sprintf(mqttTopic, "%s_%d", IRRIG_PSI_TOPIC_PREFIX, i);
      sprintf(mqttMsg, "%.2f", reportPSI);
      mqttClient.publish(mqttTopic, mqttMsg, true);
      LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

      if (reportTemperature != PRESSURE_SENSOR_INVALID)
        sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"maxPSI\": \"%.2f\", \"minPSI\": \"%.2f\", \"waterTemperature\": \"%.2f\"}",
                         i, reportMaxPSI, reportMinPSI, reportTemperature);
      else
        sprintf(mqttMsg, "{\"valveNum\": \"%d\", \"maxPSI\": \"%.2f\", \"minPSI\": \"%.2f\"}",
                         i, reportMaxPSI, reportMinPSI);
      sprintf(mqttTopic, "%s_%d%s", IRRIG_PSI_TOPIC_PREFIX, i, "/attributes");
      mqttClient.publish(mqttTopic, mqttMsg, true);
      LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);
    }
    else
      LOG("Skipping PSI/temp MQTT publish for zone %d — sensor unavailable\n", i);

    // send run duration
    sprintf(mqttTopic, "%s_%d", IRRIG_RUN_DURATION_TOPIC_PREFIX, i);
    sprintf(mqttMsg, "%.1f", zoneData[i].runDurationMs / 60000.0f);
    mqttClient.publish(mqttTopic, mqttMsg, true);
    LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), mqttTopic, mqttMsg);

    galsAllZones = galsAllZones + zoneData[i].measuredZoneGallons;
  }

  sprintf(mqttMsg, "%d", galsAllZones);
  mqttClient.publish(IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC, mqttMsg, true);
  LOG("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_TOTAL_GALS_ALL_ZONES_TOPIC, mqttMsg);

  publishLeakTopic(zone0IsLeak, "");
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
 * getActiveValve - zero means no valve is active
 *
 * The valves are 24VAC, so the optocoupler output is not a steady HIGH: LED current
 * passes through zero twice per mains cycle, notching the output LOW every 8.3ms (the
 * PS2505's anti-parallel LEDs cover both half-cycles, but not the zero crossings
 * themselves). A single digitalRead() can land in a notch and report "no valve", which
 * previously misfiled that gallon into zone 0 - the leak bucket. So sample across a
 * window wider than one mains cycle and treat any HIGH seen as active.
 */
int getActiveValve()
{
  unsigned long sampleStart = millis();
  do
  {
    if (digitalRead(VALVE_1_PIN) == HIGH)
      return(1);
    if (digitalRead(VALVE_2_PIN) == HIGH)
      return(2);
    if (digitalRead(VALVE_3_PIN) == HIGH)
      return(3);
    if (digitalRead(VALVE_4_PIN) == HIGH)
      return(4);
    delay(1);                            // 1ms granularity is ample against an 8.3ms notch, and yields
  } while ((millis() - sampleStart) < VALVE_AC_SAMPLE_MS);
  return(0);                             // only reached after a full window with no valve seen
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
      int retries = 0;
      while (sensorStatus != 0 && retries++ < 5) // retry up to 5x for stale status; sensor updates every ~2ms
      {
        Wire.requestFrom(PRESSURE_SENSOR_I2C_ADDR, 4);
        int n = Wire.available();
        if (n == 4)
        {
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

          // Only convert data the sensor says is good. Previously the conversion ran on every
          // pass regardless of status, so a stale or faulted reading was returned looking
          // exactly like a valid one.
          if (sensorStatus == M3200_STATUS_NORMAL)
          {
            psiTminus0 = ((rawP - 1638.0) / (14746.0 - 1638.0)) * MAX_PRESSURE;
            temperature = ((rawT - 512.0) / (1075.0 - 512.0)) * 55.0;
          }
          else if (sensorStatus == M3200_STATUS_FAULT)
          {
            lastPressErrReportNow = millis();
            if ((unsigned long)(lastPressErrReportNow - lastPressErrReport) > (unsigned long)PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS)
            {
              LOG("Pressure sensor reports diagnostic fault (status %d)\n", sensorStatus);
              lastPressErrReport = millis();
            }
            break;                    // a fault does not clear by re-reading
          }
          // M3200_STATUS_STALE means we read faster than the ~2ms conversion cycle - benign,
          // so fall through and retry rather than treating it as a failure.
        }
        else
        {
          lastPressErrReportNow = millis();
          if ((unsigned long)(lastPressErrReportNow - lastPressErrReport) > (unsigned long)PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS)
          {
            LOG("Error reading pressure sensor: got %d bytes (expected 4)\n", n);
            lastPressErrReport = millis();
          }
          psiTminus0 = PRESSURE_SENSOR_INVALID;
          temperature = PRESSURE_SENSOR_INVALID;
          while ((millis() - lastPressErrReportNow) < PRESSURE_READ_INTERVAL_MS)
            yield();
          break;
        }
      }

      // Left the loop without the sensor ever reporting good data - retries exhausted on
      // persistent stale, or a fault. Return the sentinel so the callers' existing
      // skip-publish handling applies; otherwise a bad reading reaches Home Assistant
      // looking indistinguishable from a real one. These are globals, so without this a
      // previous call's value would also leak through.
      if (sensorStatus != M3200_STATUS_NORMAL)
      {
        psiTminus0 = PRESSURE_SENSOR_INVALID;
        temperature = PRESSURE_SENSOR_INVALID;
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
