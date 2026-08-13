/****************************
 *                          *
 * Irrigation Leak Detector *
 *                          *
 ****************************/

#include <Arduino.h>
#include <atomic>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
// add the below libraries from the Library Manager
#include <PubSubClient.h>
#include <ezTime.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

// local definitions
#include "prototypes.h"
#include "credentials.h"     // <<<<<<< not in the repo - supply WIFI_SSID, WIFI_PASSWORD, MQTT_SERVER, MQTT_USER_NAME, MQTT_PASSWORD

// name the device
#define DEVICE_HOST_NAME "irrig-leak"

// TIME SETTINGS
#define MY_TIMEZONE "America/New_York"               // <<<<<<< use Olson format: https://en.wikipedia.org/wiki/List_of_tz_database_time_zones

#define VERSION "Ver 0.4 build 2026.08.1"

// GPIO PIN DEFINITIONS
#define BUILT_IN_LED_PIN 21                         // yellow - link status, ACTIVE LOW
#define STATUS_LED_PIN 7                            // blue - flow/report activity, ACTIVE HIGH

#define VALVE_1_PIN 1
#define VALVE_2_PIN 2
#define VALVE_3_PIN 3
#define VALVE_4_PIN 4

#define FLOW_SENSOR_BLUE_PIN 43                       // Hunter HC100FLOW flow meter - ACTIVE LOW, 1 pulse/gallon
#define FLOW_SENSOR_RED_PIN 44                        // second meter lead - wired and pulled up, not read by the firmware

#define I2C_SCL_PIN 6
#define I2C_SDA_PIN 5
#define I2C_BUS_FREQ_HZ 100000                       // 100kHz standard mode (M3200 supports up to 400kHz)
#define PRESSURE_SENSOR_I2C_ADDR 0x28                // TE M3200 pressure sensor

// OPERATIONAL PARAMETERS & PREFERENCES
#define FLOW_GALS_PER_PULSE  1                       // HC-100 rated to 34 GPM, so pulses arrive no faster than 1765ms apart
                                                     // debounce is done in hardware by an LS18-P, not here
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
#define VALVE_POLL_INTERVAL_MS 1000                  // idle valve-change logging; keeps VALVE_AC_SAMPLE_MS out of the hot loop

// LED TIMING - yellow shows link state, blue shows activity; the two are fully independent
#define LED_BLINK_FAST_MS 100                        // yellow while connecting to WiFi; also the blue report burst rate
#define LED_BLINK_SLOW_MS 500                        // yellow while WiFi is up but MQTT is down
#define FLOW_PULSE_LED_MS 500                        // blue dwell per flow pulse - fixed, NOT tied to reed closure length (see updateLEDs)
#define REPORT_BLINK_MS 5000                         // blue burst duration when a session report is published

#define WIFI_DIAGNOSTICS 0                           // set to 0 to drop the boot-time AP scan and status decoding
#define MAX_PRESSURE 100                             // max rated pressure of pressure sensor
#define PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS 60000  // how often a pressure sensor error is logged if the error condition persists
#define PRESSURE_READ_INTERVAL_MS 0                  // minimum ms between I2C reads (0 = read every call)
#define PRESSURE_SENSOR_INVALID -99.0f               // sentinel returned when sensor is unavailable
#define M3200_STATUS_NORMAL 0                        // top 2 bits of the pressure word: valid data
#define M3200_STATUS_STALE 2                         // no new conversion since last read - benign, retry
#define M3200_STATUS_FAULT 3                         // diagnostic fault - will not clear by re-reading

// MQTT
#define MQTT_MSG_BUFFER_SIZE 512                            // for MQTT message payload
#define MQTT_MAX_TOPIC_SIZE 128                             // longest topic built below is ~46 chars
#define MAX_MQTT_CONNECT_ATTTEMPTS 10                       // boot path only - see connectMQTT()
#define MQTT_SOCKET_TIMEOUT_SECS 3                          // caps ONE connect attempt; PubSubClient defaults to 15
#define LINK_RETRY_IDLE_MS 5000                             // min gap between runtime reconnect attempts when no session is running
#define LINK_RETRY_SESSION_MS 60000                         // ...and while measuring, where a blocking attempt costs gallons

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

#define READ_TEMPERATURE 1  // pass to readPressureSensor() to return temperature
#define READ_PRESSURE 0     // pass to readPressureSensor() to return pressure

struct ZoneSummary
{
  unsigned int valveNum;
  unsigned int measuredZoneGallons;
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
unsigned long lastLinkRetryMs = 0;         // rate limit for the runtime reconnect path in loop()
bool flowPinWasLow = false;                // previous flow input level, for falling-edge detection
unsigned long lastValvePollMs = 0;         // rate limit for the idle valve-change diagnostic
unsigned long pressureLastRead = 0, lastPressErrReport = 0;
unsigned long millisStart = 0, millisPrev = 0;
unsigned long zoneStartMs = 0;

// The last good reading from the M3200. Both are filled from a single 4-byte I2C transaction
// but readPressureSensor() can only return one of them, so the other has to survive the call.
float sensorPSI = 0;
float sensorTempC = 0;

float avgPressure, maxPressure, minPressure, currentPressure, runningTotPressure = 0;
unsigned int validPressureReadCount = 0;
float instantGPM = 0, zoneMedianGPM, maxGPM;

// GPM histogram for the current zone. A median over the whole run needs unbounded storage
// if kept as samples, so bin instead: fixed 1200 bytes, and unlike a ring buffer of recent
// pulses it does not quietly redefine "average GPM" as "recent GPM".
uint16_t gpmHistogram[GPM_HIST_BINS];
unsigned int gpmSampleCount = 0;

bool leakAlertSent = false;           // interim alert fires once per session

bool sessionActive = false;
unsigned long lastHeartbeatMs = 0;

// LED state. Both blue deadlines are absolute millis() values, compared as a signed
// difference against 0 so they stay correct across the 49-day millis() wrap.
unsigned long flowLedOffMs = 0;            // 0 = blue idle
unsigned long reportBlinkUntilMs = 0;      // 0 = not reporting; outranks the flow one-shot
unsigned long lastBlueToggleMs = 0;
bool blueLit = false;
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
  digitalWrite(STATUS_LED_PIN, LOW);          // blue is activity-only - dark until flow or a report

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_BUS_FREQ_HZ);

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
  waitForSync();  // sync the time; ezTime will re-sync periodically on its own schedule
  myTZ.setLocation(F(MY_TIMEZONE));
  LOG("Got local time: %s\n", myTZ.dateTime("[H:i:s.v]").c_str());
  connectMQTT();
  updateLEDs();

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
  updateLEDs();

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
    // Flow pin level is reported here rather than logged periodically: a magnet parked on the
    // reed holds it LOW, which is normal and would only spam the log if announced on a timer.
    LOG("irrig-leak> %s | WiFi %s %ddBm | MQTT %s | zone %s | flow pin %s | %.2f PSI | %.2f%c\n",
      myTZ.dateTime("[Y-m-d H:i:s]").c_str(),
      WiFi.SSID().c_str(), WiFi.RSSI(),
      mqttClient.connected() ? "OK" : "LOST",
      sessionActive ? "ACTIVE" : "idle",
      digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW ? "LOW (closed)" : "HIGH (open)",
      promptPSI, promptTemp, PREFER_FAHRENHEIT ? 'F' : 'C');
  }

  ArduinoOTA.handle();

  // Reconnect WiFi/MQTT if dropped. The yellow LED needs no help here - updateLEDs() reads
  // the live link state every pass, so it drops to blinking on its own.
  //
  // This path is deliberately stingy, because loop() also polls the flow sensor. calling
  // connectMQTT() here used to block for up to MAX_MQTT_CONNECT_ATTTEMPTS x the socket
  // timeout - over two minutes at PubSubClient's 15s default. That is longer than
  // INACTIVITY_TIMEOUT_SECS, so a single outage could both miss flow pulses AND time out a
  // live session mid-irrigation, fragmenting one run into several reports. Three guards:
  //   1. rate limit, so a sustained outage does not retry (or log) every pass
  //   2. else-if, so the broker is never chased over a dead network
  //   3. ONE attempt per pass via mqttAttemptConnect(), bounded by MQTT_SOCKET_TIMEOUT_SECS
  //
  // Guard 1 backs off hard WHILE MEASURING. MQTT_SOCKET_TIMEOUT_SECS still exceeds the 1765ms
  // minimum pulse interval at the meter's rated 34 GPM, so every attempt made during a run can
  // cost a gallon. Retrying once a minute instead of every 5s keeps that under ~5% of the time
  // rather than most of it. The reports are not urgent - they only have to arrive eventually -
  // whereas a missed gallon is gone for good. Only MQTT needs this: WiFi.reconnect() does not
  // block, so it stays prompt either way.
  if (WiFi.status() != WL_CONNECTED)
  {
    if ((unsigned long)(millis() - lastLinkRetryMs) >= LINK_RETRY_IDLE_MS)
    {
      lastLinkRetryMs = millis();
      LOG("WiFi lost, reconnecting...\n");
      WiFi.reconnect();
    }
  }
  else if (!mqttClient.connected())
  {
    if ((unsigned long)(millis() - lastLinkRetryMs) >=
        (sessionActive ? LINK_RETRY_SESSION_MS : LINK_RETRY_IDLE_MS))
    {
      lastLinkRetryMs = millis();
      LOG("MQTT lost, reconnecting...\n");
      mqttAttemptConnect();
    }
  }
  mqttClient.loop();

  // Periodic heartbeat - IDLE ONLY.
  //
  // Suppressed while a session is running because these topics are named "idle" and Home
  // Assistant treats them as a between-runs baseline. A heartbeat landing mid-irrigation
  // published that zone's operating pressure to irrig_leak/idle/water_pressure, which is a
  // real reading of the wrong thing - observed 2026-08-13, 49.06 PSI captured during zone 4
  // against a ~61 PSI idle line.
  //
  // lastHeartbeatMs is deliberately NOT advanced while suppressed, so a heartbeat that came
  // due mid-session fires as soon as the session ends. That is wanted: it yields a genuine
  // post-irrigation idle baseline rather than skipping the interval outright.
  if (!sessionActive && (unsigned long)(millis() - lastHeartbeatMs) >= (HEARTBEAT_SECS * 1000UL))
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

  // Flow detection - EDGE triggered, on the falling edge only.
  //
  // When flow stops the impeller coasts to a halt, and the magnet can come to rest parked on
  // the reed switch, holding this input LOW indefinitely. It does not happen every time - where
  // the magnet stops is chance - but over enough runs it is bound to, so it is a NORMAL resting
  // state of a healthy sensor rather than a fault to detect.
  //
  // Level triggering cannot express that. The previous version blocked in a wait-for-release
  // loop, so a parked magnet stalled the loop for INACTIVITY_TIMEOUT_SECS and then published an
  // all-zero session report (it returned before gallons were ever recorded), repeating every
  // ~180s and overwriting good retained data in Home Assistant with zeros.
  //
  // Counting the falling edge instead makes a parked magnet a non-event: no new edge means no
  // new pulse, the session ends normally through the inactivity timeout below, and the report
  // carries the real gallons measured before the magnet stopped. One edge counts exactly one
  // gallon however long the closure lasts.
  //
  // There is NO software debounce: an LS18-P debounce IC conditions the reed switch on the
  // carrier PCB, so this input is already clean. If phantom gallons ever appear, that is a
  // hardware signal-integrity problem to fix at the source, not something to paper over here.
  //
  // Timing margin: the HC-100 gives 1 pulse per gallon and is rated to 34 GPM, so pulses can
  // arrive no faster than 60/34 = 1765ms apart. loop() runs in a few ms while a valve is
  // energised, which is ~500x margin on sampling the input often enough to see every edge.
  bool flowPinLow = (digitalRead(FLOW_SENSOR_BLUE_PIN) == LOW);
  bool flowPulseEdge = (flowPinLow && !flowPinWasLow);
  flowPinWasLow = flowPinLow;

  if (flowPulseEdge)
  {
    if (!sessionActive)
    {
      sessionActive = true;
      resetSessionData();
      zoneStartMs = millis();
      LOG("\n--- Irrigation session started ---\n");
    }

    digitalWrite(STATUS_LED_PIN, HIGH);                   // blue on for this pulse
    flowLedOffMs = millis() + FLOW_PULSE_LED_MS;          // updateLEDs() turns it back off

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
    else if (currentPressure != PRESSURE_SENSOR_INVALID)
    {
      runningTotPressure += currentPressure;
      validPressureReadCount++;
      avgPressure = runningTotPressure / validPressureReadCount;
    }

    millisStart = millis();

    unsigned long millisElapsed = (millisPrev > 0) ? (millisStart - millisPrev) : 0;

    // Gallons are recorded on EVERY pulse, unconditionally. Previously this lived inside a
    // settle-window gate, so a run shorter than the window recorded nothing at all.
    // Accumulate rather than assign a per-visit counter: if a zone is revisited within a
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
    // Gallons come from the zone accumulator, not a per-visit pulse count: the old counter
    // restarted at 1 on every zone change, so a revisited zone under-reported in the log.
    LOG("flow:  zone = %d  gallons = %d  millisElapsed = %lu  instantGPM = %.2f  medianGPM = %.2f  maxGPM = %.2f\n",
                 valveThisFlowPulse, zoneData[valveThisFlowPulse].measuredZoneGallons,
                 millisElapsed, instantGPM, zoneMedianGPM, maxGPM);
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
  else  // no new pulse this pass - the input may be HIGH, or LOW with the magnet parked on it
  {
    if (sessionActive && ((millis() - millisStart) > (INACTIVITY_TIMEOUT_SECS * 1000UL)))
    {
      LOG("\nINACTIVITY_TIMEOUT_SECS - irrigation session ended\n");
      publishSessionReport();
      sessionActive = false;
    }
    yield();

    // Diagnostic only: log valve changes seen with no flow. Rate limited because
    // getActiveValve() costs a full VALVE_AC_SAMPLE_MS whenever no valve is energised, and
    // that is exactly when it runs. Flow detection above samples the input once per loop pass,
    // so a slow loop can step over a short reed closure entirely - and the no-valve case is
    // zone 0, the one that matters most. Polling this on a timer keeps the idle loop fast.
    if ((unsigned long)(millis() - lastValvePollMs) >= VALVE_POLL_INTERVAL_MS)
    {
      static int lastValvePollResult = -1;
      lastValvePollMs = millis();
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
 * updateLEDs - drives both LEDs. Non-blocking and idempotent, so it is safe to call at any
 *              rate; it must be called from every blocking wait or the LEDs freeze there.
 *
 *   yellow (BUILT_IN_LED_PIN, active LOW)  fast blink = connecting to WiFi
 *                                          slow blink = WiFi up, MQTT down
 *                                          solid      = both up
 *   blue   (STATUS_LED_PIN, active HIGH)   fixed 500ms per flow pulse, rapid burst while a
 *                                          session report is published, dark otherwise
 *
 * The blue flow blip is a one-shot deliberately DECOUPLED from the reed switch closure. Tying
 * it to the closure makes the LED's visibility depend on the meter's magnet geometry, and a
 * brief contact would produce a flash too short to catch. 500ms is always legible.
 *
 * The yellow state is derived live from WiFi/MQTT on every call rather than cached in a flag.
 * The old connectedOK flag was updated only at drop/connect events, so any state change the
 * event handlers missed left the LED lying until the next one.
 */
void updateLEDs()
{
  static unsigned long lastLinkToggleMs = 0;
  static bool linkLit = false;
  unsigned long now = millis();

  // --- yellow: link state ---
  unsigned long period;
  if (WiFi.status() != WL_CONNECTED)  period = LED_BLINK_FAST_MS;
  else if (!mqttClient.connected())   period = LED_BLINK_SLOW_MS;
  else                                period = 0;                // solid

  if (period == 0)
  {
    if (!linkLit) { linkLit = true; digitalWrite(BUILT_IN_LED_PIN, LOW); }   // active LOW
  }
  else if ((unsigned long)(now - lastLinkToggleMs) >= period)
  {
    lastLinkToggleMs = now;
    linkLit = !linkLit;
    digitalWrite(BUILT_IN_LED_PIN, linkLit ? LOW : HIGH);
  }

  // --- blue: the report burst outranks the flow one-shot ---
  if (reportBlinkUntilMs)
  {
    if ((long)(now - reportBlinkUntilMs) >= 0)
    {
      reportBlinkUntilMs = 0;
      flowLedOffMs = 0;              // drop any pulse latch stranded by the burst
      blueLit = false;
      digitalWrite(STATUS_LED_PIN, LOW);
    }
    else if ((unsigned long)(now - lastBlueToggleMs) >= LED_BLINK_FAST_MS)
    {
      lastBlueToggleMs = now;
      blueLit = !blueLit;
      digitalWrite(STATUS_LED_PIN, blueLit ? HIGH : LOW);        // active HIGH
    }
  }
  else if (flowLedOffMs && (long)(now - flowLedOffMs) >= 0)
  {
    digitalWrite(STATUS_LED_PIN, LOW);
    flowLedOffMs = 0;
  }
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
  // Announce the report on the blue LED. The publishes below are synchronous and finish in
  // milliseconds, and updateLEDs() does not run during them, so this is a fixed-duration
  // burst that plays out in loop() afterward - not a progress indicator.
  reportBlinkUntilMs = millis() + REPORT_BLINK_MS;
  lastBlueToggleMs = 0;                     // start the burst on a clean edge
  blueLit = false;

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
  unsigned long lastDotTick = 0;               // 0 so the first dot prints immediately
#if WIFI_DIAGNOSTICS
  unsigned long lastStatusTick = millis();
#endif
  while (WiFi.status() != WL_CONNECTED)
  {
    if ((millis() - pauseTick) >= 90000)
      ESP.restart();

    // Polled rather than delay(500): a 100ms blink cannot happen inside a half-second delay.
    // The dot cadence and the 90s restart watchdog above are unchanged.
    if (lastDotTick == 0 || (millis() - lastDotTick) >= 500)
    {
      lastDotTick = millis();
      Serial.print(F("."));
#if WIFI_DIAGNOSTICS
      if ((millis() - lastStatusTick) >= 5000)     // periodic so the dots stay readable
      {
        lastStatusTick = millis();
        LOG("\n  status=%s\n", wifiStatusName(WiFi.status()));
      }
#endif
    }
    updateLEDs();
    delay(10);
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
 * mqttAttemptConnect - exactly ONE connect attempt, announcing success on the LWT topic.
 *
 * Shared by the boot path and the loop path so the two cannot disagree about LWT state: the
 * broker's Last Will publishes "Disconnected" on an ungraceful drop, so whoever reconnects
 * must publish "Connected" or Home Assistant is left believing the device is still offline.
 *
 * Blocking time is bounded by MQTT_SOCKET_TIMEOUT_SECS, which is what makes this safe to call
 * from loop() where connectMQTT() was not.
 */
bool mqttAttemptConnect()
{
  if (!reconnect())
    return(false);

  mqttClient.publish(IRRIG_LWT_TOPIC, "Connected", true);
  LOG("\n%s MQTT SENT: %s/Connected\n", myTZ.dateTime("[H:i:s.v]").c_str(), IRRIG_LWT_TOPIC);
  return(true);
}


/*
 * connectMQTT - the BOOT path: keep trying until connected or attempts are exhausted.
 *
 * Blocking is acceptable here because nothing else is happening yet. Do NOT call this from
 * loop() - see the reconnect block there for why. It also performs the one-time client
 * configuration that mqttAttemptConnect() relies on, which is safe because setup() always
 * calls it before loop() runs.
 */
void connectMQTT()
{
  int connectAttemptCount = 0;
  mqttClient.setBufferSize(MQTT_MSG_BUFFER_SIZE);
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(callback);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SECS);
  while (connectAttemptCount < MAX_MQTT_CONNECT_ATTTEMPTS)
  {
    ArduinoOTA.handle();
    updateLEDs();          // this is the yellow slow-blink window: WiFi up, MQTT not yet
    if (!mqttClient.connected())
    {
      unsigned long mqttNow = millis();
      if (mqttNow - lastReconnectAttempt > 1000)
      {
        LOG("[%s] Waiting for MQTT...\n", myTZ.dateTime(RFC3339).c_str());
        lastReconnectAttempt = mqttNow;
        connectAttemptCount++;
        if (mqttAttemptConnect())
        {
          lastReconnectAttempt = 0;
          return;
        }
      }
    }
  }
  LOG("ERROR-----Max MQTT connect attempts exceeded\n");
}


/*
 * MQTT callback - registered but currently inert: nothing is subscribed, so this never fires.
 *                 Kept as the landing point for future irrig_leak/cmd/# handling.
 */
void callback(char *topic, byte *payload, unsigned int length)
{
  if (length >= sizeof(mqttMsg))            // payload is not NUL-terminated; never trust its length
    length = sizeof(mqttMsg) - 1;
  memcpy(mqttMsg, payload, length);
  mqttMsg[length] = '\0';
  LOG("\n%s MQTT RECVD: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), topic, mqttMsg);
}


/*
 * MQTT reconnect
 */
boolean reconnect()
{
  if (mqttClient.connect(DEVICE_HOST_NAME, MQTT_USER_NAME, MQTT_PASSWORD, IRRIG_LWT_TOPIC, 2, true, "Disconnected"))
  {
    LOG("MQTT connected to %s\n", MQTT_SERVER);
    updateLEDs();          // yellow goes solid now that both links are up
  }
  return mqttClient.connected();
}


/*
 * sendTotalsReport
 */
void sendTotalsReport()
{

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

  unsigned int galsAllZones = zoneData[0].measuredZoneGallons;   // zone-0 water still crossed the meter

  // Zones 1-4 only - zone 0 is reported through IRRIG_VALVES_OFF_LEAK_TOPIC instead.
  for (int i = 1; i <= TOT_NUM_VALVES; i++)
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
    byte sensorStatus = 0xFF;                 // non-zero so the while() runs at least once
    if ((unsigned long)(millis() - pressureLastRead) > PRESSURE_READ_INTERVAL_MS)
    {
      pressureLastRead = millis();
      int retries = 0;
      while (sensorStatus != M3200_STATUS_NORMAL && retries++ < 5)  // stale clears in ~2ms
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
            sensorPSI = ((rawP - 1638.0) / (14746.0 - 1638.0)) * MAX_PRESSURE;
            sensorTempC = ((rawT - 512.0) / (1075.0 - 512.0)) * 55.0;
          }
          else if (sensorStatus == M3200_STATUS_FAULT)
          {
            if ((unsigned long)(millis() - lastPressErrReport) > PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS)
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
          if ((unsigned long)(millis() - lastPressErrReport) > PRESSURE_SENSOR_FAULT_PUB_INTERVAL_MS)
          {
            LOG("Error reading pressure sensor: got %d bytes (expected 4)\n", n);
            lastPressErrReport = millis();
          }
          break;
        }
      }

      // Left the loop without the sensor ever reporting good data - an I2C failure, retries
      // exhausted on persistent stale, or a fault. Return the sentinel so the callers'
      // skip-publish handling applies; otherwise a bad reading reaches Home Assistant looking
      // indistinguishable from a real one. sensorPSI/sensorTempC persist across calls, so
      // without this a previous call's value would leak through looking perfectly plausible.
      if (sensorStatus != M3200_STATUS_NORMAL)
      {
        sensorPSI = PRESSURE_SENSOR_INVALID;
        sensorTempC = PRESSURE_SENSOR_INVALID;
      }
    }
  }
  else
    return(PRESSURE_SENSOR_INVALID);   // no sensor fitted - not a reading of zero

  if (pressOrtemp == READ_TEMPERATURE)
  {
    if (sensorTempC == PRESSURE_SENSOR_INVALID)
      return(PRESSURE_SENSOR_INVALID);
    return(PREFER_FAHRENHEIT ? (sensorTempC * 9 / 5) + 32 : sensorTempC);
  }
  return(sensorPSI);
}
