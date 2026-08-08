/*
 *  prototypes.h
 */
void LOG(const char* fmt, ...);

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
const char* wifiStatusName(int status);
void scanForNetworks();
void setup_wifi();
void setup_OTA();
void callback(char *topic, byte *payload, unsigned int length);
boolean reconnect();
void connectMQTT();
void resetSessionData();
void publishSessionReport();

int getActiveValve();
void resetGPMHistogram();
void addGPMSample(float gpm);
float medianGPM();
void publishLeakTopic(bool isLeak, const char* context);
void sendTotalsReport();
void sendPressureSensorStatus();
float readPressureSensor(int pressOrtemp);
