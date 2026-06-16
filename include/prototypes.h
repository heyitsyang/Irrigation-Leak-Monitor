/*
 *  prototypes.h
 */
struct CandidateRSSI { int8_t primary; int8_t secondary; };

void setup_wifi();
void setup_OTA();
CandidateRSSI scanAndLogWifi();
void callback(char *topic, byte *payload, unsigned int length);
boolean reconnect();
void connectMQTT();
void resetSessionData();
void publishSessionReport();

int getActiveValve();
void sendTotalsReport();
void sendPressureSensorStatus();
float readPressureSensor(int pressOrtemp);
