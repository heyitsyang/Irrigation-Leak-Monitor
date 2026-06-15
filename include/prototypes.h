/*
 *  prototypes.h
 */
void setup_wifi();
void setup_OTA();
void callback(char *topic, byte *payload, unsigned int length);
boolean reconnect();
void connectMQTT();
void resetSessionData();
void publishSessionReport();

int getActiveValve();
void sendTotalsReport();
void sendPressureSensorStatus();
void sendBatteryStatus();
float readPressureSensor(int pressOrtemp);
