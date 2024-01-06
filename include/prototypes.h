/*
 *  prototypes.h
 */
void setup_wifi();
void setup_OTA();
void callback(char *topic, byte *payload, unsigned int length);
boolean reconnect();
void connectMQTT();
void exchangeComms(esp_sleep_wakeup_cause_t w_reason);

bool read_sleep_toggle();
uint32_t readADC_Cal(int ADC_Raw);
void print_wakeup_reason(esp_sleep_wakeup_cause_t w_reason);
void GoToSleep();
int getActiveValve();
void sendTotalsReport();
void sendPressureSensorStatus();
void sendBatteryStatus();
float readPressureSensor(int pressOrtemp);
