#include <Arduino.h>
#include <Wire.h>

#define I2C_SCL_PIN   1     // was 13
#define I2C_SDA_PIN   2     // was 14
#define M3200_ADDR    0x28
#define MAX_PRESSURE  100.0f

static void checkBus(const char* label)
{
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  delay(2);
  Serial.printf("%s  SDA=%d SCL=%d\n", label, digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
}

// Read without enabling internal pullups — reveals whether external pullups (LTC4311) are present
static void checkBusNoPullup(const char* label)
{
  pinMode(I2C_SCL_PIN, INPUT);
  pinMode(I2C_SDA_PIN, INPUT);
  delay(2);
  Serial.printf("%s  SDA=%d SCL=%d  (no internal pullup)\n", label, digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
}

// Drive both lines HIGH and read while still in OUTPUT mode
// If we read 0 here, something is fighting GPIO output drive (short to GND, or GPIO damaged)
// If we read 1 here, the GPIO can win — something else pulls LOW only when we release
static void driveAndRead(const char* label)
{
  pinMode(I2C_SCL_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delay(2);
  int scl = digitalRead(I2C_SCL_PIN);
  int sda = digitalRead(I2C_SDA_PIN);
  Serial.printf("%s  SDA=%d SCL=%d  (while driving OUTPUT HIGH)\n", label, sda, scl);
  if (scl == 0 || sda == 0)
    Serial.println("  *** FAILED to drive line HIGH — possible GPIO damage or short to GND ***");
  // Release back to input with pullup
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  delay(2);
  Serial.printf("%s  SDA=%d SCL=%d  (after releasing to INPUT_PULLUP)\n", label, digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
}

// Drive both lines LOW and read — reveals if something external prevents LOW drive
static void driveAndReadLow(const char* label)
{
  pinMode(I2C_SCL_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, LOW);
  digitalWrite(I2C_SDA_PIN, LOW);
  delay(2);
  int scl = digitalRead(I2C_SCL_PIN);
  int sda = digitalRead(I2C_SDA_PIN);
  Serial.printf("%s  SDA=%d SCL=%d  (while driving OUTPUT LOW — expect 0,0)\n", label, sda, scl);
  if (scl == 1 || sda == 1)
    Serial.println("  *** FAILED to drive line LOW — something pulling HIGH strongly ***");
  // Release back to input with pullup
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  delay(2);
  Serial.printf("%s  SDA=%d SCL=%d  (after releasing to INPUT_PULLUP)\n", label, digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
}

static void busRecovery()
{
  Serial.println("Recovery: 9 SCL pulses at 50us half-period...");
  pinMode(I2C_SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(50);
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(50);
    Serial.printf("  pulse %d: SDA=%d\n", i+1, digitalRead(I2C_SDA_PIN));
  }
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(50);
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(50);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(50);
  Serial.println("Recovery: STOP sent");
}

static void readM3200()
{
  Wire.requestFrom(M3200_ADDR, (uint8_t)4);
  int n = Wire.available();
  if (n == 4) {
    uint16_t rawP = ((uint16_t)Wire.read() << 8) | Wire.read();
    uint16_t rawT = ((uint16_t)Wire.read() << 8) | Wire.read();
    uint8_t status = rawP >> 14;
    rawP &= 0x3FFF;
    rawT >>= 5;
    float psi   = ((rawP - 1000.0f) / (15000.0f - 1000.0f)) * MAX_PRESSURE;
    float tempC = ((rawT -  512.0f) / ( 1075.0f -  512.0f)) * 55.0f;
    Serial.printf("  status=0x%02X  PSI=%.2f  TempF=%.1f\n",
                  status, psi, (tempC * 9.0f / 5.0f) + 32.0f);
  } else {
    Wire.endTransmission();
    Serial.printf("  read failed: %d bytes\n", n);
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  delay(500);
  Serial.println("\n\n=== I2C Bus Diagnostic for M3200 ===");

  // Step 0: Wire.begin() as the VERY FIRST operation — no GPIO manipulation before this.
  // If this works, the GPIO manipulation in steps 3-4 is what corrupts Wire on GPIO 13/14.
  // If this also fails, the problem is independent of our setup code.
  Serial.println("0. Wire.begin() FIRST (no prior GPIO manipulation):");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  Serial.print("   M3200 read: ");
  readM3200();
  Wire.end();
  delay(10);

  // Step 1: What does the bus look like at raw boot?
  // No pullup: shows whether LTC4311 external pullups are present and working
  // With pullup: shows whether internal pullup is sufficient
  checkBusNoPullup("1a. Boot, no pullup:      ");
  checkBus        ("1b. Boot, with pullup:    ");

  // Step 2: Wire.end() to release any peripheral GPIO mux state
  Serial.println("2. Calling Wire.end()...");
  Wire.end();
  delay(20);
  checkBus("   After Wire.end:         ");

  // Step 3a: Drive both lines HIGH and read (test if something prevents HIGH)
  Serial.println("3a. Driving BOTH lines HIGH (OUTPUT mode)...");
  driveAndRead("   Both HIGH:            ");

  // Step 3b: Drive both lines LOW and read (test if something prevents LOW)
  Serial.println("3b. Driving BOTH lines LOW (OUTPUT mode)...");
  driveAndReadLow("   Both LOW:             ");

  // Step 4: 9-clock recovery
  busRecovery();
  checkBus("4. After 9-clock recovery: ");

  // Step 5: Full bus scan
  Serial.println("5. Wire.begin() + full bus scan...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(10);
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) { Serial.printf("   Device at 0x%02X\n", addr); found++; }
    else if (err != 2) Serial.printf("   Error %d at 0x%02X\n", err, addr);
  }
  Serial.printf("   Total found: %d\n", found);

  Wire.setTimeOut(100);
  Serial.println("6. Reading M3200 (0x28):");
  readM3200();

  // Step 7: Solar-style minimal test — exact equivalent of the working solar firmware.
  // No recovery, no timeout, no write probe. Fresh Wire.begin then requestFrom.
  Serial.println("7. Solar-style minimal test (no timeout, no recovery):");
  Wire.end();
  delay(50);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  checkBus("   Bus after solar Wire.begin:  ");
  Serial.print("   M3200 read: ");
  readM3200();

  Serial.println("=== Setup complete -- reading every 2s ===\n");
}

void loop()
{
  delay(2000);
  Serial.printf("[%lu] ", millis());
  readM3200();
}
