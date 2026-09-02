#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// ================================
// I2C
// ================================
#define SDA_PIN 21
#define SCL_PIN 22

// ================================
// SX1278
// ================================
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23

#define LORA_NSS   27
#define LORA_DIO0  26
#define LORA_RESET 25

#define LORA_VERSION_REG 0x42

// ================================
// GPS
// ================================
#define GPS_RX 16
#define GPS_TX 17

// ================================
// Device addresses
// ================================
#define MPU6050_ADDR 0x68
#define BMP280_ADDR  0x76

bool mpuFound = false;
bool bmpFound = false;
bool loraFound = false;


// ============================================================
// I2C READ
// ============================================================

uint8_t readI2C(uint8_t address, uint8_t reg)
{
  Wire.beginTransmission(address);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  Wire.requestFrom(address, (uint8_t)1);

  if (Wire.available())
    return Wire.read();

  return 0xFF;
}


// ============================================================
// TEST MPU6050
// ============================================================

void testMPU6050()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("        MPU6050 TEST");
  Serial.println("================================");

  uint8_t whoAmI = readI2C(MPU6050_ADDR, 0x75);

  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI == 0x68)
  {
    Serial.println("MPU6050: PASS");
    mpuFound = true;

    // Wake MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
  }
  else
  {
    Serial.println("MPU6050: FAIL");
  }
}


// ============================================================
// READ MPU6050
// ============================================================

void readMPU6050()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

  if (Wire.available() < 14)
  {
    Serial.println("MPU6050 read error!");
    return;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  int16_t temp = (Wire.read() << 8) | Wire.read();

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  Serial.print("ACCEL: ");
  Serial.print("X=");
  Serial.print(ax);
  Serial.print("  Y=");
  Serial.print(ay);
  Serial.print("  Z=");
  Serial.println(az);

  Serial.print("GYRO:  ");
  Serial.print("X=");
  Serial.print(gx);
  Serial.print("  Y=");
  Serial.print(gy);
  Serial.print("  Z=");
  Serial.println(gz);

  float temperatureC = (temp / 340.0) + 36.53;

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" C");
}


// ============================================================
// TEST BMP280
// ============================================================

void testBMP280()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("         BMP280 TEST");
  Serial.println("================================");

  uint8_t chipID = readI2C(BMP280_ADDR, 0xD0);

  Serial.print("Chip ID: 0x");
  Serial.println(chipID, HEX);

  if (chipID == 0x58)
  {
    Serial.println("BMP280: PASS");
    bmpFound = true;
  }
  else
  {
    Serial.println("BMP280: FAIL");
  }
}


// ============================================================
// TEST SX1278
// ============================================================

void testSX1278()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("         SX1278 TEST");
  Serial.println("================================");

  // Reset LoRa module

  digitalWrite(LORA_RESET, LOW);
  delay(20);

  digitalWrite(LORA_RESET, HIGH);
  delay(20);

  // Read version register

  SPI.beginTransaction(
    SPISettings(8000000, MSBFIRST, SPI_MODE0)
  );

  digitalWrite(LORA_NSS, LOW);

  SPI.transfer(LORA_VERSION_REG & 0x7F);

  uint8_t version = SPI.transfer(0x00);

  digitalWrite(LORA_NSS, HIGH);

  SPI.endTransaction();

  Serial.print("Version register: 0x");
  Serial.println(version, HEX);

  if (version == 0x12)
  {
    Serial.println("SX1278: PASS");
    loraFound = true;
  }
  else
  {
    Serial.println("SX1278: FAIL");
    Serial.println("Check:");
    Serial.println("- VCC");
    Serial.println("- GND");
    Serial.println("- SCK");
    Serial.println("- MISO");
    Serial.println("- MOSI");
    Serial.println("- NSS");
  }
}


// ============================================================
// GPS TEST
// ============================================================

void testGPS()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("          GPS TEST");
  Serial.println("================================");

  Serial.println("GPS UART initialized.");
  Serial.println("Waiting for NMEA data...");
}


// ============================================================
// GPS READ
// ============================================================

void readGPS()
{
  while (Serial2.available())
  {
    char c = Serial2.read();
    Serial.write(c);
  }
}


// ============================================================
// I2C SCANNER
// ============================================================

void scanI2C()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("          I2C SCAN");
  Serial.println("================================");

  int count = 0;

  for (uint8_t address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);

    if (Wire.endTransmission() == 0)
    {
      Serial.print("Device found: 0x");

      if (address < 16)
        Serial.print("0");

      Serial.println(address, HEX);

      count++;
    }
  }

  Serial.print("Total devices: ");
  Serial.println(count);
}


// ============================================================
// SUMMARY
// ============================================================

void printSummary()
{
  Serial.println();
  Serial.println();
  Serial.println("================================");
  Serial.println("       TEST SUMMARY");
  Serial.println("================================");

  Serial.print("MPU6050 : ");
  Serial.println(mpuFound ? "PASS" : "FAIL");

  Serial.print("BMP280  : ");
  Serial.println(bmpFound ? "PASS" : "FAIL");

  Serial.print("SX1278  : ");
  Serial.println(loraFound ? "PASS" : "FAIL");

  Serial.println("NEO-6M  : Check NMEA output");

  Serial.println("================================");
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("########################################");
  Serial.println("# ESP32 ROCKET AVIONICS HARDWARE TEST #");
  Serial.println("########################################");

  // -------------------------
  // I2C
  // -------------------------

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // -------------------------
  // SX1278 pins
  // -------------------------

  pinMode(LORA_NSS, OUTPUT);
  pinMode(LORA_RESET, OUTPUT);
  pinMode(LORA_DIO0, INPUT);

  digitalWrite(LORA_NSS, HIGH);
  digitalWrite(LORA_RESET, HIGH);

  // -------------------------
  // SPI
  // -------------------------

  SPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI
  );

  // -------------------------
  // GPS
  // -------------------------

  Serial2.begin(
    9600,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );

  // -------------------------
  // TEST
  // -------------------------

  scanI2C();

  testMPU6050();

  testBMP280();

  testSX1278();

  testGPS();

  printSummary();

  Serial.println();
  Serial.println("Starting live data...");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  static unsigned long lastMPU = 0;

  // MPU6050 every 1 second
  if (millis() - lastMPU >= 1000)
  {
    lastMPU = millis();

    if (mpuFound)
    {
      Serial.println();
      Serial.println("--- MPU6050 ---");

      readMPU6050();
    }
  }

  // GPS continuously
  readGPS();
}