// /*
//   AeroNotts CanSat Component Test
//   ESP32 + MPU6050 + BMP280 + NEO-6M + SX1278 + W25Q128

//   Serial monitor: 115200 baud

//   Tests:
//     1) I2C scan
//     2) MPU6050 detection + live accel/gyro
//     3) BMP280 detection + pressure/temperature
//     4) NEO-6M serial data + GPS fix status
//     5) W25Q128 JEDEC ID (NON-DESTRUCTIVE: no erase/write)
//     6) SX1278 initialization + test packet every 10 s

//   Required libraries:
//     Adafruit MPU6050
//     Adafruit BMP280
//     Adafruit Unified Sensor
//     TinyGPSPlus
//     RadioLib
// */

// #include <Arduino.h>
// #include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_BMP280.h>
// #include <Adafruit_Sensor.h>
// #include <TinyGPSPlus.h>
// #include <RadioLib.h>

// // ---------------- Pin map ----------------
// static constexpr int I2C_SDA_PIN = 21;
// static constexpr int I2C_SCL_PIN = 22;

// static constexpr int GPS_RX_PIN = 16;   // ESP32 RX <- GPS TX
// static constexpr int GPS_TX_PIN = 17;   // ESP32 TX -> GPS RX

// static constexpr int SPI_SCK_PIN  = 18;
// static constexpr int SPI_MISO_PIN = 19;
// static constexpr int SPI_MOSI_PIN = 23;

// static constexpr int LORA_CS_PIN   = 27;
// static constexpr int LORA_DIO0_PIN = 26;
// static constexpr int LORA_RST_PIN  = 25;
// static constexpr int LORA_DIO1_PIN = 32;

// static constexpr int FLASH_CS_PIN  = 33;

// // ---------------- LoRa settings ----------------
// static constexpr float LORA_FREQ_MHZ = 433.0;
// static constexpr float LORA_BW_KHZ = 125.0;
// static constexpr uint8_t LORA_SF = 9;
// static constexpr uint8_t LORA_CR = 5;
// static constexpr uint8_t LORA_SYNC_WORD = 0x12;
// static constexpr int8_t LORA_POWER_DBM = 10;
// static constexpr uint16_t LORA_PREAMBLE = 8;

// // ---------------- Objects ----------------
// Adafruit_MPU6050 mpu;
// Adafruit_BMP280 bmp;
// TinyGPSPlus gps;
// HardwareSerial GPSSerial(2);
// SX1278 radio = new Module(LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, LORA_DIO1_PIN);

// // ---------------- State ----------------
// bool mpuOk = false;
// bool bmpOk = false;
// bool flashOk = false;
// bool radioOk = false;
// uint8_t bmpAddress = 0;

// uint32_t lastReportMs = 0;
// uint32_t lastLoRaMs = 0;
// uint32_t txCounter = 0;

// static constexpr uint8_t W25_CMD_JEDEC_ID = 0x9F;

// // ============================================================
// // Utility
// // ============================================================
// void passFail(const char* name, bool ok) {
//   Serial.printf("%-18s : %s\n", name, ok ? "PASS" : "FAIL");
// }

// void serviceGps() {
//   while (GPSSerial.available()) {
//     gps.encode((char)GPSSerial.read());
//   }
// }

// // ============================================================
// // I2C test
// // ============================================================
// void scanI2C() {
//   Serial.println("\n[I2C SCAN]");
//   int count = 0;

//   for (uint8_t addr = 1; addr < 127; ++addr) {
//     Wire.beginTransmission(addr);
//     if (Wire.endTransmission() == 0) {
//       Serial.printf("  Found device at 0x%02X\n", addr);
//       ++count;
//     }
//   }

//   if (count == 0) {
//     Serial.println("  No I2C devices found.");
//   } else {
//     Serial.printf("  Total: %d device(s)\n", count);
//   }
// }

// // ============================================================
// // MPU6050
// // ============================================================
// void testMPU6050() {
//   Serial.println("\n[MPU6050]");

//   mpuOk = mpu.begin(0x68, &Wire);

//   if (!mpuOk) {
//     Serial.println("  Not detected at 0x68.");
//     Serial.println("  Check VCC, GND, SDA, SCL and AD0.");
//     passFail("MPU6050", false);
//     return;
//   }

//   mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
//   mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   sensors_event_t a, g, t;
//   mpu.getEvent(&a, &g, &t);

//   Serial.printf("  Accel X/Y/Z: %.2f / %.2f / %.2f m/s^2\n",
//                 a.acceleration.x, a.acceleration.y, a.acceleration.z);
//   Serial.printf("  Gyro  X/Y/Z: %.3f / %.3f / %.3f rad/s\n",
//                 g.gyro.x, g.gyro.y, g.gyro.z);
//   Serial.printf("  Temp: %.2f C\n", t.temperature);

//   passFail("MPU6050", true);
// }

// // ============================================================
// // BMP280
// // ============================================================
// void testBMP280() {
//   Serial.println("\n[BMP280]");

//   bmpOk = bmp.begin(0x76);
//   if (bmpOk) {
//     bmpAddress = 0x76;
//   } else {
//     bmpOk = bmp.begin(0x77);
//     if (bmpOk) bmpAddress = 0x77;
//   }

//   if (!bmpOk) {
//     Serial.println("  Not detected at 0x76 or 0x77.");
//     passFail("BMP280", false);
//     return;
//   }

//   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
//                   Adafruit_BMP280::SAMPLING_X2,
//                   Adafruit_BMP280::SAMPLING_X16,
//                   Adafruit_BMP280::FILTER_X16,
//                   Adafruit_BMP280::STANDBY_MS_63);

//   delay(100);

//   float p = bmp.readPressure();
//   float t = bmp.readTemperature();

//   Serial.printf("  Address: 0x%02X\n", bmpAddress);
//   Serial.printf("  Pressure: %.1f Pa\n", p);
//   Serial.printf("  Temperature: %.2f C\n", t);

//   bool valid = isfinite(p) && isfinite(t) && p > 30000.0f && p < 120000.0f;
//   bmpOk = valid;
//   passFail("BMP280", bmpOk);
// }

// // ============================================================
// // W25Q128 - read-only JEDEC test
// // ============================================================
// uint32_t readW25Jedec() {
//   digitalWrite(LORA_CS_PIN, HIGH);

//   SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
//   digitalWrite(FLASH_CS_PIN, LOW);

//   SPI.transfer(W25_CMD_JEDEC_ID);
//   uint8_t m = SPI.transfer(0x00);
//   uint8_t t = SPI.transfer(0x00);
//   uint8_t c = SPI.transfer(0x00);

//   digitalWrite(FLASH_CS_PIN, HIGH);
//   SPI.endTransaction();

//   return ((uint32_t)m << 16) | ((uint32_t)t << 8) | c;
// }

// void testW25Q128() {
//   Serial.println("\n[W25Q128]");

//   uint32_t jedec = readW25Jedec();
//   Serial.printf("  JEDEC ID: 0x%06lX\n", (unsigned long)jedec);

//   flashOk = (jedec == 0xEF4018UL);

//   if (!flashOk) {
//     Serial.println("  Expected 0xEF4018 for Winbond W25Q128.");
//     Serial.println("  Check 3.3V, GND, CS, SCK, MISO and MOSI.");
//   }

//   passFail("W25Q128", flashOk);
// }

// // ============================================================
// // SX1278
// // ============================================================
// void testSX1278() {
//   Serial.println("\n[SX1278 LORA]");

//   digitalWrite(FLASH_CS_PIN, HIGH);

//   int16_t state = radio.begin(LORA_FREQ_MHZ,
//                               LORA_BW_KHZ,
//                               LORA_SF,
//                               LORA_CR,
//                               LORA_SYNC_WORD,
//                               LORA_POWER_DBM,
//                               LORA_PREAMBLE,
//                               0);

//   radioOk = (state == RADIOLIB_ERR_NONE);

//   if (!radioOk) {
//     Serial.printf("  RadioLib error code: %d\n", state);
//     Serial.println("  Check 3.3V, GND, NSS, DIO0, RST, DIO1 and SPI wiring.");
//   } else {
//     Serial.println("  SX1278 initialized.");
//     Serial.printf("  %.1f MHz | BW %.1f kHz | SF%u\n",
//                   LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF);
//   }

//   passFail("SX1278", radioOk);
// }

// // ============================================================
// // GPS status
// // ============================================================
// void printGPS() {
//   serviceGps();

//   Serial.println("\n[NEO-6M GPS]");
//   Serial.printf("  Characters received: %lu\n",
//                 (unsigned long)gps.charsProcessed());

//   if (gps.satellites.isValid()) {
//     Serial.printf("  Satellites: %u\n", gps.satellites.value());
//   } else {
//     Serial.println("  Satellites: unavailable");
//   }

//   if (gps.location.isValid()) {
//     Serial.printf("  Latitude : %.7f\n", gps.location.lat());
//     Serial.printf("  Longitude: %.7f\n", gps.location.lng());
//     Serial.printf("  Fix age  : %lu ms\n",
//                   (unsigned long)gps.location.age());
//   } else {
//     Serial.println("  Position: NO VALID FIX");
//   }

//   if (gps.charsProcessed() == 0) {
//     Serial.println("  GPS SERIAL: FAIL");
//     Serial.println("  Check GPS TX -> ESP32 GPIO16.");
//   } else if (!gps.location.isValid()) {
//     Serial.println("  GPS SERIAL: PASS");
//     Serial.println("  GPS FIX   : NOT YET ACQUIRED");
//   } else {
//     Serial.println("  GPS SERIAL: PASS");
//     Serial.println("  GPS FIX   : PASS");
//   }
// }

// // ============================================================
// // Live report
// // ============================================================
// void printLiveReport() {
//   Serial.println("\n============================================================");
//   Serial.printf("LIVE REPORT @ %.1f s\n", millis() / 1000.0f);
//   Serial.println("============================================================");

//   if (mpuOk) {
//     sensors_event_t a, g, t;
//     mpu.getEvent(&a, &g, &t);

//     Serial.printf("MPU Accel: %7.2f %7.2f %7.2f m/s^2\n",
//                   a.acceleration.x,
//                   a.acceleration.y,
//                   a.acceleration.z);

//     Serial.printf("MPU Gyro : %7.3f %7.3f %7.3f rad/s\n",
//                   g.gyro.x,
//                   g.gyro.y,
//                   g.gyro.z);
//   } else {
//     Serial.println("MPU6050   : FAIL");
//   }

//   if (bmpOk) {
//     Serial.printf("BMP280    : %.1f Pa | %.2f C\n",
//                   bmp.readPressure(),
//                   bmp.readTemperature());
//   } else {
//     Serial.println("BMP280    : FAIL");
//   }

//   Serial.printf("W25Q128   : %s\n", flashOk ? "PASS" : "FAIL");
//   Serial.printf("SX1278    : %s\n", radioOk ? "PASS" : "FAIL");

//   printGPS();
// }

// // ============================================================
// // LoRa transmission test
// // ============================================================
// void sendLoRaTestPacket() {
//   if (!radioOk) return;

//   digitalWrite(FLASH_CS_PIN, HIGH);

//   String msg = "AERONOTTS_TEST_" + String(txCounter++);

//   Serial.printf("\n[LORA TX] %s\n", msg.c_str());

//   int16_t state = radio.transmit(msg);

//   if (state == RADIOLIB_ERR_NONE) {
//     Serial.println("[LORA TX] PASS");
//   } else {
//     Serial.printf("[LORA TX] FAIL - RadioLib code %d\n", state);
//   }
// }

// // ============================================================
// // Setup
// // ============================================================
// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   Serial.println("\n============================================================");
//   Serial.println("        AERONOTTS CANSAT COMPONENT TEST");
//   Serial.println("============================================================");
//   Serial.println("Non-destructive test: W25Q128 is NOT erased or written.");

//   pinMode(LORA_CS_PIN, OUTPUT);
//   pinMode(FLASH_CS_PIN, OUTPUT);
//   digitalWrite(LORA_CS_PIN, HIGH);
//   digitalWrite(FLASH_CS_PIN, HIGH);

//   Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
//   Wire.setClock(400000);

//   SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

//   GPSSerial.setRxBufferSize(1024);
//   GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

//   Serial.printf("\nI2C: SDA=%d SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
//   Serial.printf("SPI: SCK=%d MISO=%d MOSI=%d\n",
//                 SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
//   Serial.printf("GPS: RX=%d TX=%d @ 9600 baud\n",
//                 GPS_RX_PIN, GPS_TX_PIN);

//   scanI2C();
//   testMPU6050();
//   testBMP280();
//   testW25Q128();
//   testSX1278();

//   Serial.println("\n============================================================");
//   Serial.println("INITIAL SUMMARY");
//   Serial.println("============================================================");
//   passFail("MPU6050", mpuOk);
//   passFail("BMP280", bmpOk);
//   passFail("W25Q128", flashOk);
//   passFail("SX1278", radioOk);
//   Serial.println("NEO-6M GPS        : wait for live report");
//   Serial.println("============================================================");
//   Serial.println("Live report: every 5 seconds");
//   Serial.println("LoRa TX test: every 10 seconds");
//   Serial.println("For GPS fix, test outdoors with antenna facing sky.");

//   lastReportMs = millis();
//   lastLoRaMs = millis();
// }

// // ============================================================
// // Loop
// // ============================================================
// void loop() {
//   serviceGps();

//   uint32_t now = millis();

//   if (now - lastReportMs >= 5000) {
//     lastReportMs = now;
//     printLiveReport();
//   }

//   if (now - lastLoRaMs >= 10000) {
//     lastLoRaMs = now;
//     sendLoRaTestPacket();
//   }

//   delay(5);
// }
