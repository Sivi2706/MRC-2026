
/*
  AeroNotts CanSat Flight Computer
  ESP32 + MPU6050 + NEO-6M + BMP280 + SX1278 + W25Q128

  Data path:
    Sensors -> scale -> 42-byte binary packet -> W25Q128 -> SX1278

  Recovery:
    Ground station can request old packet numbers.
    CanSat reads the exact original 42-byte record from W25Q128 and retransmits it.

  THIS IS THE COMPLETE CANSAT PROGRAM:
  no separate project header/calibration/erase sketch is required.

  Built-in startup maintenance commands:
    CALACC   = six-position accelerometer calibration and save
    CLEARCAL = clear saved accelerometer calibration
    ERASE    = erase W25Q128 for a fresh mission
    START    = skip maintenance window

  Required Arduino libraries:
    Adafruit MPU6050
    Adafruit BMP280
    Adafruit Unified Sensor
    TinyGPSPlus
    RadioLib

  IMPORTANT:
    This sketch assumes the shared SPI wiring:
      SCK  = GPIO18
      MISO = GPIO19
      MOSI = GPIO23

    If your particular W25Q128 breakout only worked in your previous test with
    MISO=23 and MOSI=19, swap SPI_MISO_PIN/SPI_MOSI_PIN HERE and wire BOTH the
    W25Q128 and SX1278 to that same swapped bus. Both SPI devices must share the
    same ESP32 MISO/MOSI directions.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <Preferences.h>

// ============================================================
// SHARED TELEMETRY PROTOCOL (contained in this one file)
// ============================================================

static constexpr uint8_t TEAM_ID_AERONOTTS = 0x01;

// Status bits carried in TelemetryPacket.status
enum TelemetryStatus : uint8_t {
  STATUS_MPU_OK        = 1 << 0,
  STATUS_BMP_OK        = 1 << 1,
  STATUS_GPS_FIX       = 1 << 2,
  STATUS_IMU_CAL       = 1 << 3,
  STATUS_BMP_ZERO      = 1 << 4,
  STATUS_FLASH_OK      = 1 << 5,
  STATUS_LORA_OK       = 1 << 6,
  STATUS_RESERVED      = 1 << 7
};

#pragma pack(push, 1)

struct TelemetryPacket {
  uint8_t  teamId;          // 0x01 = AeroNotts
  uint32_t packetNo;        // monotonically increasing
  uint32_t timeMs;          // milliseconds since ESP32 boot

  int16_t  accX_mg;         // milli-g; 1000 mg = 1 g
  int16_t  accY_mg;
  int16_t  accZ_mg;

  int16_t  gyroX_dps10;     // deg/s * 10
  int16_t  gyroY_dps10;
  int16_t  gyroZ_dps10;

  int32_t  latitude_e7;     // degrees * 1e7
  int32_t  longitude_e7;    // degrees * 1e7

  uint32_t pressure_Pa;     // Pa
  int16_t  temperature_cC;  // deg C * 100
  int32_t  altitude_mm;     // relative barometric altitude, mm

  uint8_t  status;
  uint16_t crc16;           // CRC-16/CCITT-FALSE over every byte before crc16
};

// Ground -> CanSat request.
// Historical telemetry is resent as the original 42-byte TelemetryPacket.
struct ResendRequest {
  uint16_t magic;           // 0xA65C
  uint8_t  version;         // 1
  uint8_t  command;         // 1 = resend range
  uint32_t startPacket;     // first requested packet number
  uint16_t count;           // number of sequential packets requested
  uint16_t crc16;
};

#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == 42, "TelemetryPacket must be exactly 42 bytes");
static_assert(sizeof(ResendRequest) == 12, "ResendRequest must be exactly 12 bytes");

static constexpr uint16_t RESEND_MAGIC = 0xA65C;
static constexpr uint8_t  PROTOCOL_VERSION = 1;
static constexpr uint8_t  CMD_RESEND_RANGE = 1;

inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

inline uint16_t telemetry_crc(const TelemetryPacket& p) {
  return crc16_ccitt_false(reinterpret_cast<const uint8_t*>(&p),
                           sizeof(TelemetryPacket) - sizeof(p.crc16));
}

inline bool telemetry_valid(const TelemetryPacket& p) {
  return p.teamId == TEAM_ID_AERONOTTS && p.crc16 == telemetry_crc(p);
}

inline uint16_t resend_crc(const ResendRequest& r) {
  return crc16_ccitt_false(reinterpret_cast<const uint8_t*>(&r),
                           sizeof(ResendRequest) - sizeof(r.crc16));
}

inline bool resend_valid(const ResendRequest& r) {
  return r.magic == RESEND_MAGIC &&
         r.version == PROTOCOL_VERSION &&
         r.command == CMD_RESEND_RANGE &&
         r.crc16 == resend_crc(r);
}


// ---------------- Pin map ----------------
static constexpr int I2C_SDA_PIN = 21;
static constexpr int I2C_SCL_PIN = 22;

static constexpr int GPS_RX_PIN = 16;   // ESP32 RX <- NEO-6M TX
static constexpr int GPS_TX_PIN = 17;   // ESP32 TX -> NEO-6M RX

static constexpr int SPI_SCK_PIN  = 18;
static constexpr int SPI_MISO_PIN = 19;
static constexpr int SPI_MOSI_PIN = 23;

static constexpr int LORA_CS_PIN   = 27;
static constexpr int LORA_DIO0_PIN = 26;
static constexpr int LORA_RST_PIN  = 25;
static constexpr int LORA_DIO1_PIN = 32;

static constexpr int FLASH_CS_PIN  = 33;

// ---------------- Mission timing ----------------
static constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;
static constexpr uint32_t IMU_UPDATE_MS = 20;       // 50 Hz internal filter
static constexpr uint32_t COMMAND_WINDOW_MS = 280;
static constexpr uint32_t RESEND_GUARD_MS = 360;    // keep enough time for next 1 Hz live packet
static constexpr uint16_t MAX_RESEND_REQUEST = 8;

// ---------------- LoRa settings ----------------
// Same settings MUST be used at the ground station.
// SF9/BW125 keeps a 42-byte packet well below 1 s airtime and leaves time for recovery.
static constexpr float   LORA_FREQ_MHZ = 433.0;
static constexpr float   LORA_BW_KHZ = 125.0;
static constexpr uint8_t LORA_SF = 9;
static constexpr uint8_t LORA_CR = 5;               // 4/5
static constexpr uint8_t LORA_SYNC_WORD = 0x12;
static constexpr int8_t  LORA_POWER_DBM = 17;       // verify local permitted RF output
static constexpr uint16_t LORA_PREAMBLE = 8;

// ---------------- Sensor calibration ----------------
// Accelerometer calibration is stored persistently in ESP32 NVS.
// Run the built-in CALACC startup command once after assembling the avionics.
float accBiasX_mps2 = 0.0f;
float accBiasY_mps2 = 0.0f;
float accBiasZ_mps2 = 0.0f;
float accScaleX = 1.0f;
float accScaleY = 1.0f;
float accScaleZ = 1.0f;
bool accelCalibrationStored = false;

Preferences preferences;

static constexpr float STANDARD_GRAVITY = 9.80665f;
static constexpr float RAD_TO_DEG_F = 57.2957795131f;

// Short startup window for maintenance commands.
// Commands:
//   CALACC   -> six-position accelerometer calibration, saved in ESP32 NVS
//   CLEARCAL -> remove saved accelerometer calibration
//   ERASE    -> erase W25Q128 flight log
//   START    -> immediately continue to flight mode
static constexpr uint32_t MAINTENANCE_WINDOW_MS = 5000;

// ---------------- W25Q128 ----------------
static constexpr uint32_t W25_SIZE_BYTES = 16UL * 1024UL * 1024UL;
static constexpr uint32_t FLASH_HEADER_ADDR = 0x000000;
static constexpr uint32_t FLASH_DATA_ADDR   = 0x000100;  // 256-byte aligned
static constexpr uint32_t W25_PAGE_SIZE = 256;

static constexpr uint8_t W25_CMD_JEDEC_ID    = 0x9F;
static constexpr uint8_t W25_CMD_READ_DATA   = 0x03;
static constexpr uint8_t W25_CMD_WRITE_EN    = 0x06;
static constexpr uint8_t W25_CMD_READ_SR1    = 0x05;
static constexpr uint8_t W25_CMD_PAGE_PROG   = 0x02;
static constexpr uint8_t W25_CMD_CHIP_ERASE  = 0xC7;

#pragma pack(push, 1)
struct FlightHeader {
  char     magic[4];        // "ANT1"
  uint8_t  version;
  uint8_t  teamId;
  char     teamName[10];    // "AeroNotts"
  uint16_t recordSize;      // 42
  uint16_t samplePeriodMs;  // 1000
  uint8_t  reserved[42];
  uint16_t crc16;
};
#pragma pack(pop)
static_assert(sizeof(FlightHeader) == 64, "FlightHeader must be 64 bytes");

// Arduino .ino preprocessing can auto-generate function prototypes before
// custom structs are visible. These explicit declarations prevent that.
struct AccelAverage {
  double x;
  double y;
  double z;
};

uint16_t headerCrc(const FlightHeader& h);
bool savePacketToFlash(const TelemetryPacket& p);
bool loadPacketFromFlash(uint32_t packetNo, TelemetryPacket& p);
AccelAverage averageAccelerometer(uint16_t samples);
TelemetryPacket buildTelemetryPacket();
void acceptResendRequest(const ResendRequest& req);

// ---------------- Objects ----------------
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);
SX1278 radio = new Module(LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, LORA_DIO1_PIN);

// ---------------- Health/state ----------------
bool mpuOk = false;
bool bmpOk = false;
bool flashOk = false;
bool radioReady = false;
bool lastLoraTxOk = false;
bool imuCalibrated = false;
bool bmpZeroValid = false;

float gyroBiasX = 0.0f;  // rad/s
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

float launchPressurePa = NAN;

// Filtered IMU values
bool imuFilterInitialized = false;
float filtAx = 0, filtAy = 0, filtAz = 0;
float filtGx = 0, filtGy = 0, filtGz = 0;
static constexpr float IMU_ALPHA = 0.20f;

uint32_t nextPacketNo = 0;
uint32_t nextTelemetryMs = 0;
uint32_t nextImuMs = 0;

// Resend queue
uint32_t resendNextPacket = 0;
uint16_t resendRemaining = 0;

// ---------------- Utility ----------------
template <typename T>
T clampValue(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

int16_t toInt16Rounded(float v) {
  long x = lroundf(v);
  x = clampValue<long>(x, INT16_MIN, INT16_MAX);
  return (int16_t)x;
}

int32_t toInt32Rounded(double v) {
  long long x = llround(v);
  if (x < INT32_MIN) x = INT32_MIN;
  if (x > INT32_MAX) x = INT32_MAX;
  return (int32_t)x;
}

// ---------------- W25Q128 raw SPI driver ----------------
void flashSelect() {
  digitalWrite(LORA_CS_PIN, HIGH);
  digitalWrite(FLASH_CS_PIN, LOW);
}

void flashDeselect() {
  digitalWrite(FLASH_CS_PIN, HIGH);
}

void flashBeginTransaction() {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}

void flashEndTransaction() {
  SPI.endTransaction();
}

uint8_t w25ReadStatus1() {
  flashBeginTransaction();
  flashSelect();
  SPI.transfer(W25_CMD_READ_SR1);
  uint8_t s = SPI.transfer(0x00);
  flashDeselect();
  flashEndTransaction();
  return s;
}

bool w25WaitReady(uint32_t timeoutMs = 1000) {
  uint32_t start = millis();
  while (w25ReadStatus1() & 0x01) {
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

void w25WriteEnable() {
  flashBeginTransaction();
  flashSelect();
  SPI.transfer(W25_CMD_WRITE_EN);
  flashDeselect();
  flashEndTransaction();
}

uint32_t w25JedecId() {
  flashBeginTransaction();
  flashSelect();
  SPI.transfer(W25_CMD_JEDEC_ID);
  uint8_t m = SPI.transfer(0x00);
  uint8_t t = SPI.transfer(0x00);
  uint8_t c = SPI.transfer(0x00);
  flashDeselect();
  flashEndTransaction();
  return ((uint32_t)m << 16) | ((uint32_t)t << 8) | c;
}

void w25Read(uint32_t addr, uint8_t* dst, size_t len) {
  flashBeginTransaction();
  flashSelect();
  SPI.transfer(W25_CMD_READ_DATA);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (size_t i = 0; i < len; ++i) dst[i] = SPI.transfer(0x00);
  flashDeselect();
  flashEndTransaction();
}

bool w25PageProgram(uint32_t addr, const uint8_t* src, size_t len) {
  if (len == 0 || len > W25_PAGE_SIZE) return false;
  size_t pageRemaining = W25_PAGE_SIZE - (addr % W25_PAGE_SIZE);
  if (len > pageRemaining) return false;

  if (!w25WaitReady()) return false;
  w25WriteEnable();

  flashBeginTransaction();
  flashSelect();
  SPI.transfer(W25_CMD_PAGE_PROG);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (size_t i = 0; i < len; ++i) SPI.transfer(src[i]);
  flashDeselect();
  flashEndTransaction();

  return w25WaitReady();
}

bool w25Write(uint32_t addr, const uint8_t* src, size_t len) {
  while (len > 0) {
    size_t pageRemaining = W25_PAGE_SIZE - (addr % W25_PAGE_SIZE);
    size_t chunk = len < pageRemaining ? len : pageRemaining;
    if (!w25PageProgram(addr, src, chunk)) return false;
    addr += chunk;
    src += chunk;
    len -= chunk;
  }
  return true;
}

uint16_t headerCrc(const FlightHeader& h) {
  return crc16_ccitt_false(reinterpret_cast<const uint8_t*>(&h),
                           sizeof(FlightHeader) - sizeof(h.crc16));
}

bool bytesAreFF(const uint8_t* p, size_t len) {
  for (size_t i = 0; i < len; ++i) if (p[i] != 0xFF) return false;
  return true;
}

bool ensureFlightHeader() {
  FlightHeader existing{};
  w25Read(FLASH_HEADER_ADDR, reinterpret_cast<uint8_t*>(&existing), sizeof(existing));

  if (bytesAreFF(reinterpret_cast<const uint8_t*>(&existing), sizeof(existing))) {
    FlightHeader h{};
    memcpy(h.magic, "ANT1", 4);
    h.version = PROTOCOL_VERSION;
    h.teamId = TEAM_ID_AERONOTTS;
    memset(h.teamName, 0, sizeof(h.teamName));
    strncpy(h.teamName, "AeroNotts", sizeof(h.teamName) - 1);
    h.recordSize = sizeof(TelemetryPacket);
    h.samplePeriodMs = TELEMETRY_PERIOD_MS;
    memset(h.reserved, 0, sizeof(h.reserved));
    h.crc16 = headerCrc(h);
    return w25Write(FLASH_HEADER_ADDR, reinterpret_cast<const uint8_t*>(&h), sizeof(h));
  }

  bool valid = memcmp(existing.magic, "ANT1", 4) == 0 &&
               existing.version == PROTOCOL_VERSION &&
               existing.teamId == TEAM_ID_AERONOTTS &&
               existing.recordSize == sizeof(TelemetryPacket) &&
               existing.crc16 == headerCrc(existing);

  return valid;
}

uint32_t maxPacketCount() {
  return (W25_SIZE_BYTES - FLASH_DATA_ADDR) / sizeof(TelemetryPacket);
}

uint32_t packetAddress(uint32_t packetNo) {
  return FLASH_DATA_ADDR + packetNo * sizeof(TelemetryPacket);
}

uint32_t findFirstFreePacket() {
  const uint32_t maxCount = maxPacketCount();
  uint8_t firstByte = 0xFF;

  for (uint32_t i = 0; i < maxCount; ++i) {
    w25Read(packetAddress(i), &firstByte, 1);
    if (firstByte == 0xFF) return i;
  }
  return maxCount;
}

bool savePacketToFlash(const TelemetryPacket& p) {
  if (p.packetNo >= maxPacketCount()) return false;
  return w25Write(packetAddress(p.packetNo),
                  reinterpret_cast<const uint8_t*>(&p),
                  sizeof(p));
}

bool loadPacketFromFlash(uint32_t packetNo, TelemetryPacket& p) {
  if (packetNo >= nextPacketNo || packetNo >= maxPacketCount()) return false;
  w25Read(packetAddress(packetNo), reinterpret_cast<uint8_t*>(&p), sizeof(p));
  return telemetry_valid(p) && p.packetNo == packetNo;
}


void serviceGps();

// ---------------- Built-in maintenance + accelerometer calibration ----------------
void loadAccelerometerCalibration() {
  preferences.begin("aeronotts", true);
  accelCalibrationStored = preferences.getBool("accValid", false);

  if (accelCalibrationStored) {
    accBiasX_mps2 = preferences.getFloat("abx", 0.0f);
    accBiasY_mps2 = preferences.getFloat("aby", 0.0f);
    accBiasZ_mps2 = preferences.getFloat("abz", 0.0f);
    accScaleX = preferences.getFloat("asx", 1.0f);
    accScaleY = preferences.getFloat("asy", 1.0f);
    accScaleZ = preferences.getFloat("asz", 1.0f);
  }
  preferences.end();

  Serial.printf("Accel calibration stored: %s\n",
                accelCalibrationStored ? "YES" : "NO");
  if (accelCalibrationStored) {
    Serial.printf("Bias  X/Y/Z: %.6f %.6f %.6f m/s^2\n",
                  accBiasX_mps2, accBiasY_mps2, accBiasZ_mps2);
    Serial.printf("Scale X/Y/Z: %.6f %.6f %.6f\n",
                  accScaleX, accScaleY, accScaleZ);
  }
}

void saveAccelerometerCalibration() {
  preferences.begin("aeronotts", false);
  preferences.putFloat("abx", accBiasX_mps2);
  preferences.putFloat("aby", accBiasY_mps2);
  preferences.putFloat("abz", accBiasZ_mps2);
  preferences.putFloat("asx", accScaleX);
  preferences.putFloat("asy", accScaleY);
  preferences.putFloat("asz", accScaleZ);
  preferences.putBool("accValid", true);
  preferences.end();
  accelCalibrationStored = true;
}

void clearAccelerometerCalibration() {
  preferences.begin("aeronotts", false);
  preferences.clear();
  preferences.end();

  accBiasX_mps2 = 0.0f;
  accBiasY_mps2 = 0.0f;
  accBiasZ_mps2 = 0.0f;
  accScaleX = 1.0f;
  accScaleY = 1.0f;
  accScaleZ = 1.0f;
  accelCalibrationStored = false;

  Serial.println("Stored accelerometer calibration cleared.");
}

String readSerialLineBlocking() {
  String s;
  while (true) {
    serviceGps();
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r' || c == '\n') {
        s.trim();
        if (s.length() > 0) return s;
      } else {
        s += c;
      }
    }
    delay(10);
  }
}

void waitForCalibrationEnter(const char* message) {
  Serial.println();
  Serial.println(message);
  Serial.println("Keep the CanSat completely motionless, then press ENTER.");
  while (Serial.available()) Serial.read();

  while (true) {
    serviceGps();
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      delay(100);
      return;
    }
    delay(10);
  }
}

AccelAverage averageAccelerometer(uint16_t samples = 500) {
  AccelAverage v{0.0, 0.0, 0.0};

  for (uint16_t i = 0; i < samples; ++i) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    v.x += a.acceleration.x;
    v.y += a.acceleration.y;
    v.z += a.acceleration.z;
    serviceGps();
    delay(5);
  }

  v.x /= samples;
  v.y /= samples;
  v.z /= samples;
  return v;
}

void runSixPositionAccelerometerCalibration() {
  if (!mpuOk) {
    Serial.println("CALACC aborted: MPU6050 is not available.");
    return;
  }

  Serial.println();
  Serial.println("===== MPU6050 SIX-POSITION ACCELEROMETER CALIBRATION =====");
  Serial.println("Use the axis markings on the MPU6050 board.");
  Serial.println("For each step, point the stated axis vertically UP.");

  waitForCalibrationEnter("1/6: +X axis UP");
  AccelAverage xp = averageAccelerometer();

  waitForCalibrationEnter("2/6: -X axis UP");
  AccelAverage xn = averageAccelerometer();

  waitForCalibrationEnter("3/6: +Y axis UP");
  AccelAverage yp = averageAccelerometer();

  waitForCalibrationEnter("4/6: -Y axis UP");
  AccelAverage yn = averageAccelerometer();

  waitForCalibrationEnter("5/6: +Z axis UP");
  AccelAverage zp = averageAccelerometer();

  waitForCalibrationEnter("6/6: -Z axis UP");
  AccelAverage zn = averageAccelerometer();

  accBiasX_mps2 = (float)((xp.x + xn.x) / 2.0);
  accBiasY_mps2 = (float)((yp.y + yn.y) / 2.0);
  accBiasZ_mps2 = (float)((zp.z + zn.z) / 2.0);

  double dx = xp.x - xn.x;
  double dy = yp.y - yn.y;
  double dz = zp.z - zn.z;

  if (fabs(dx) < 1.0 || fabs(dy) < 1.0 || fabs(dz) < 1.0) {
    Serial.println("Calibration failed: one axis did not show enough +g/-g separation.");
    return;
  }

  accScaleX = (float)((2.0 * STANDARD_GRAVITY) / dx);
  accScaleY = (float)((2.0 * STANDARD_GRAVITY) / dy);
  accScaleZ = (float)((2.0 * STANDARD_GRAVITY) / dz);

  saveAccelerometerCalibration();

  Serial.println();
  Serial.println("Calibration saved permanently in ESP32 NVS.");
  Serial.printf("Bias  X/Y/Z: %.8f %.8f %.8f m/s^2\n",
                accBiasX_mps2, accBiasY_mps2, accBiasZ_mps2);
  Serial.printf("Scale X/Y/Z: %.8f %.8f %.8f\n",
                accScaleX, accScaleY, accScaleZ);
  Serial.println("==========================================================");
}

bool w25ChipErase() {
  if (!flashOk) return false;

  Serial.println("Erasing complete W25Q128. Do not remove power.");
  if (!w25WaitReady(1000)) return false;

  w25WriteEnable();

  flashBeginTransaction();
  flashSelect();
  SPI.transfer(W25_CMD_CHIP_ERASE);
  flashDeselect();
  flashEndTransaction();

  // A full chip erase can take many seconds.
  bool ok = w25WaitReady(180000);
  Serial.println(ok ? "W25Q128 erase complete." : "W25Q128 erase timed out.");
  return ok;
}

void runStartupMaintenanceWindow() {
  Serial.println();
  Serial.println("Maintenance window: 5 seconds");
  Serial.println("Type CALACC, CLEARCAL, ERASE, or START.");

  uint32_t start = millis();
  String command;

  while (millis() - start < MAINTENANCE_WINDOW_MS) {
    serviceGps();

    while (Serial.available()) {
      char c = (char)Serial.read();

      if (c == '\r' || c == '\n') {
        command.trim();
        command.toUpperCase();

        if (command == "START") {
          Serial.println("Starting flight mode.");
          return;
        }

        if (command == "CALACC") {
          runSixPositionAccelerometerCalibration();
          Serial.println("Type START to continue, or another maintenance command.");
          command = "";
          start = millis();
          continue;
        }

        if (command == "CLEARCAL") {
          clearAccelerometerCalibration();
          command = "";
          start = millis();
          continue;
        }

        if (command == "ERASE") {
          if (!flashOk) {
            Serial.println("ERASE aborted: W25Q128 not detected.");
          } else {
            Serial.println("Type ERASE again to confirm permanent deletion:");
            String confirm = readSerialLineBlocking();
            confirm.trim();
            confirm.toUpperCase();

            if (confirm == "ERASE") {
              if (w25ChipErase()) {
                nextPacketNo = 0;
              }
            } else {
              Serial.println("Erase cancelled.");
            }
          }

          command = "";
          start = millis();
          continue;
        }

        if (command.length() > 0) {
          Serial.printf("Unknown command: %s\n", command.c_str());
          command = "";
        }
      } else {
        command += c;
      }
    }

    delay(10);
  }

  Serial.println("Maintenance window closed. Entering flight mode.");
}


// ---------------- Sensors ----------------
void serviceGps() {
  while (GPSSerial.available() > 0) {
    gps.encode((char)GPSSerial.read());
  }
}

void calibrateGyroStationary() {
  if (!mpuOk) return;

  Serial.println("Keep CanSat completely still: calibrating gyro...");
  constexpr int N = 500;
  double sx = 0, sy = 0, sz = 0;
  int valid = 0;

  for (int i = 0; i < N; ++i) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    if (isfinite(g.gyro.x) && isfinite(g.gyro.y) && isfinite(g.gyro.z)) {
      sx += g.gyro.x;
      sy += g.gyro.y;
      sz += g.gyro.z;
      ++valid;
    }
    serviceGps();
    delay(5);
  }

  if (valid > N * 0.9) {
    gyroBiasX = sx / valid;
    gyroBiasY = sy / valid;
    gyroBiasZ = sz / valid;
    imuCalibrated = true;
  }
}

void establishBmpZero() {
  if (!bmpOk) return;

  Serial.println("Averaging launch-site pressure...");
  double sum = 0;
  int valid = 0;

  for (int i = 0; i < 60; ++i) {
    float p = bmp.readPressure();
    if (isfinite(p) && p > 30000.0f && p < 120000.0f) {
      sum += p;
      ++valid;
    }
    serviceGps();
    delay(40);
  }

  if (valid >= 50) {
    launchPressurePa = (float)(sum / valid);
    bmpZeroValid = true;
  }
}

void updateImuFilter() {
  if (!mpuOk) return;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float ax = (a.acceleration.x - accBiasX_mps2) * accScaleX;
  float ay = (a.acceleration.y - accBiasY_mps2) * accScaleY;
  float az = (a.acceleration.z - accBiasZ_mps2) * accScaleZ;

  float gx = g.gyro.x - gyroBiasX;
  float gy = g.gyro.y - gyroBiasY;
  float gz = g.gyro.z - gyroBiasZ;

  if (!imuFilterInitialized) {
    filtAx = ax; filtAy = ay; filtAz = az;
    filtGx = gx; filtGy = gy; filtGz = gz;
    imuFilterInitialized = true;
  } else {
    filtAx += IMU_ALPHA * (ax - filtAx);
    filtAy += IMU_ALPHA * (ay - filtAy);
    filtAz += IMU_ALPHA * (az - filtAz);
    filtGx += IMU_ALPHA * (gx - filtGx);
    filtGy += IMU_ALPHA * (gy - filtGy);
    filtGz += IMU_ALPHA * (gz - filtGz);
  }
}

float relativeAltitudeMeters(float pressurePa) {
  if (!bmpZeroValid || !isfinite(pressurePa) || pressurePa <= 0) return NAN;
  return 44330.0f * (1.0f - powf(pressurePa / launchPressurePa, 0.190294957f));
}

// ---------------- Packet creation ----------------
uint8_t buildStatus() {
  uint8_t s = 0;
  if (mpuOk) s |= STATUS_MPU_OK;
  if (bmpOk) s |= STATUS_BMP_OK;
  if (gps.location.isValid() && gps.location.age() < 3000) s |= STATUS_GPS_FIX;
  if (imuCalibrated) s |= STATUS_IMU_CAL;
  if (bmpZeroValid) s |= STATUS_BMP_ZERO;
  if (flashOk) s |= STATUS_FLASH_OK;
  if (radioReady && lastLoraTxOk) s |= STATUS_LORA_OK;
  return s;
}

TelemetryPacket buildTelemetryPacket() {
  TelemetryPacket p{};
  p.teamId = TEAM_ID_AERONOTTS;
  p.packetNo = nextPacketNo;
  p.timeMs = millis();

  if (mpuOk && imuFilterInitialized) {
    p.accX_mg = toInt16Rounded((filtAx / STANDARD_GRAVITY) * 1000.0f);
    p.accY_mg = toInt16Rounded((filtAy / STANDARD_GRAVITY) * 1000.0f);
    p.accZ_mg = toInt16Rounded((filtAz / STANDARD_GRAVITY) * 1000.0f);

    p.gyroX_dps10 = toInt16Rounded(filtGx * RAD_TO_DEG_F * 10.0f);
    p.gyroY_dps10 = toInt16Rounded(filtGy * RAD_TO_DEG_F * 10.0f);
    p.gyroZ_dps10 = toInt16Rounded(filtGz * RAD_TO_DEG_F * 10.0f);
  }

  if (gps.location.isValid() && gps.location.age() < 3000) {
    p.latitude_e7  = toInt32Rounded(gps.location.lat() * 1.0e7);
    p.longitude_e7 = toInt32Rounded(gps.location.lng() * 1.0e7);
  } else {
    p.latitude_e7 = 0;
    p.longitude_e7 = 0;
  }

  float pressure = NAN;
  float temperature = NAN;
  float altitude = NAN;

  if (bmpOk) {
    pressure = bmp.readPressure();
    temperature = bmp.readTemperature();
    altitude = relativeAltitudeMeters(pressure);
  }

  p.pressure_Pa = (isfinite(pressure) && pressure > 0) ? (uint32_t)lroundf(pressure) : 0;
  p.temperature_cC = isfinite(temperature) ? toInt16Rounded(temperature * 100.0f) : 0;
  p.altitude_mm = isfinite(altitude) ? toInt32Rounded(altitude * 1000.0) : 0;

  p.status = buildStatus();
  p.crc16 = telemetry_crc(p);
  return p;
}

// ---------------- Radio recovery ----------------
void acceptResendRequest(const ResendRequest& req) {
  if (!resend_valid(req)) return;
  if (req.count == 0) return;
  if (req.startPacket >= nextPacketNo) return;

  resendNextPacket = req.startPacket;
  uint32_t available = nextPacketNo - req.startPacket;
  uint32_t requested = req.count;
  if (requested > MAX_RESEND_REQUEST) requested = MAX_RESEND_REQUEST;
  if (requested > available) requested = available;
  resendRemaining = (uint16_t)requested;
}

void listenForResendRequest() {
  if (!radioReady) return;

  ResendRequest req{};
  int16_t state = radio.receive(reinterpret_cast<uint8_t*>(&req),
                                sizeof(req),
                                COMMAND_WINDOW_MS);

  if (state == RADIOLIB_ERR_NONE) {
    acceptResendRequest(req);
  }
}

void maybeSendOneRecoveredPacket() {
  if (!radioReady || !flashOk || resendRemaining == 0) return;

  // Do not sacrifice the next live 1 Hz sample.
  uint32_t now = millis();
  uint32_t timeUntilNext = nextTelemetryMs - now;
  if ((int32_t)timeUntilNext < (int32_t)RESEND_GUARD_MS) return;

  TelemetryPacket oldPacket{};
  if (loadPacketFromFlash(resendNextPacket, oldPacket)) {
    radio.transmit(reinterpret_cast<const uint8_t*>(&oldPacket), sizeof(oldPacket));
  }

  ++resendNextPacket;
  --resendRemaining;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(FLASH_CS_PIN, OUTPUT);
  pinMode(LORA_CS_PIN, OUTPUT);
  digitalWrite(FLASH_CS_PIN, HIGH);
  digitalWrite(LORA_CS_PIN, HIGH);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  GPSSerial.setRxBufferSize(1024);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // MPU6050
  mpuOk = mpu.begin(0x68, &Wire);
  if (mpuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // BMP280 (try both common I2C addresses)
  bmpOk = bmp.begin(0x76);
  if (!bmpOk) bmpOk = bmp.begin(0x77);
  if (bmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_63);
  }

  // W25Q128 hardware check
  uint32_t jedec = w25JedecId();
  Serial.printf("W25 JEDEC ID: 0x%06lX\n", (unsigned long)jedec);
  flashOk = (jedec == 0xEF4018UL);

  // Load saved accelerometer calibration and allow optional maintenance.
  loadAccelerometerCalibration();
  runStartupMaintenanceWindow();

  // Create/validate the binary flight header only after optional erase.
  if (flashOk) {
    flashOk = ensureFlightHeader();
    if (flashOk) {
      nextPacketNo = findFirstFreePacket();
      if (nextPacketNo >= maxPacketCount()) flashOk = false;
    }
  }

  // SX1278
  int16_t radioState = radio.begin(LORA_FREQ_MHZ,
                                   LORA_BW_KHZ,
                                   LORA_SF,
                                   LORA_CR,
                                   LORA_SYNC_WORD,
                                   LORA_POWER_DBM,
                                   LORA_PREAMBLE,
                                   0);
  radioReady = (radioState == RADIOLIB_ERR_NONE);
  lastLoraTxOk = radioReady;

  // Calibration happens before flight logging starts.
  calibrateGyroStationary();
  establishBmpZero();

  Serial.printf("MPU:%d BMP:%d FLASH:%d RADIO:%d nextPacket:%lu\n",
                mpuOk, bmpOk, flashOk, radioReady, (unsigned long)nextPacketNo);

  nextImuMs = millis();
  nextTelemetryMs = millis() + TELEMETRY_PERIOD_MS;
}

// ---------------- Main loop ----------------
void loop() {
  serviceGps();

  uint32_t now = millis();

  // Internal 50 Hz MPU update.
  if ((int32_t)(now - nextImuMs) >= 0) {
    nextImuMs += IMU_UPDATE_MS;
    updateImuFilter();
  }

  // Primary 1 Hz telemetry.
  if ((int32_t)(now - nextTelemetryMs) >= 0) {
    nextTelemetryMs += TELEMETRY_PERIOD_MS;

    TelemetryPacket packet = buildTelemetryPacket();

    // 1) STORE FIRST
    bool stored = false;
    if (flashOk) {
      stored = savePacketToFlash(packet);
      if (!stored) {
        flashOk = false;
        packet.status &= ~STATUS_FLASH_OK;
        packet.crc16 = telemetry_crc(packet);
      }
    }

    // Packet number advances whether or not LoRa succeeds.
    ++nextPacketNo;

    // 2) TRANSMIT THE SAME RECORD
    // A single timeout/error does NOT permanently disable the radio.
    // The next 1 Hz cycle will try again.
    if (radioReady) {
      int16_t state = radio.transmit(reinterpret_cast<const uint8_t*>(&packet),
                                     sizeof(packet));
      lastLoraTxOk = (state == RADIOLIB_ERR_NONE);
    } else {
      lastLoraTxOk = false;
    }

    // 3) Brief downlink window for missing-packet requests.
    // The window is long enough for the small request frame at SF9/BW125.
    if (radioReady) {
      listenForResendRequest();
      maybeSendOneRecoveredPacket();
    }

    Serial.printf("pkt=%lu stored=%d radio=%d txOK=%d gps=%d\n",
                  (unsigned long)packet.packetNo,
                  stored,
                  radioReady,
                  lastLoraTxOk,
                  (packet.status & STATUS_GPS_FIX) != 0);
  }
}