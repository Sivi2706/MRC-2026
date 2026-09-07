
/*
  AeroNotts LoRa Ground Station
  ESP32 + SX1278

  Responsibilities:
    1. Receive 42-byte telemetry packets.
    2. Validate application CRC.
    3. Track packet numbers and detect gaps.
    4. When the live link has been stable for several packets, request only missing ranges.
    5. Print decoded rows to USB Serial for the Python logger.

  The CanSat replies to a resend request by retransmitting the exact original
  binary record stored in W25Q128.

  THIS IS THE COMPLETE GROUND-STATION ESP32 PROGRAM.
  No separate telemetry_protocol.h file is required.

  Required Arduino library:
    RadioLib
*/

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

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


// Use the same radio wiring as the CanSat unless your receiver hardware differs.
static constexpr int SPI_SCK_PIN  = 18;
static constexpr int SPI_MISO_PIN = 19;
static constexpr int SPI_MOSI_PIN = 23;

static constexpr int LORA_CS_PIN   = 27;
static constexpr int LORA_DIO0_PIN = 26;
static constexpr int LORA_RST_PIN  = 25;
static constexpr int LORA_DIO1_PIN = 32;

// Must exactly match CanSat settings.
static constexpr float   LORA_FREQ_MHZ = 433.0;
static constexpr float   LORA_BW_KHZ = 125.0;
static constexpr uint8_t LORA_SF = 9;
static constexpr uint8_t LORA_CR = 5;
static constexpr uint8_t LORA_SYNC_WORD = 0x12;
static constexpr int8_t  LORA_POWER_DBM = 17;   // downlink request power; verify local permitted RF output
static constexpr uint16_t LORA_PREAMBLE = 8;

// Recovery policy
static constexpr uint8_t STABLE_PACKETS_BEFORE_RECOVERY = 5;
static constexpr uint32_t REQUEST_COOLDOWN_MS = 5000;
static constexpr uint16_t REQUEST_BATCH = 4;

// 65,536 seconds = 18.2 h at 1 Hz.
// Bitset uses only 8192 bytes RAM.
static constexpr uint32_t TRACKED_PACKETS = 65536;
uint8_t receivedBits[TRACKED_PACKETS / 8] = {0};

SX1278 radio = new Module(LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, LORA_DIO1_PIN);
bool radioReady = false;

bool haveAny = false;
uint32_t highestSeen = 0;
uint32_t lastForwardPacket = 0;
bool haveForward = false;
uint8_t stableCount = 0;
uint32_t lastRequestMs = 0;

bool packetTracked(uint32_t n) {
  return n < TRACKED_PACKETS;
}

bool isReceived(uint32_t n) {
  if (!packetTracked(n)) return false;
  return (receivedBits[n >> 3] >> (n & 7)) & 1U;
}

void markReceived(uint32_t n) {
  if (!packetTracked(n)) return;
  receivedBits[n >> 3] |= (uint8_t)(1U << (n & 7));
}

bool findMissingRange(uint32_t& start, uint16_t& count) {
  if (!haveAny) return false;

  uint32_t limit = (highestSeen < (TRACKED_PACKETS - 1)) ? highestSeen : (TRACKED_PACKETS - 1);
  for (uint32_t i = 0; i <= limit; ++i) {
    if (!isReceived(i)) {
      start = i;
      count = 1;
      while (count < REQUEST_BATCH &&
             i + count <= limit &&
             !isReceived(i + count)) {
        ++count;
      }
      return true;
    }
  }
  return false;
}

void sendResendRequest(uint32_t start, uint16_t count) {
  if (!radioReady) return;
  ResendRequest req{};
  req.magic = RESEND_MAGIC;
  req.version = PROTOCOL_VERSION;
  req.command = CMD_RESEND_RANGE;
  req.startPacket = start;
  req.count = count;
  req.crc16 = resend_crc(req);

  // Give the CanSat a short moment to switch from TX to RX.
  delay(20);
  int16_t state = radio.transmit(reinterpret_cast<const uint8_t*>(&req), sizeof(req));
  Serial.printf("#REQ,%lu,%u,%d\n",
                (unsigned long)start,
                count,
                state);
  lastRequestMs = millis();
}

const char* teamName(uint8_t id) {
  return id == TEAM_ID_AERONOTTS ? "AeroNotts" : "Unknown";
}

void printDecoded(const TelemetryPacket& p, float rssi, float snr, bool recovered) {
  const float ax = (p.accX_mg / 1000.0f) * 9.80665f;
  const float ay = (p.accY_mg / 1000.0f) * 9.80665f;
  const float az = (p.accZ_mg / 1000.0f) * 9.80665f;

  const float gx = p.gyroX_dps10 / 10.0f;
  const float gy = p.gyroY_dps10 / 10.0f;
  const float gz = p.gyroZ_dps10 / 10.0f;

  const double lat = p.latitude_e7 / 1.0e7;
  const double lon = p.longitude_e7 / 1.0e7;
  const float tempC = p.temperature_cC / 100.0f;
  const float altM = p.altitude_mm / 1000.0f;

  // D = valid data row. Python logger ignores # diagnostic lines.
  Serial.printf(
    "D,%s,%lu,%lu,%.5f,%.5f,%.5f,%.1f,%.1f,%.1f,%.7f,%.7f,%lu,%.2f,%.3f,%u,%.1f,%.1f,%u\n",
    teamName(p.teamId),
    (unsigned long)p.packetNo,
    (unsigned long)p.timeMs,
    ax, ay, az,
    gx, gy, gz,
    lat, lon,
    (unsigned long)p.pressure_Pa,
    tempC,
    altM,
    p.status,
    rssi,
    snr,
    recovered ? 1 : 0
  );
}

void setup() {
  Serial.begin(115200);
  delay(300);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  int16_t state = radio.begin(LORA_FREQ_MHZ,
                              LORA_BW_KHZ,
                              LORA_SF,
                              LORA_CR,
                              LORA_SYNC_WORD,
                              LORA_POWER_DBM,
                              LORA_PREAMBLE,
                              0);

  radioReady = (state == RADIOLIB_ERR_NONE);
  Serial.printf("#RADIO,%d\n", state);
  Serial.println("#CSV,Team,Packet,Time_ms,Ax_mps2,Ay_mps2,Az_mps2,Gx_dps,Gy_dps,Gz_dps,Lat,Lon,Pressure_Pa,Temp_C,Alt_m,Status,RSSI_dBm,SNR_dB,Recovered");
}

void loop() {
  if (!radioReady) {
    delay(1000);
    return;
  }

  TelemetryPacket p{};
  int16_t state = radio.receive(reinterpret_cast<uint8_t*>(&p), sizeof(p));

  if (state != RADIOLIB_ERR_NONE) {
    return;
  }

  float rssi = radio.getRSSI();
  float snr = radio.getSNR();

  if (!telemetry_valid(p)) {
    Serial.printf("#BADCRC,%.1f,%.1f\n", rssi, snr);
    return;
  }

  bool duplicate = isReceived(p.packetNo);
  uint32_t previousHighest = haveAny ? highestSeen : 0;
  bool recovered = haveAny && p.packetNo < previousHighest;

  if (!duplicate) {
    markReceived(p.packetNo);

    if (!haveAny || p.packetNo > highestSeen) {
      highestSeen = p.packetNo;
      haveAny = true;
    }

    printDecoded(p, rssi, snr, recovered);
  }

  // Treat only new forward-moving packets as live-link evidence.
  bool forwardPacket = !haveForward || p.packetNo > lastForwardPacket;
  if (forwardPacket) {
    if (!haveForward) {
      stableCount = 1;
    } else if (p.packetNo == lastForwardPacket + 1) {
      if (stableCount < 255) ++stableCount;
    } else {
      stableCount = 1;
    }

    lastForwardPacket = p.packetNo;
    haveForward = true;

    // Send request immediately after a good live packet because the CanSat
    // opens its short receive window directly after live transmission.
    if (stableCount >= STABLE_PACKETS_BEFORE_RECOVERY &&
        millis() - lastRequestMs >= REQUEST_COOLDOWN_MS) {
      uint32_t start = 0;
      uint16_t count = 0;
      if (findMissingRange(start, count)) {
        sendResendRequest(start, count);
      }
    }
  }
}