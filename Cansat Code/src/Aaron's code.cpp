#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

Adafruit_BMP280 bmp;
MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2 on ESP32 (GPIO16=RX2, GPIO17=TX2)

// ---------- MPU6050 calibration offsets ----------
const int16_t AX_OFFSET = -2248;
const int16_t AY_OFFSET = -271;
const int16_t AZ_OFFSET = 320;
const int16_t GX_OFFSET = -473;
const int16_t GY_OFFSET = 49;
const int16_t GZ_OFFSET = 189;

float groundLevelPressure;

// ---------- Velocity ----------
float velocityX = 0, velocityY = 0, velocityZ = 0;
unsigned long lastTime;

const float REST_THRESHOLD = 0.15;
const int REST_SAMPLES_REQUIRED = 5;
int quietCounter = 0;

// ---------- Gravity vector estimate ----------
float gravityX = 0, gravityY = 0, gravityZ = 9.81;
const float GRAVITY_ALPHA = 0.02;

// ---------- Fast accel filter ----------
float filteredAccelX = 0, filteredAccelY = 0, filteredAccelZ = 0;
const float FILTER_ALPHA = 0.2;

// ---------- Altitude smoothing (baro) ----------
const int ALT_SAMPLES = 10;
float altitudeBuffer[ALT_SAMPLES];
int altitudeIndex = 0;
float smoothedAltitude = 0;
float previousAltitude = 0;

// ---------- Baro vertical speed smoothing ----------
float smoothedBaroSpeed = 0;
const float BARO_SPEED_FILTER_ALPHA = 0.1;
const float BARO_FUSION_ALPHA = 0.95; // Z fusion: accel + baro

// ---------- GPS fusion (X/Y horizontal velocity) ----------
// GPS gives ground speed + heading -> we convert to X/Y components
float gpsSpeed = 0;       // m/s, from GPS
float gpsHeading = 0;     // degrees, from GPS
float gpsVelX = 0, gpsVelY = 0; // decomposed GPS velocity
bool gpsHasFix = false;

// Fusion weight for horizontal velocity: accel (fast) vs GPS (accurate, slower update rate)
const float GPS_FUSION_ALPHA = 0.90; // GPS typically updates once per second, so weight it a bit more

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin();

  // --- GPS UART ---
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // NEO-7M default baud is 9600

  // --- BMP280 ---
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find BMP280, check wiring!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  delay(2000);
  groundLevelPressure = bmp.readPressure() / 100.0F;
  Serial.print("Ground level pressure: ");
  Serial.println(groundLevelPressure);

  // --- MPU6050 ---
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Could not find MPU6050, check wiring!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 connected.");

  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
  gravityX = (ax_raw - AX_OFFSET) / 16384.0 * 9.81;
  gravityY = (ay_raw - AY_OFFSET) / 16384.0 * 9.81;
  gravityZ = (az_raw - AZ_OFFSET) / 16384.0 * 9.81;

  float initialAlt = bmp.readAltitude(groundLevelPressure);
  for (int i = 0; i < ALT_SAMPLES; i++) altitudeBuffer[i] = initialAlt;
  smoothedAltitude = initialAlt;
  previousAltitude = initialAlt;

  Serial.println("Waiting for GPS fix (needs clear sky view, may take 30s-2min)...");

  lastTime = millis();
}

void loop() {
  // ---------- Feed GPS parser continuously ----------
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Update GPS-derived values whenever a new valid sentence is parsed
  if (gps.location.isUpdated() && gps.location.isValid()) {
    gpsHasFix = true;
  }
  if (gps.speed.isValid()) {
    gpsSpeed = gps.speed.mps(); // ground speed in m/s
  }
  if (gps.course.isValid()) {
    gpsHeading = gps.course.deg(); // heading in degrees (0=North, 90=East)
  }

  // Decompose GPS speed+heading into X (East) and Y (North) components
  if (gpsHasFix) {
    float headingRad = gpsHeading * PI / 180.0;
    gpsVelX = gpsSpeed * sin(headingRad); // East component
    gpsVelY = gpsSpeed * cos(headingRad); // North component
  }

  // ---------- BMP280 ----------
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  float rawAltitude = bmp.readAltitude(groundLevelPressure);

  altitudeBuffer[altitudeIndex] = rawAltitude;
  altitudeIndex = (altitudeIndex + 1) % ALT_SAMPLES;
  float sum = 0;
  for (int i = 0; i < ALT_SAMPLES; i++) sum += altitudeBuffer[i];
  smoothedAltitude = sum / ALT_SAMPLES;

  // ---------- MPU6050 ----------
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  int16_t ax = ax_raw - AX_OFFSET;
  int16_t ay = ay_raw - AY_OFFSET;
  int16_t az = az_raw - AZ_OFFSET;

  float accelX = (ax / 16384.0) * 9.81;
  float accelY = (ay / 16384.0) * 9.81;
  float accelZ = (az / 16384.0) * 9.81;

  gravityX = GRAVITY_ALPHA * accelX + (1 - GRAVITY_ALPHA) * gravityX;
  gravityY = GRAVITY_ALPHA * accelY + (1 - GRAVITY_ALPHA) * gravityY;
  gravityZ = GRAVITY_ALPHA * accelZ + (1 - GRAVITY_ALPHA) * gravityZ;

  float linearAccelX = accelX - gravityX;
  float linearAccelY = accelY - gravityY;
  float linearAccelZ = accelZ - gravityZ;

  filteredAccelX = FILTER_ALPHA * linearAccelX + (1 - FILTER_ALPHA) * filteredAccelX;
  filteredAccelY = FILTER_ALPHA * linearAccelY + (1 - FILTER_ALPHA) * filteredAccelY;
  filteredAccelZ = FILTER_ALPHA * linearAccelZ + (1 - FILTER_ALPHA) * filteredAccelZ;

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // ---------- X/Y velocity: NOW fused with GPS instead of raw accel-only ----------
  float velocityX_accel = velocityX + filteredAccelX * dt;
  float velocityY_accel = velocityY + filteredAccelY * dt;

  if (gpsHasFix) {
    // Complementary filter: fast accel response, corrected toward accurate GPS ground truth
    velocityX = GPS_FUSION_ALPHA * velocityX_accel + (1 - GPS_FUSION_ALPHA) * gpsVelX;
    velocityY = GPS_FUSION_ALPHA * velocityY_accel + (1 - GPS_FUSION_ALPHA) * gpsVelY;
  } else {
    // No GPS fix yet (e.g., indoors) - fall back to accel-only (known to drift)
    velocityX = velocityX_accel;
    velocityY = velocityY_accel;
  }

  // ---------- Z velocity: baro fusion (unchanged from before) ----------
  float velocityZ_accel = velocityZ + filteredAccelZ * dt;
  float rawBaroSpeed = (dt > 0) ? (smoothedAltitude - previousAltitude) / dt : 0;
  previousAltitude = smoothedAltitude;
  smoothedBaroSpeed = BARO_SPEED_FILTER_ALPHA * rawBaroSpeed + (1 - BARO_SPEED_FILTER_ALPHA) * smoothedBaroSpeed;
  velocityZ = BARO_FUSION_ALPHA * velocityZ_accel + (1 - BARO_FUSION_ALPHA) * smoothedBaroSpeed;

  // ---------- Rest detection (only matters when no GPS fix, as fallback safety) ----------
  bool isQuiet = (abs(filteredAccelX) < REST_THRESHOLD &&
                  abs(filteredAccelY) < REST_THRESHOLD &&
                  abs(filteredAccelZ) < REST_THRESHOLD);

  if (isQuiet) {
    quietCounter++;
  } else {
    quietCounter = 0;
  }

  if (quietCounter >= REST_SAMPLES_REQUIRED && !gpsHasFix) {
    velocityX = 0;
    velocityY = 0;
  }

  // ---------- Print everything ----------
  Serial.println("---------------------------");
  Serial.print("Temp: "); Serial.print(temperature); Serial.println(" C");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("Altitude (baro): "); Serial.print(smoothedAltitude); Serial.println(" m");

  Serial.print("GPS Fix: "); Serial.println(gpsHasFix ? "YES" : "NO (waiting...)");
  if (gpsHasFix) {
    Serial.print("GPS Lat: "); Serial.print(gps.location.lat(), 6);
    Serial.print(" Lng: "); Serial.println(gps.location.lng(), 6);
    Serial.print("GPS Altitude: "); Serial.print(gps.altitude.meters()); Serial.println(" m");
    Serial.print("GPS Speed: "); Serial.print(gpsSpeed); Serial.println(" m/s");
    Serial.print("GPS Satellites: "); Serial.println(gps.satellites.value());
  }

  Serial.print("Linear Accel X: "); Serial.print(filteredAccelX);
  Serial.print(" Y: "); Serial.print(filteredAccelY);
  Serial.print(" Z: "); Serial.println(filteredAccelZ);

  Serial.print("Velocity (fused) X: "); Serial.print(velocityX);
  Serial.print(" Y: "); Serial.print(velocityY);
  Serial.print(" Z: "); Serial.println(velocityZ);

  delay(200);
}