// // ============================================================================
// //  kalman_apogee_hand_test.cpp
// //  HAND-TEST VERSION for checking apogee detection behaviour by moving the board
// //
// //  WARNING:
// //    This file intentionally uses low hand-test thresholds so that launch, coast
// //    and apogee can be triggered by hand motion. Do NOT use this file for a real
// //    rocket flight or parachute deployment. Use the normal flight firmware for
// //    launch.
// //
// //  Apogee detection using decoupled dual Kalman filter
// //  Hardware: ESP32 + BMI160 (I2C) + BMP280 (I2C) + MicroSD (SPI)
// //
// //  Wiring:
// //    BMI160  SDA→GPIO21  SCL→GPIO22  VCC→3.3V
// //    BMP280  SDA→GPIO21  SCL→GPIO22  VCC→5V
// //    SD      CLK→GPIO18  MISO→GPIO19  MOSI→GPIO23  CS→GPIO13
// //
// //  SD output (new file each power-on):
// //    /flight_NNNN.csv
// //    Columns: time_ms, baro_alt_m, baro_pres_pa, baro_temp_c,
// //             imu_ax, imu_ay, imu_az (m/s^2), imu_gx, imu_gy, imu_gz (deg/s),
// //             kf_h_m, kf_v_ms, kf_a_ms2, phase,
// //             apogee_detected, apogee_h_m, apogee_t_ms
// //
// //  Dual-filter Kalman architecture (matches MATLAB v6):
// //    Filter A: [h, v]  updated by BMP280 baro
// //    Filter B: [a]     updated by BMI160 axial accel
// //    Link:     a_est drives height/velocity prediction
// //
// //  Dependencies (install via Arduino Library Manager):
// //    - No BMI160 library needed; BMI160 is read using direct I2C register access
// //    - Adafruit BMP280
// //    - SD (built-in ESP32 Arduino core)
// //    - Wire (built-in)
// // ============================================================================

// #include <Arduino.h>
// #include <Wire.h>
// #include <SD.h>
// #include <SPI.h>
// #include <Adafruit_BMP280.h>

// // ── BMI160 via I2C direct register access ───────────────────────────────────
// // Default I2C address:
// //   SDO/SDIO to GND   => 0x68
// //   SDO/SDIO to VDDIO => 0x69
// // This can also be overridden from PlatformIO using:
// //   build_flags = -D BMI160_I2C_ADDR=0x69
// #ifndef BMI160_I2C_ADDR
// #define BMI160_I2C_ADDR 0x68
// #endif

// #define BMI160_CMD          0x7E
// #define BMI160_ACC_CONF     0x40
// #define BMI160_ACC_RANGE    0x41
// #define BMI160_GYR_CONF     0x42
// #define BMI160_GYR_RANGE    0x43
// #define BMI160_DATA_START   0x0C   // gyro XL, XH, YL, YH, ZL, ZH, acc XL...

// // ── Pin assignments ───────────────────────────────────────────────────────────
// #define SD_CS_PIN           13
// #define I2C_SDA             21
// #define I2C_SCL             22

// // ── Serial monitor settings ───────────────────────────────────────────────────
// // Open Arduino IDE Serial Monitor / PlatformIO monitor at this baud rate.
// #define SERIAL_BAUD         115200
// static const unsigned long SERIAL_TELEM_MS = 250;   // live telemetry interval
// static const unsigned long SERIAL_SD_MS    = 1000;  // SD status interval
// static const unsigned long SERIAL_CAL_MS   = 250;   // calibration progress interval


// // ── Rocket axial accelerometer selection ─────────────────────────────────────
// // The hand-test CSV showed gravity mainly on imu_ax ≈ -9.26 m/s², so the
// // default axial direction is -X. This means the rocket's upward/longitudinal
// // direction is interpreted as -imu.ax.
// //
// // Change these from PlatformIO if your board is mounted differently:
// //   build_flags =
// //       -D AXIAL_AXIS=2       ; 0=X, 1=Y, 2=Z
// //       -D AXIAL_SIGN=1.0f    ; +1.0 or -1.0
// //
// #ifndef AXIAL_AXIS
// #define AXIAL_AXIS 0          // 0=X, 1=Y, 2=Z
// #endif

// #ifndef AXIAL_SIGN
// #define AXIAL_SIGN -1.0f      // hand-test default: use -imu.ax
// #endif

// // ── Hand-test mode ───────────────────────────────────────────────────────────
// // This version is deliberately sensitive so a hand movement can exercise:
// //   PAD -> BOOST -> COAST -> APOGEE DETECTED -> DESCENT
// // Set HAND_TEST_MODE=0 only if you deliberately want the real-flight thresholds
// // while keeping this file's extra diagnostics.
// #ifndef HAND_TEST_MODE
// #define HAND_TEST_MODE 1
// #endif


// // ── Kalman filter parameters (mirrors MATLAB v6) ─────────────────────────────
// static const float G              = 9.80665f;
// static const float BARO_SIGMA_M   = 2.0f;
// static const float ACCEL_SIGMA    = 0.5f;
// static const float Q_H_SIGMA      = 0.3f;
// static const float Q_V_SIGMA      = 1.5f;
// static const float Q_A_PAD        = 0.5f;
// static const float Q_A_BOOST      = 8.0f;
// static const float Q_A_COAST      = 1.5f;
// static const float Q_A_DESCENT    = 1.0f;
// static const float ADAPT_K_BOOST  = 0.08f;
// static const float ADAPT_K_DEF    = 0.03f;
// static const float INNOV_GATE     = 4.0f;
// static const float MACH_DESENSE   = 0.8f;
// static const float V_DESENSE_MS   = 272.0f;
// static const float BARO_MAX_ALT   = 30000.0f;

// #if HAND_TEST_MODE
// // Hand-test thresholds: deliberately low so a quick upward hand movement can
// // simulate BOOST, then slowing/reversal can simulate COAST and APOGEE.
// static const float BOOST_THRESH   = 2.5f;    // m/s² net upward accel to enter BOOST
// static const float COAST_THRESH   = 0.5f;    // m/s² net accel below this exits BOOST
// static const float DESCENT_THRESH = -0.10f;  // m/s; sensitive descent threshold
// static const int   APOGEE_CONFIRM = 5;       // 0.05 s at 100 Hz
// static const float MIN_APOGEE_ALT_M = 0.03f; // 3 cm; allows bench/hand testing
// static const float MIN_APOGEE_TIME_S = 0.15f;
// #else
// // Real-flight thresholds: safer gates for actual rocket use.
// static const float BOOST_THRESH   = 15.0f;
// static const float COAST_THRESH   = 3.0f;
// static const float DESCENT_THRESH = -2.0f;
// static const int   APOGEE_CONFIRM = 10;      // 0.10 s at 100 Hz
// static const float MIN_APOGEE_ALT_M = 20.0f;
// static const float MIN_APOGEE_TIME_S = 1.0f;
// #endif

// static const float ZUPT_NET       = 1.0f;
// static const float ZUPT_ALT       = 1.5f;
// static const float P_H_FLOOR      = 0.01f;
// static const float P_V_FLOOR      = 0.01f;
// static const float P_A_FLOOR      = 0.05f;
// static const int   BARO_SMOOTH_W  = 3;
// static const float PRE_LAUNCH_S   = 1.0f;   // bias estimation window

// // ── Flight phases ─────────────────────────────────────────────────────────────
// enum FlightPhase { PHASE_PAD = 0, PHASE_BOOST, PHASE_COAST, PHASE_DESCENT };

// const char* phase_name(FlightPhase p) {
//     switch (p) {
//         case PHASE_PAD:     return "PAD";
//         case PHASE_BOOST:   return "BOOST";
//         case PHASE_COAST:   return "COAST";
//         case PHASE_DESCENT: return "DESCENT";
//         default:            return "UNKNOWN";
//     }
// }

// // ── Sensor objects ────────────────────────────────────────────────────────────
// Adafruit_BMP280 bmp;

// // ── Global state ──────────────────────────────────────────────────────────────
// // Filter A: [h, v]
// float xA_h = 0.0f, xA_v = 0.0f;
// float PA_hh = 5.0f, PA_hv = 0.0f, PA_vv = 0.5f;

// // Filter B: [a] (scalar)
// float xB_a = 0.0f;
// float PB    = 1.0f;

// // Calibration
// float ground_pressure_pa = 101325.0f;
// float accel_bias_axial   = 0.0f;
// float ground_alt_m       = 0.0f;

// // Phase tracking
// FlightPhase phase      = PHASE_PAD;
// int  neg_vel_count     = 0;
// bool apogee_detected   = false;
// float apogee_h_m       = 0.0f;
// unsigned long apogee_t_ms = 0;

// // Baro smoothing ring buffer
// float baro_buf[BARO_SMOOTH_W] = {0};
// int   baro_buf_idx = 0;

// // Bias estimation
// float accel_sum   = 0.0f;
// int   accel_count = 0;
// bool  bias_ready  = false;

// // SD file
// File  logFile;
// char  filename[32];

// // Timing
// unsigned long last_us     = 0;
// unsigned long boot_ms     = 0;

// // Serial/SD activity tracking
// unsigned long last_serial_telem_ms = 0;
// unsigned long last_serial_sd_ms    = 0;
// unsigned long samples_logged       = 0;
// bool sd_write_ok                  = true;

// // ── BMI160 I2C helpers ────────────────────────────────────────────────────────
// // The active address starts from BMI160_I2C_ADDR but can fall back to the other
// // legal BMI160 address during boot. This preserves your previous config style
// // while making the firmware more tolerant of SDO/SDIO wiring changes.
// uint8_t bmi160_addr = BMI160_I2C_ADDR;

// bool i2c_device_present(uint8_t addr) {
//     Wire.beginTransmission(addr);
//     return (Wire.endTransmission() == 0);
// }

// void i2c_scan_debug() {
//     Serial.print("[BOOT] I2C scan found:");
//     bool found = false;
//     for (uint8_t addr = 1; addr < 127; addr++) {
//         Wire.beginTransmission(addr);
//         if (Wire.endTransmission() == 0) {
//             Serial.printf(" 0x%02X", addr);
//             found = true;
//         }
//     }
//     if (!found) Serial.print(" no devices");
//     Serial.println();
// }

// void bmi160_write(uint8_t reg, uint8_t val) {
//     Wire.beginTransmission(bmi160_addr);
//     Wire.write(reg);
//     Wire.write(val);
//     Wire.endTransmission();
// }

// uint8_t bmi160_read_byte(uint8_t reg) {
//     Wire.beginTransmission(bmi160_addr);
//     Wire.write(reg);
//     Wire.endTransmission(false);
//     Wire.requestFrom((uint8_t)bmi160_addr, (uint8_t)1);
//     return Wire.available() ? Wire.read() : 0;
// }

// void bmi160_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len) {
//     Wire.beginTransmission(bmi160_addr);
//     Wire.write(reg);
//     Wire.endTransmission(false);
//     Wire.requestFrom((uint8_t)bmi160_addr, len);
//     for (uint8_t i = 0; i < len && Wire.available(); i++) {
//         buf[i] = Wire.read();
//     }
// }

// bool bmi160_try_init_at(uint8_t addr) {
//     bmi160_addr = addr;

//     Serial.printf("[BOOT]   Trying BMI160 address 0x%02X... ", addr);

//     if (!i2c_device_present(addr)) {
//         Serial.println("no ACK");
//         return false;
//     }

//     uint8_t id_before = bmi160_read_byte(0x00);
//     Serial.printf("ACK, chip ID before reset=0x%02X... ", id_before);

//     if (id_before != 0xD1) {
//         Serial.println("not BMI160");
//         return false;
//     }

//     // Soft reset
//     bmi160_write(BMI160_CMD, 0xB6);
//     delay(100);

//     // Power up accelerometer (normal mode)
//     bmi160_write(BMI160_CMD, 0x11);
//     delay(50);

//     // Power up gyroscope (normal mode)
//     bmi160_write(BMI160_CMD, 0x15);
//     delay(100);

//     // Accel: ODR=100Hz, range=±16g
//     bmi160_write(BMI160_ACC_CONF,  0x28);
//     bmi160_write(BMI160_ACC_RANGE, 0x0C);

//     // Gyro: ODR=100Hz, range=±2000 deg/s
//     bmi160_write(BMI160_GYR_CONF,  0x28);
//     bmi160_write(BMI160_GYR_RANGE, 0x00);

//     uint8_t id_after = bmi160_read_byte(0x00);
//     if (id_after == 0xD1) {
//         Serial.printf("OK, chip ID after init=0x%02X\n", id_after);
//         return true;
//     }

//     Serial.printf("failed after init, chip ID=0x%02X\n", id_after);
//     return false;
// }

// bool bmi160_init() {
//     Serial.printf("[BOOT] BMI160_I2C_ADDR default = 0x%02X\n", BMI160_I2C_ADDR);

//     const uint8_t candidates[] = {
//         (uint8_t)BMI160_I2C_ADDR,
//         (uint8_t)0x68,
//         (uint8_t)0x69
//     };

//     for (size_t i = 0; i < sizeof(candidates); i++) {
//         bool already_tried = false;
//         for (size_t j = 0; j < i; j++) {
//             if (candidates[j] == candidates[i]) {
//                 already_tried = true;
//                 break;
//             }
//         }
//         if (already_tried) continue;

//         if (bmi160_try_init_at(candidates[i])) {
//             return true;
//         }
//     }

//     return false;
// }

// struct BMI160Data {
//     float ax, ay, az;   // m/s^2
//     float gx, gy, gz;   // deg/s
// };

// BMI160Data bmi160_read() {
//     uint8_t raw[12];
//     // Register 0x0C: GYR_X_L, GYR_X_H, Y, Z, then ACC_X_L, ACC_X_H, Y, Z
//     bmi160_read_bytes(0x0C, raw, 12);

//     // Gyroscope: ±2000 dps, 16-bit, LSB = 16.4 LSB/(deg/s)
//     int16_t gx_raw = (int16_t)((raw[1]  << 8) | raw[0]);
//     int16_t gy_raw = (int16_t)((raw[3]  << 8) | raw[2]);
//     int16_t gz_raw = (int16_t)((raw[5]  << 8) | raw[4]);

//     // Accelerometer: ±16g, 16-bit, LSB = 2048 LSB/g
//     int16_t ax_raw = (int16_t)((raw[7]  << 8) | raw[6]);
//     int16_t ay_raw = (int16_t)((raw[9]  << 8) | raw[8]);
//     int16_t az_raw = (int16_t)((raw[11] << 8) | raw[10]);

//     static const float ACC_SCALE = (16.0f * G) / 32768.0f;  // ±16g range
//     static const float GYR_SCALE = 2000.0f / 32768.0f;       // ±2000 dps

//     BMI160Data d;
//     d.ax = ax_raw * ACC_SCALE;
//     d.ay = ay_raw * ACC_SCALE;
//     d.az = az_raw * ACC_SCALE;
//     d.gx = gx_raw * GYR_SCALE;
//     d.gy = gy_raw * GYR_SCALE;
//     d.gz = gz_raw * GYR_SCALE;
//     return d;
// }

// // ── Axial acceleration helper ────────────────────────────────────────────────
// float get_axial_accel(const BMI160Data &imu) {
// #if AXIAL_AXIS == 0
//     return ((float)AXIAL_SIGN) * imu.ax;
// #elif AXIAL_AXIS == 1
//     return ((float)AXIAL_SIGN) * imu.ay;
// #elif AXIAL_AXIS == 2
//     return ((float)AXIAL_SIGN) * imu.az;
// #else
//     #error "AXIAL_AXIS must be 0 for X, 1 for Y, or 2 for Z"
// #endif
// }

// char axial_axis_letter() {
// #if AXIAL_AXIS == 0
//     return 'X';
// #elif AXIAL_AXIS == 1
//     return 'Y';
// #elif AXIAL_AXIS == 2
//     return 'Z';
// #else
//     return '?';
// #endif
// }

// const char* axial_axis_sign() {
//     return (((float)AXIAL_SIGN) >= 0.0f) ? "+" : "-";
// }

// // ── Serial monitor helpers ───────────────────────────────────────────────────
// void serial_banner() {
//     Serial.println();
//     Serial.println("============================================================");
//     Serial.println("  Kalman Apogee Detector v6 - ESP32 Serial Monitor");
//     Serial.println("============================================================");
//     Serial.printf("Baud: %lu | Loop target: 100 Hz | Telemetry: every %.2f s\n",
//                   (unsigned long)SERIAL_BAUD, SERIAL_TELEM_MS / 1000.0f);
//     Serial.println("CSV is still written to SD. Serial is only for live status.");
// #if HAND_TEST_MODE
//     Serial.println("MODE: HAND TEST ONLY - low thresholds enabled. DO NOT USE FOR REAL FLIGHT.");
//     Serial.println("Test motion: keep still during CAL, move board upward quickly, then slow/reverse.");
// #else
//     Serial.println("MODE: REAL-FLIGHT THRESHOLDS");
// #endif
//     Serial.printf("Axial accelerometer = %s%c axis | net_accel = axial - bias - g\n",
//                   axial_axis_sign(), axial_axis_letter());
//     Serial.printf("Thresholds: BOOST>%.2f m/s^2 | COAST<%.2f m/s^2 | APOGEE h>%.2f m | confirm=%d samples\n",
//                   BOOST_THRESH, COAST_THRESH, MIN_APOGEE_ALT_M, APOGEE_CONFIRM);
//     Serial.println();
// }

// void serial_phase_change(FlightPhase old_phase, FlightPhase new_phase,
//                          unsigned long t_ms, float h, float v, float a) {
//     Serial.printf("\n>>> PHASE CHANGE: %s -> %s at %.3f s | h=%.2f m, v=%.2f m/s, a=%.2f m/s^2\n",
//                   phase_name(old_phase), phase_name(new_phase),
//                   t_ms / 1000.0f, h, v, a);
// }

// void serial_activity(unsigned long t_ms, float baro_alt, float pres_pa, float temp_c,
//                      const BMI160Data &imu, float axial_accel, float net_accel,
//                      float approx_mach, bool baro_ok, bool apogee_allowed) {
//     Serial.printf("t=%7.3fs | %-7s | h=%8.2f m | v=%8.2f m/s | aKF=%7.2f | axial=%7.2f | net=%7.2f | baro=%8.2f m | P=%9.2f Pa | T=%5.1f C | Mach~%.2f | SD=%lu | %s | %s | neg=%d/%d%s\n",
//                   t_ms / 1000.0f,
//                   phase_name(phase),
//                   xA_h, xA_v, xB_a, axial_accel, net_accel,
//                   baro_alt, pres_pa, temp_c, approx_mach,
//                   samples_logged,
//                   baro_ok ? "BARO_OK" : "BARO_SKIPPED",
//                   apogee_allowed ? "APOGEE_ARMED" : "APOGEE_BLOCKED",
//                   neg_vel_count, APOGEE_CONFIRM,
//                   apogee_detected ? " | APOGEE_LOCKED" : "");
// }

// void serial_sd_status(unsigned long t_ms) {
//     Serial.printf("[SD] %.3f s | file=%s | rows=%lu | last_write=%s\n",
//                   t_ms / 1000.0f, filename, samples_logged,
//                   sd_write_ok ? "OK" : "FAILED");
// }

// // ── Pressure → altitude (ISA model) ─────────────────────────────────────────
// float pressure_to_alt(float pressure_pa) {
//     return 44330.0f * (1.0f - powf(pressure_pa / ground_pressure_pa, 0.1903f));
// }

// // ── Baro smoothing (causal ring buffer) ──────────────────────────────────────
// float baro_smooth_push(float new_val) {
//     baro_buf[baro_buf_idx] = new_val;
//     baro_buf_idx = (baro_buf_idx + 1) % BARO_SMOOTH_W;
//     float sum = 0;
//     for (int i = 0; i < BARO_SMOOTH_W; i++) sum += baro_buf[i];
//     return sum / BARO_SMOOTH_W;
// }

// // ── SD: find next available filename ─────────────────────────────────────────
// void find_next_filename() {
//     for (int n = 1; n <= 9999; n++) {
//         snprintf(filename, sizeof(filename), "/flight_%04d.csv", n);
//         if (!SD.exists(filename)) break;
//     }
// }

// // ── Write CSV header ──────────────────────────────────────────────────────────
// void write_csv_header() {
//     logFile.println(
//         "time_ms,"
//         "baro_alt_m,baro_pres_pa,baro_temp_c,"
//         "imu_ax_ms2,imu_ay_ms2,imu_az_ms2,"
//         "imu_gx_dps,imu_gy_dps,imu_gz_dps,"
//         "axial_accel_ms2,net_accel_ms2,"
//         "kf_h_m,kf_v_ms,kf_a_ms2,"
//         "phase,"
//         "apogee_allowed,neg_vel_count,"
//         "apogee_detected,apogee_h_m,apogee_t_ms"
//     );
//     logFile.flush();
// }

// // ── Kalman update: Filter B (acceleration, scalar) ───────────────────────────
// void filterB_update(float net_accel, float Q_a) {
//     // Predict (random walk)
//     PB += Q_a * Q_a;
//     if (PB < P_A_FLOOR) PB = P_A_FLOOR;

//     // Update
//     float R_a = ACCEL_SIGMA * ACCEL_SIGMA;
//     float K   = PB / (PB + R_a);
//     xB_a      = xB_a + K * (net_accel - xB_a);
//     // Joseph form (scalar): P = (1-K)^2*P + K^2*R
//     PB        = (1.0f - K) * (1.0f - K) * PB + K * K * R_a;
// }

// // ── Kalman update: Filter A (height/velocity, 2-state) ───────────────────────
// void filterA_predict(float dt) {
//     float dt2 = dt * dt;
//     float a   = xB_a;  // use latest acceleration estimate from Filter B

//     // Predict state: x_new = F*x + B*a
//     float h_new = xA_h + xA_v * dt + 0.5f * a * dt2;
//     float v_new = xA_v + a * dt;
//     xA_h = h_new;
//     xA_v = v_new;

//     // Predict covariance: P_new = F*P*F' + Q
//     // F = [[1,dt],[0,1]]
//     // F*P*F' = [[P_hh + 2*dt*P_hv + dt^2*P_vv,  P_hv + dt*P_vv],
//     //           [P_hv + dt*P_vv,                 P_vv           ]]
//     float Q_h2 = Q_H_SIGMA * Q_H_SIGMA * dt2;
//     float Q_v2 = Q_V_SIGMA * Q_V_SIGMA * dt2;

//     float new_hh = PA_hh + 2.0f * dt * PA_hv + dt2 * PA_vv + Q_h2;
//     float new_hv = PA_hv + dt * PA_vv;
//     float new_vv = PA_vv + Q_v2;

//     PA_hh = fmaxf(new_hh, P_H_FLOOR);
//     PA_hv = new_hv;
//     PA_vv = fmaxf(new_vv, P_V_FLOOR);
// }

// void filterA_zupt() {
//     // Zero velocity update: H = [0,1], measurement = 0
//     float R_z = 0.05f;
//     float S_z = PA_vv + R_z;
//     float K_h = PA_hv / S_z;
//     float K_v = PA_vv / S_z;
//     float innov = 0.0f - xA_v;

//     xA_h += K_h * innov;
//     xA_v += K_v * innov;

//     // Joseph form: H = [0,1]
//     // (I - K*H) = [[1, -K_h],[0, 1-K_v]]
//     float IKH_hh = 1.0f,   IKH_hv = -K_h;
//     float IKH_vh = 0.0f,   IKH_vv = 1.0f - K_v;

//     float tmp_hh = IKH_hh * PA_hh + IKH_hv * PA_hv;
//     float tmp_hv = IKH_hh * PA_hv + IKH_hv * PA_vv;
//     float tmp_vh = IKH_vh * PA_hh + IKH_vv * PA_hv;  // = 0 + (1-Kv)*P_hv
//     float tmp_vv = IKH_vh * PA_hv + IKH_vv * PA_vv;

//     PA_hh = tmp_hh * IKH_hh + tmp_hv * IKH_hv + K_h * K_h * R_z;
//     PA_hv = tmp_hh * IKH_vh + tmp_hv * IKH_vv + K_h * K_v * R_z;
//     PA_vv = tmp_vh * IKH_vh + tmp_vv * IKH_vv + K_v * K_v * R_z;

//     PA_hh = fmaxf(PA_hh, P_H_FLOOR);
//     PA_vv = fmaxf(PA_vv, P_V_FLOOR);
// }

// void filterA_baro_update(float baro_alt, float R_baro) {
//     // H = [1, 0]: observe height only
//     float S     = PA_hh + R_baro;
//     float K_h   = PA_hh / S;
//     float K_v   = PA_hv / S;
//     float innov = baro_alt - xA_h;

//     // Innovation gate
//     float gate_limit = INNOV_GATE * sqrtf(S);
//     if (xA_h > 1.0f && fabsf(innov) > gate_limit) return;  // reject

//     xA_h += K_h * innov;
//     xA_v += K_v * innov;

//     // Joseph form: H = [1,0]
//     // (I - K*H) = [[1-K_h, 0],[-K_v, 1]]
//     float IKH_hh = 1.0f - K_h, IKH_hv = 0.0f;
//     float IKH_vh = -K_v,       IKH_vv = 1.0f;

//     float tmp_hh = IKH_hh * PA_hh;
//     float tmp_hv = IKH_hh * PA_hv;
//     float tmp_vh = IKH_vh * PA_hh + PA_hv;   // IKH_vh*P_hh + IKH_vv*P_hv
//     float tmp_vv = IKH_vh * PA_hv + PA_vv;

//     PA_hh = tmp_hh * IKH_hh + K_h * K_h * R_baro;
//     PA_hv = tmp_hh * IKH_vh + K_h * K_v * R_baro;
//     PA_vv = tmp_vh * IKH_vh + tmp_vv + K_v * K_v * R_baro;

//     PA_hh = fmaxf(PA_hh, P_H_FLOOR);
//     PA_vv = fmaxf(PA_vv, P_V_FLOOR);
// }

// // ── setup() ──────────────────────────────────────────────────────────────────
// void setup() {
//     Serial.begin(SERIAL_BAUD);
//     delay(500);
//     serial_banner();

//     Serial.println("[BOOT] Initialising I2C bus...");

//     // I2C
//     Wire.begin(I2C_SDA, I2C_SCL);
//     Wire.setClock(400000);
//     Serial.printf("[BOOT] I2C ready: SDA=GPIO%d, SCL=GPIO%d, clock=400 kHz\n", I2C_SDA, I2C_SCL);
//     i2c_scan_debug();

//     // BMI160
//     Serial.println("[BOOT] BMI160 init...");
//     if (!bmi160_init()) {
//         Serial.println("[BOOT] BMI160 FAILED! Check SDA/SCL, 3.3V power, GND, and SDO/SDIO address strap.");
//         while (1) delay(1000);
//     }
//     Serial.printf("[BOOT] BMI160 OK (chip ID 0x%02X, active address 0x%02X)\n", bmi160_read_byte(0x00), bmi160_addr);

//     // BMP280
//     Serial.print("[BOOT] BMP280 init... ");
//     uint8_t bmp_addr = 0x76;
//     if (!bmp.begin(0x76)) {          // try 0x76 first, then 0x77
//         bmp_addr = 0x77;
//         if (!bmp.begin(0x77)) {
//             Serial.println("FAILED! Check wiring / SDO address.");
//             while (1) delay(1000);
//         }
//     }
//     bmp.setSampling(
//         Adafruit_BMP280::MODE_NORMAL,
//         Adafruit_BMP280::SAMPLING_X4,   // temp oversampling
//         Adafruit_BMP280::SAMPLING_X16,  // pressure oversampling (high accuracy)
//         Adafruit_BMP280::FILTER_X4,     // IIR filter
//         Adafruit_BMP280::STANDBY_MS_1   // 1ms standby (fast ODR)
//     );
//     Serial.printf("OK (address 0x%02X)\n", bmp_addr);

//     // SD card
//     Serial.print("[BOOT] SD card init... ");
//     SPI.begin(18, 19, 23, SD_CS_PIN);
//     if (!SD.begin(SD_CS_PIN)) {
//         Serial.println("FAILED! Check wiring/card.");
//         while (1) delay(1000);
//     }
//     Serial.printf("OK (CS=GPIO%d, CLK=18, MISO=19, MOSI=23)\n", SD_CS_PIN);

//     // Find unique filename
//     find_next_filename();
//     logFile = SD.open(filename, FILE_WRITE);
//     if (!logFile) {
//         Serial.printf("Cannot open %s\n", filename);
//         while (1) delay(1000);
//     }
//     write_csv_header();
//     Serial.printf("[SD] Logging to: %s\n", filename);

//     // Calibration: sample ground pressure and accel bias for PRE_LAUNCH_S
//     Serial.printf("[CAL] Calibrating for %.1f s. Keep rocket still and vertical...\n", PRE_LAUNCH_S);
//     float pres_sum = 0;
//     int   pres_cnt = 0;
//     unsigned long cal_start = millis();
//     unsigned long cal_end = cal_start + (unsigned long)(PRE_LAUNCH_S * 1000);
//     unsigned long last_cal_print = 0;

//     // Pre-fill baro buffer
//     float first_p = bmp.readPressure();
//     for (int i = 0; i < BARO_SMOOTH_W; i++) baro_buf[i] = 44330.0f; // placeholder

//     while (millis() < cal_end) {
//         float p = bmp.readPressure();
//         pres_sum += p;
//         pres_cnt++;

//         BMI160Data imu = bmi160_read();
//         float axial_accel = get_axial_accel(imu);
//         accel_sum += axial_accel;
//         accel_count++;

//         unsigned long now_ms = millis();
//         if (now_ms - last_cal_print >= SERIAL_CAL_MS) {
//             last_cal_print = now_ms;
//             float elapsed_s = (now_ms - cal_start) / 1000.0f;
//             Serial.printf("[CAL] %.2f / %.2f s | samples: pressure=%d, accel=%d\n",
//                           elapsed_s, PRE_LAUNCH_S, pres_cnt, accel_count);
//         }

//         delay(10);
//     }

//     ground_pressure_pa = pres_sum / pres_cnt;
//     ground_alt_m       = 0.0f;  // reference is ground

//     float mean_axial = accel_sum / accel_count;
//     accel_bias_axial = mean_axial - G;   // axial should read +G at rest when rocket points up
//     bias_ready = true;

//     // Init baro smoothing buffer with ground altitude
//     for (int i = 0; i < BARO_SMOOTH_W; i++) baro_buf[i] = 0.0f;
//     baro_buf_idx = 0;

//     Serial.printf("[CAL] Done. Ground P=%.1f Pa | axial=%s%c | mean_axial=%.4f m/s^2 | accel_bias=%.4f m/s^2 | samples: pressure=%d, accel=%d\n",
//                   ground_pressure_pa, axial_axis_sign(), axial_axis_letter(),
//                   mean_axial, accel_bias_axial, pres_cnt, accel_count);
// #if HAND_TEST_MODE
//     Serial.printf("[READY] HAND TEST: move upward sharply to enter BOOST, then slow/reverse to trigger apogee. BOOST threshold = %.2f m/s^2.\n", BOOST_THRESH);
// #else
//     Serial.printf("[READY] Waiting for launch. BOOST threshold = %.1f m/s^2 net axial accel.\n", BOOST_THRESH);
// #endif
//     Serial.printf("[READY] Apogee gated: phase=COAST, altitude>%.2f m, confirm=%d samples, min time=%.2f s.\n",
//                   MIN_APOGEE_ALT_M, APOGEE_CONFIRM, MIN_APOGEE_TIME_S);

//     boot_ms  = millis();
//     last_us  = micros();
// }

// // ── loop() ───────────────────────────────────────────────────────────────────
// void loop() {
//     unsigned long now_us  = micros();
//     float dt              = (now_us - last_us) * 1e-6f;
//     last_us               = now_us;
//     if (dt <= 0 || dt > 0.5f) dt = 0.01f;  // guard

//     unsigned long t_ms = millis() - boot_ms;

//     // ── Read BMP280 ─────────────────────────────────────────────────────────
//     float pres_pa  = bmp.readPressure();
//     float temp_c   = bmp.readTemperature();
//     float baro_raw = pressure_to_alt(pres_pa);
//     float baro_alt = baro_smooth_push(baro_raw);   // causal 3-sample average

//     // ── Read BMI160 ─────────────────────────────────────────────────────────
//     BMI160Data imu = bmi160_read();
//     float axial_accel = get_axial_accel(imu);
//     float axial_debiased = axial_accel - accel_bias_axial;
//     float net_accel = axial_debiased - G;          // signed vertical net acceleration

//     // ── Approximate Mach (ISA at sea level, ignore altitude correction) ──────
//     float approx_mach = fabsf(xA_v) / 340.3f;

//     // ── Phase state machine ──────────────────────────────────────────────────
//     FlightPhase prev_phase = phase;
//     switch (phase) {
//         case PHASE_PAD:
//             if (net_accel > BOOST_THRESH)   phase = PHASE_BOOST;
//             break;
//         case PHASE_BOOST:
//             if (net_accel < COAST_THRESH)   phase = PHASE_COAST;
//             break;
//         case PHASE_COAST:
//             if (xA_v < DESCENT_THRESH)      phase = PHASE_DESCENT;
//             break;
//         case PHASE_DESCENT:
//             break;
//     }
//     if (phase != prev_phase) {
//         serial_phase_change(prev_phase, phase, t_ms, xA_h, xA_v, net_accel);
//     }

//     // ── Q_A selection ────────────────────────────────────────────────────────
//     float Q_a;
//     switch (phase) {
//         case PHASE_PAD:      Q_a = Q_A_PAD;     break;
//         case PHASE_BOOST:    Q_a = Q_A_BOOST;   break;
//         case PHASE_COAST:    Q_a = Q_A_COAST;   break;
//         case PHASE_DESCENT:  Q_a = Q_A_DESCENT; break;
//         default:             Q_a = Q_A_COAST;
//     }

//     // ── Filter B: Acceleration ───────────────────────────────────────────────
//     filterB_update(net_accel, Q_a);

//     // ── Filter A: Predict ────────────────────────────────────────────────────
//     filterA_predict(dt);

//     // ── ZUPT: on pad only ────────────────────────────────────────────────────
//     if (phase == PHASE_PAD &&
//         fabsf(net_accel) < ZUPT_NET &&
//         xA_h < ZUPT_ALT) {
//         filterA_zupt();
//     }

//     // ── Baro update (conditional) ────────────────────────────────────────────
//     bool baro_ok = (approx_mach < MACH_DESENSE) &&
//                    (fabsf(xA_v) < V_DESENSE_MS) &&
//                    (xA_h < BARO_MAX_ALT);

//     if (baro_ok) {
//         float adapt_k  = (phase == PHASE_BOOST) ? ADAPT_K_BOOST : ADAPT_K_DEF;
//         float R_baro   = BARO_SIGMA_M * BARO_SIGMA_M *
//                          (1.0f + adapt_k * fabsf(net_accel));
//         filterA_baro_update(baro_alt, R_baro);
//     }

//     // ── Apogee detection (confirmed zero-crossing) ───────────────────────────
//     // Important: apogee is only allowed after launch, in COAST, above a minimum
//     // altitude. This prevents false apogee during hand tests or pad noise.
//     bool apogee_allowed = (phase == PHASE_COAST) &&
//                           (xA_h > MIN_APOGEE_ALT_M) &&
//                           (t_ms > (unsigned long)(MIN_APOGEE_TIME_S * 1000.0f));

//     if (!apogee_detected && apogee_allowed) {
//         if (xA_v < 0.0f) {
//             neg_vel_count++;
//             if (neg_vel_count >= APOGEE_CONFIRM) {
//                 apogee_detected = true;
//                 apogee_h_m      = xA_h;
//                 apogee_t_ms     = t_ms;
//                 phase           = PHASE_DESCENT;
//                 Serial.printf("\n*** APOGEE DETECTED: %.2f m at %.3f s | phase=%s | confirm=%d samples | rows=%lu ***\n",
//                               apogee_h_m, apogee_t_ms / 1000.0f, phase_name(phase),
//                               APOGEE_CONFIRM, samples_logged);
//             }
//         } else {
//             neg_vel_count = 0;
//         }
//     } else if (!apogee_allowed) {
//         neg_vel_count = 0;
//     }

//     // ── SD logging ───────────────────────────────────────────────────────────
//     // Format: time_ms, baro_alt, pres, temp, ax, ay, az, gx, gy, gz,
//     //         kf_h, kf_v, kf_a, phase, apogee_det, apogee_h, apogee_t
//     char line[320];
//     snprintf(line, sizeof(line),
//         "%lu,"           // time_ms
//         "%.3f,%.2f,%.2f,"   // baro_alt_m, pres_pa, temp_c
//         "%.4f,%.4f,%.4f,"   // imu ax ay az (m/s^2)
//         "%.3f,%.3f,%.3f,"   // imu gx gy gz (dps)
//         "%.4f,%.4f,"        // axial_accel, net_accel
//         "%.3f,%.3f,%.3f,"   // kf h v a
//         "%d,"               // phase
//         "%d,%d,"            // apogee_allowed, neg_vel_count
//         "%d,%.3f,%lu",      // apogee_detected, apogee_h_m, apogee_t_ms
//         t_ms,
//         baro_alt, pres_pa, temp_c,
//         imu.ax, imu.ay, imu.az,
//         imu.gx, imu.gy, imu.gz,
//         axial_accel, net_accel,
//         xA_h, xA_v, xB_a,
//         (int)phase,
//         (int)apogee_allowed, neg_vel_count,
//         (int)apogee_detected, apogee_h_m, apogee_t_ms
//     );
//     size_t bytes_written = logFile.println(line);
//     sd_write_ok = (bytes_written > 0);
//     samples_logged++;
//     if (!sd_write_ok) {
//         Serial.printf("[SD ERROR] Write failed at t=%.3f s | file=%s\n", t_ms / 1000.0f, filename);
//     }

//     // Flush every 50 samples (~0.5 s at 100 Hz) to limit SD write latency
//     static int flush_count = 0;
//     if (++flush_count >= 50) {
//         logFile.flush();
//         flush_count = 0;
//     }

//     // ── Serial monitor live activity ─────────────────────────────────────────
//     if (t_ms - last_serial_telem_ms >= SERIAL_TELEM_MS) {
//         last_serial_telem_ms = t_ms;
//         serial_activity(t_ms, baro_alt, pres_pa, temp_c, imu,
//                         axial_accel, net_accel, approx_mach, baro_ok, apogee_allowed);
//     }

//     if (t_ms - last_serial_sd_ms >= SERIAL_SD_MS) {
//         last_serial_sd_ms = t_ms;
//         serial_sd_status(t_ms);
//     }

//     // ── Loop timing: target 100 Hz ────────────────────────────────────────────
//     // BMP280 at 25Hz internally; filter runs at IMU rate (100Hz)
//     static unsigned long next_loop_us = 0;
//     if (next_loop_us == 0) next_loop_us = micros();
//     next_loop_us += 10000;  // 10 ms = 100 Hz
//     long wait_us = (long)(next_loop_us - micros());
//     if (wait_us > 0) delayMicroseconds((uint32_t)wait_us);
// }