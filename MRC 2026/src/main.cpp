// // ============================================================================
// //  kalman_apogee.cpp
// //  Apogee detection using decoupled dual Kalman filter
// //  Hardware: ESP32 + BMI160 (I2C) + BMP280 (I2C) + MicroSD (SPI)
// //
// //  Wiring:
// //    BMI160  SDA→GPIO21  SCL→GPIO22  VCC→3.3V
// //    BMP280  SDA→GPIO21  SCL→GPIO22  VCC→5V
// //    SD      CLK→GPIO18  MISO→GPIO19  MOSI→GPIO23  CS→GPIO13
// //    Red LED GPIO32→series resistor→LED anode; LED cathode→GND
// //    Blue LED GPIO33→series resistor→LED anode; LED cathode→GND
// //
// //  LED indications:
// //    Red solid  = initialising / calibrating
// //    Blue solid = armed and sampling acceleration at 100 Hz
// //    Red + blue flashing together = previous launch data is still in EEPROM;
// //                                  type clr in the Serial Monitor before launch
// //    Red blinks = error code (count the flashes between long pauses)
// //      1 = EEPROM unavailable (nonfatal; system can still arm)
// //      2 = BMI160 initialisation failed (fatal)
// //      3 = BMP280 initialisation failed (fatal)
// //      4 = SD unavailable/write failed (nonfatal; EEPROM remains primary)
// //      5 = calibration data invalid (fatal)
// //
// //  Serial Monitor commands (115200 baud, send with Enter/newline):
// //    clr  = clear previous EEPROM records while still on the pad
// //    view = display the EEPROM flight-event records again
// //
// //  Serial output is event-driven. Live 250 ms telemetry is deliberately
// //  disabled so the monitor shows the previous-flight summary, armed status,
// //  important state changes, apogee and errors without constant scrolling.
// //
// //  Event logging (new SD file each power-on + EEPROM ring buffer):
// //    /flight_NNNN.csv
// //    Columns: time_since_arm_ms, time_since_arm_s, altitude_m,
// //             flight_state, event
// //
// //  Sensor data is NOT continuously written to the SD card. Only the following
// //  safety-critical events are committed to both EEPROM and SD:
// //    - avionics armed
// //    - launch detected / BOOST
// //    - each later flight-state transition
// //    - confirmed apogee / DESCENT
// //
// //  EEPROM is written first. The SD file is opened, appended, flushed and closed
// //  for every event, so an SD failure cannot stop the flight-state machine and
// //  cannot erase the EEPROM copy.
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
// #include <EEPROM.h>
// #include <cmath>

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

// // LED pins can be overridden from PlatformIO build_flags if required:
// //   -D RED_LED_PIN=32 -D BLUE_LED_PIN=33
// #ifndef RED_LED_PIN
// #define RED_LED_PIN         32
// #endif

// #ifndef BLUE_LED_PIN
// #define BLUE_LED_PIN        33
// #endif

// // Set either value to 0 if an LED is wired active-low instead of active-high.
// #ifndef RED_LED_ACTIVE_HIGH
// #define RED_LED_ACTIVE_HIGH 1
// #endif

// #ifndef BLUE_LED_ACTIVE_HIGH
// #define BLUE_LED_ACTIVE_HIGH 1
// #endif

// // ── Status/error LED codes ──────────────────────────────────────────────────
// enum LedErrorCode : uint8_t {
//     LED_ERROR_NONE        = 0,
//     LED_ERROR_EEPROM      = 1,
//     LED_ERROR_BMI160      = 2,
//     LED_ERROR_BMP280      = 3,
//     LED_ERROR_SD          = 4,
//     LED_ERROR_CALIBRATION = 5
// };

// static const unsigned long LED_PULSE_ON_MS   = 200;
// static const unsigned long LED_PULSE_OFF_MS  = 200;
// static const unsigned long LED_GROUP_GAP_MS  = 1200;
// static const unsigned long OLD_FLIGHT_FLASH_MS = 400;

// uint8_t led_warning_code = LED_ERROR_NONE;
// bool    avionics_armed   = false;
// bool    previous_launch_data_present = false;
// bool    current_launch_started = false;
// unsigned long led_pattern_start_ms = 0;

// void led_write(uint8_t pin, bool active_high, bool on) {
//     digitalWrite(pin, (on == active_high) ? HIGH : LOW);
// }

// void led_begin_initialising() {
//     pinMode(RED_LED_PIN, OUTPUT);
//     pinMode(BLUE_LED_PIN, OUTPUT);
//     led_write(RED_LED_PIN, RED_LED_ACTIVE_HIGH, true);
//     led_write(BLUE_LED_PIN, BLUE_LED_ACTIVE_HIGH, false);
// }

// // Retain the first nonfatal warning so its blink code remains unambiguous.
// void led_set_warning(uint8_t code) {
//     if (code == LED_ERROR_NONE || led_warning_code != LED_ERROR_NONE) return;
//     led_warning_code = code;
//     led_pattern_start_ms = millis();
//     Serial.printf("[LED] Red warning pattern set to error code %u.\n", code);
// }

// // Called only after setup/calibration is complete and loop() is ready to start.
// void led_set_armed() {
//     avionics_armed = true;
//     led_pattern_start_ms = millis();
//     led_write(BLUE_LED_PIN, BLUE_LED_ACTIVE_HIGH, true);
//     led_write(RED_LED_PIN, RED_LED_ACTIVE_HIGH, false);
// }

// // Nonblocking warning pattern so LED activity never delays the 100 Hz loop.
// void led_update() {
//     if (!avionics_armed) {
//         led_write(RED_LED_PIN, RED_LED_ACTIVE_HIGH, true);
//         led_write(BLUE_LED_PIN, BLUE_LED_ACTIVE_HIGH, false);
//         return;
//     }

//     // A valid BOOST/COAST/DESCENT event from an earlier power cycle is treated
//     // as retained launch data. Flash both LEDs together until clr is accepted
//     // or until the current launch begins. This pattern has priority over the
//     // normal armed and nonfatal-warning patterns while still on the pad.
//     if (previous_launch_data_present && !current_launch_started) {
//         bool flash_on = ((millis() - led_pattern_start_ms) /
//                          OLD_FLIGHT_FLASH_MS) % 2UL == 0UL;
//         led_write(RED_LED_PIN, RED_LED_ACTIVE_HIGH, flash_on);
//         led_write(BLUE_LED_PIN, BLUE_LED_ACTIVE_HIGH, flash_on);
//         return;
//     }

//     led_write(BLUE_LED_PIN, BLUE_LED_ACTIVE_HIGH, true);

//     if (led_warning_code == LED_ERROR_NONE) {
//         led_write(RED_LED_PIN, RED_LED_ACTIVE_HIGH, false);
//         return;
//     }

//     const unsigned long pulse_pair_ms = LED_PULSE_ON_MS + LED_PULSE_OFF_MS;
//     const unsigned long pulses_ms = (unsigned long)led_warning_code * pulse_pair_ms;
//     const unsigned long group_ms = pulses_ms + LED_GROUP_GAP_MS;
//     unsigned long position_ms = (millis() - led_pattern_start_ms) % group_ms;

//     bool red_on = false;
//     if (position_ms < pulses_ms) {
//         red_on = (position_ms % pulse_pair_ms) < LED_PULSE_ON_MS;
//     }
//     led_write(RED_LED_PIN, RED_LED_ACTIVE_HIGH, red_on);
// }

// // Fatal sensor/calibration faults cannot safely enter the flight loop.
// // Blink the code forever; delay() yields to the ESP32 watchdog.
// void led_halt_with_error(uint8_t code, const char *message) {
//     avionics_armed = false;
//     led_write(BLUE_LED_PIN, BLUE_LED_ACTIVE_HIGH, false);
//     Serial.printf("[FATAL] %s | RED LED error code %u\n", message, code);

//     while (true) {
//         for (uint8_t flash = 0; flash < code; flash++) {
//             led_write(RED_LED_PIN, RED_LED_ACTIVE_HIGH, true);
//             delay(LED_PULSE_ON_MS);
//             led_write(RED_LED_PIN, RED_LED_ACTIVE_HIGH, false);
//             delay(LED_PULSE_OFF_MS);
//         }
//         delay(LED_GROUP_GAP_MS);
//     }
// }

// // ── Serial monitor settings ───────────────────────────────────────────────────
// // Open Arduino IDE Serial Monitor / PlatformIO monitor at this baud rate.
// #define SERIAL_BAUD         115200

// // ── EEPROM flight-state event log ───────────────────────────────────────────
// // ESP32 EEPROM is flash-backed. To reduce flash wear, records are written only
// // for significant events (boot/ready, phase changes and apogee), not every loop.
// static const size_t   EEPROM_BYTES        = 4096;
// static const uint32_t EEPROM_LOG_MAGIC    = 0x46535445UL; // ASCII "FSTE"
// static const uint16_t EEPROM_LOG_VERSION  = 1;
// static const uint16_t EEPROM_MAX_RECORDS  = 50;

// #pragma pack(push, 1)
// struct EepromLogHeader {
//     uint32_t magic;
//     uint16_t version;
//     uint16_t record_size;
//     uint16_t max_records;
//     uint16_t write_index;
//     uint16_t count;
//     uint16_t reserved;
// };

// struct EepromFlightRecord {
//     uint32_t time_ms;
//     float    altitude_m;
//     uint8_t  flight_state;
//     char     action[24];
//     char     flight_file[32];
//     uint8_t  checksum;
// };
// #pragma pack(pop)

// static_assert(sizeof(EepromLogHeader) +
//               EEPROM_MAX_RECORDS * sizeof(EepromFlightRecord) <= EEPROM_BYTES,
//               "EEPROM log exceeds configured EEPROM size");

// EepromLogHeader eeprom_header;
// bool eeprom_ready = false;


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
// static const float BOOST_THRESH   = 15.0f;
// static const float COAST_THRESH   = 3.0f;
// static const int   LAUNCH_CONFIRM = 3;       // three consecutive samples at 100 Hz
// static const float ZUPT_NET       = 1.0f;
// static const float ZUPT_ALT       = 1.5f;
// static const float P_H_FLOOR      = 0.01f;
// static const float P_V_FLOOR      = 0.01f;
// static const float P_A_FLOOR      = 0.05f;
// static const int   APOGEE_CONFIRM = 10;      // 0.10 s at 100 Hz; avoids hand-test noise
// static const int   BARO_SMOOTH_W  = 3;
// static const float PRE_LAUNCH_S   = 1.0f;   // bias estimation window
// static const float MIN_APOGEE_ALT_M = 20.0f;  // apogee cannot trigger below this altitude
// static const float MIN_APOGEE_TIME_S = 1.0f;  // must spend time in coast before apogee check

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


// // The SD filename is defined in the global-state section below.
// extern char filename[32];

// // ── EEPROM flight-state log helpers ────────────────────────────────────────
// size_t eeprom_record_address(uint16_t index) {
//     return sizeof(EepromLogHeader) + (size_t)index * sizeof(EepromFlightRecord);
// }

// uint8_t eeprom_record_checksum(const EepromFlightRecord &record) {
//     const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
//     uint8_t checksum = 0xA5;
//     for (size_t i = 0; i < sizeof(EepromFlightRecord) - 1; i++) {
//         checksum ^= bytes[i];
//         checksum = (uint8_t)((checksum << 1) | (checksum >> 7));
//     }
//     return checksum;
// }

// bool eeprom_header_valid(const EepromLogHeader &header) {
//     return header.magic == EEPROM_LOG_MAGIC &&
//            header.version == EEPROM_LOG_VERSION &&
//            header.record_size == sizeof(EepromFlightRecord) &&
//            header.max_records == EEPROM_MAX_RECORDS &&
//            header.write_index < EEPROM_MAX_RECORDS &&
//            header.count <= EEPROM_MAX_RECORDS;
// }

// void eeprom_write_header() {
//     EEPROM.put(0, eeprom_header);
// }

// bool eeprom_reset_log() {
//     memset(&eeprom_header, 0, sizeof(eeprom_header));
//     eeprom_header.magic       = EEPROM_LOG_MAGIC;
//     eeprom_header.version     = EEPROM_LOG_VERSION;
//     eeprom_header.record_size = sizeof(EepromFlightRecord);
//     eeprom_header.max_records = EEPROM_MAX_RECORDS;
//     eeprom_header.write_index = 0;
//     eeprom_header.count       = 0;
//     eeprom_write_header();
//     return EEPROM.commit();
// }

// uint16_t eeprom_count_unique_flights() {
//     if (!eeprom_ready || eeprom_header.count == 0) {
//         return 0;
//     }

//     char known_files[EEPROM_MAX_RECORDS][32];
//     memset(known_files, 0, sizeof(known_files));
//     uint16_t unique_count = 0;

//     uint16_t first = (eeprom_header.count == EEPROM_MAX_RECORDS)
//                          ? eeprom_header.write_index
//                          : 0;

//     for (uint16_t i = 0; i < eeprom_header.count; i++) {
//         uint16_t index = (first + i) % EEPROM_MAX_RECORDS;
//         EepromFlightRecord record;
//         EEPROM.get(eeprom_record_address(index), record);

//         if (record.checksum != eeprom_record_checksum(record)) {
//             continue;
//         }

//         bool already_found = false;
//         for (uint16_t j = 0; j < unique_count; j++) {
//             if (strncmp(known_files[j], record.flight_file,
//                         sizeof(known_files[j])) == 0) {
//                 already_found = true;
//                 break;
//             }
//         }

//         if (!already_found && unique_count < EEPROM_MAX_RECORDS) {
//             snprintf(known_files[unique_count],
//                      sizeof(known_files[unique_count]),
//                      "%s", record.flight_file);
//             unique_count++;
//         }
//     }

//     return unique_count;
// }

// // True only when EEPROM contains a valid event proving a previous launch.
// // PAD/armed records by themselves do not trigger the two-LED warning.
// bool eeprom_has_previous_launch_data() {
//     if (!eeprom_ready || eeprom_header.count == 0) {
//         return false;
//     }

//     uint16_t first = (eeprom_header.count == EEPROM_MAX_RECORDS)
//                          ? eeprom_header.write_index
//                          : 0;

//     for (uint16_t i = 0; i < eeprom_header.count; i++) {
//         uint16_t index = (first + i) % EEPROM_MAX_RECORDS;
//         EepromFlightRecord record;
//         EEPROM.get(eeprom_record_address(index), record);

//         if (record.checksum != eeprom_record_checksum(record)) {
//             continue;
//         }

//         if (record.flight_state >= (uint8_t)PHASE_BOOST &&
//             record.flight_state <= (uint8_t)PHASE_DESCENT) {
//             return true;
//         }
//     }

//     return false;
// }

// void eeprom_print_history() {
//     Serial.println();
//     Serial.println("================ PREVIOUS FLIGHT DATA =================");

//     if (!eeprom_ready) {
//         Serial.println("EEPROM STATUS : UNAVAILABLE");
//         Serial.println("=======================================================");
//         Serial.println();
//         return;
//     }

//     if (eeprom_header.count == 0) {
//         Serial.println("PREVIOUS FLIGHT HISTORY : EMPTY");
//         Serial.println("No stored flight-state events were found.");
//         Serial.println("=======================================================");
//         Serial.println();
//         return;
//     }

//     Serial.printf("Stored events : %u / %u\n",
//                   eeprom_header.count,
//                   EEPROM_MAX_RECORDS);
//     Serial.printf("Stored flights: %u\n", eeprom_count_unique_flights());
//     Serial.println();

//     uint16_t first = (eeprom_header.count == EEPROM_MAX_RECORDS)
//                          ? eeprom_header.write_index
//                          : 0;

//     // Build a chronological list of unique flight filenames, then print each
//     // flight as its own section. This keeps states belonging to flight_0001,
//     // flight_0002, etc. visually separated.
//     char flight_files[EEPROM_MAX_RECORDS][32];
//     memset(flight_files, 0, sizeof(flight_files));
//     uint16_t flight_count = 0;

//     for (uint16_t i = 0; i < eeprom_header.count; i++) {
//         uint16_t index = (first + i) % EEPROM_MAX_RECORDS;
//         EepromFlightRecord record;
//         EEPROM.get(eeprom_record_address(index), record);

//         if (record.checksum != eeprom_record_checksum(record)) {
//             continue;
//         }

//         const char *record_file = record.flight_file[0]
//                                       ? record.flight_file
//                                       : "UNKNOWN";
//         bool already_listed = false;
//         for (uint16_t j = 0; j < flight_count; j++) {
//             if (strncmp(flight_files[j], record_file,
//                         sizeof(flight_files[j])) == 0) {
//                 already_listed = true;
//                 break;
//             }
//         }

//         if (!already_listed && flight_count < EEPROM_MAX_RECORDS) {
//             snprintf(flight_files[flight_count],
//                      sizeof(flight_files[flight_count]), "%s", record_file);
//             flight_count++;
//         }
//     }

//     for (uint16_t flight_index = 0; flight_index < flight_count; flight_index++) {
//         unsigned long stored_flight_number = 0;
//         bool numbered_file =
//             sscanf(flight_files[flight_index], "/flight_%lu.csv",
//                    &stored_flight_number) == 1;

//         if (numbered_file) {
//             Serial.printf("---------------- FLIGHT %04lu ----------------\n",
//                           stored_flight_number);
//         } else {
//             Serial.printf("--------------- FLIGHT GROUP %u ---------------\n",
//                           flight_index + 1);
//         }
//         Serial.printf("File: %s\n", flight_files[flight_index]);

//         uint16_t state_number = 0;
//         for (uint16_t i = 0; i < eeprom_header.count; i++) {
//             uint16_t index = (first + i) % EEPROM_MAX_RECORDS;
//             EepromFlightRecord record;
//             EEPROM.get(eeprom_record_address(index), record);

//             if (record.checksum != eeprom_record_checksum(record)) {
//                 continue;
//             }

//             const char *record_file = record.flight_file[0]
//                                           ? record.flight_file
//                                           : "UNKNOWN";
//             if (strncmp(record_file, flight_files[flight_index],
//                         sizeof(record.flight_file)) != 0) {
//                 continue;
//             }

//             state_number++;
//             const char *state_text =
//                 (record.flight_state <= (uint8_t)PHASE_DESCENT)
//                     ? phase_name((FlightPhase)record.flight_state)
//                     : "UNKNOWN";

//             Serial.printf(
//                 "  State %u | %7.3f s | %-7s | %8.2f m | %s\n",
//                 state_number,
//                 record.time_ms / 1000.0f,
//                 state_text,
//                 record.altitude_m,
//                 record.action[0] ? record.action : "UNSPECIFIED"
//             );
//         }

//         Serial.println();
//     }

//     Serial.println("=======================================================");
//     Serial.println();
// }

// void eeprom_init_and_print_history() {
//     Serial.printf("[EEPROM] Initialising %u-byte flash-backed EEPROM... ",
//                   (unsigned int)EEPROM_BYTES);

//     if (!EEPROM.begin(EEPROM_BYTES)) {
//         Serial.println("FAILED");
//         eeprom_ready = false;
//         return;
//     }

//     EEPROM.get(0, eeprom_header);
//     if (!eeprom_header_valid(eeprom_header)) {
//         Serial.println("no valid log found; creating a new log.");
//         if (!eeprom_reset_log()) {
//             Serial.println("[EEPROM] ERROR: could not create the EEPROM log.");
//             eeprom_ready = false;
//             return;
//         }
//     } else {
//         Serial.printf("OK (%u stored event%s).\n",
//                       eeprom_header.count,
//                       eeprom_header.count == 1 ? "" : "s");
//     }

//     eeprom_ready = true;
// }

// bool eeprom_store_flight_event(unsigned long time_ms, float altitude_m,
//                                FlightPhase state, const char *action) {
//     if (!eeprom_ready) {
//         Serial.println("[EEPROM] Event not saved because EEPROM is unavailable.");
//         return false;
//     }

//     EepromFlightRecord record;
//     memset(&record, 0, sizeof(record));
//     record.time_ms      = time_ms;
//     record.altitude_m   = altitude_m;
//     record.flight_state = (uint8_t)state;
//     snprintf(record.action, sizeof(record.action), "%s", action ? action : "STATE EVENT");
//     snprintf(record.flight_file, sizeof(record.flight_file), "%s",
//              filename[0] ? filename : "NO_FILE");
//     record.checksum = eeprom_record_checksum(record);

//     uint16_t slot = eeprom_header.write_index;
//     EEPROM.put(eeprom_record_address(slot), record);

//     eeprom_header.write_index = (slot + 1) % EEPROM_MAX_RECORDS;
//     if (eeprom_header.count < EEPROM_MAX_RECORDS) {
//         eeprom_header.count++;
//     }
//     eeprom_write_header();

//     if (EEPROM.commit()) {
//         return true;
//     } else {
//         Serial.println("[EEPROM] ERROR: commit failed; event may not be persistent.");
//         return false;
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
// int  launch_confirm_count = 0;
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

// // SD event mirror
// char  filename[32];
// bool  sd_ready         = false;
// bool  sd_last_write_ok = false;
// unsigned long sd_events_written = 0;

// // Timing
// unsigned long last_us     = 0;
// unsigned long arm_ms      = 0;

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
// static char serial_command_buffer[16] = {0};
// static uint8_t serial_command_length = 0;

// void terminal_execute_command(const char *command) {
//     if (strcmp(command, "clr") == 0) {
//         if (!eeprom_ready) {
//             Serial.println("[TERMINAL] clr failed: EEPROM is unavailable.");
//             return;
//         }

//         // Do not erase safety-critical evidence after launch detection starts.
//         if (phase != PHASE_PAD || current_launch_started ||
//             launch_confirm_count > 0) {
//             Serial.println("[TERMINAL] clr rejected: launch detection has started.");
//             return;
//         }

//         uint16_t erased_events = eeprom_header.count;
//         uint16_t erased_flights = eeprom_count_unique_flights();

//         if (!eeprom_reset_log()) {
//             Serial.println("[TERMINAL] clr failed: EEPROM commit was unsuccessful.");
//             led_set_warning(LED_ERROR_EEPROM);
//             return;
//         }
//         previous_launch_data_present = false;
//         led_pattern_start_ms = millis();

//         // Retain one marker for the active session. All records belonging to
//         // previous launches have been removed, so this PAD record will not
//         // trigger the previous-launch LED warning on the next power cycle.
//         bool marker_saved = eeprom_store_flight_event(
//             millis() - arm_ms, xA_h, PHASE_PAD, "EEPROM CLR / ARMED"
//         );

//         Serial.println();
//         Serial.println("=======================================================");
//         Serial.println("              EEPROM WIPE COMPLETE");
//         Serial.println("=======================================================");
//         Serial.printf("Previous flights erased : %u\n", erased_flights);
//         Serial.printf("Previous events erased  : %u\n", erased_events);
//         Serial.println("PREVIOUS FLIGHT HISTORY : EMPTY");
//         Serial.printf("Current armed marker    : %s (%s)\n",
//                       marker_saved ? "SAVED" : "NOT SAVED", filename);
//         Serial.println("System status           : ARMED");
//         Serial.println("Acceleration detection  : ACTIVE");
//         Serial.println("Blue LED                : STEADY ON");
//         Serial.println("=======================================================");
//         Serial.println();
//         return;
//     }

//     if (strcmp(command, "view") == 0) {
//         eeprom_print_history();
//         return;
//     }

//     Serial.printf("[TERMINAL] Unknown command: %s\n", command);
//     Serial.println("[TERMINAL] Available commands: clr, view");
// }

// void terminal_poll_commands() {
//     while (Serial.available() > 0) {
//         char incoming = (char)Serial.read();

//         if (incoming == '\r' || incoming == '\n') {
//             if (serial_command_length > 0) {
//                 serial_command_buffer[serial_command_length] = '\0';
//                 terminal_execute_command(serial_command_buffer);
//                 serial_command_length = 0;
//                 serial_command_buffer[0] = '\0';
//             }
//             continue;
//         }

//         // Accept upper- or lower-case commands without using dynamic Strings.
//         if (incoming >= 'A' && incoming <= 'Z') {
//             incoming = (char)(incoming - 'A' + 'a');
//         }

//         if (incoming == ' ' || incoming == '\t') {
//             continue;
//         }

//         if (serial_command_length < sizeof(serial_command_buffer) - 1) {
//             serial_command_buffer[serial_command_length++] = incoming;
//         } else {
//             serial_command_length = 0;
//             serial_command_buffer[0] = '\0';
//             Serial.println("[TERMINAL] Command too long. Available: clr, view");
//         }
//     }
// }

// void serial_banner() {
//     Serial.println();
//     Serial.println("============================================================");
//     Serial.println("  Kalman Apogee Detector v6 - ESP32 Serial Monitor");
//     Serial.println("============================================================");
//     Serial.printf("Baud: %lu | Sensor loop: 100 Hz\n",
//                   (unsigned long)SERIAL_BAUD);
//     Serial.println("Live telemetry: OFF (important events and errors only)");
//     Serial.printf("Axial accelerometer = %s%c axis | net_accel = axial - bias - g\n",
//                   axial_axis_sign(), axial_axis_letter());
//     Serial.printf("LEDs: RED=GPIO%d (initialising/error code), BLUE=GPIO%d (armed)\n",
//                   RED_LED_PIN, BLUE_LED_PIN);
//     Serial.println("Red codes: 1=EEPROM, 2=BMI160, 3=BMP280, 4=SD, 5=calibration.");
//     Serial.println("Terminal commands: clr=clear previous EEPROM data, view=show EEPROM data.");
//     Serial.println();
// }

// void serial_phase_change(FlightPhase old_phase, FlightPhase new_phase,
//                          unsigned long t_ms, float h, float v, float a) {
//     Serial.printf("\n>>> PHASE CHANGE: %s -> %s at %.3f s | h=%.2f m, v=%.2f m/s, a=%.2f m/s^2\n",
//                   phase_name(old_phase), phase_name(new_phase),
//                   t_ms / 1000.0f, h, v, a);
// }

// void serial_print_armed_summary() {
//     unsigned long current_flight_number = 0;
//     bool numbered_file = sscanf(filename, "/flight_%lu.csv",
//                                 &current_flight_number) == 1;

//     Serial.println();
//     Serial.println("=======================================================");
//     Serial.println("                 SYSTEM ARMED");
//     Serial.println("=======================================================");
//     if (numbered_file) {
//         Serial.printf("Current flight          : FLIGHT %04lu\n",
//                       current_flight_number);
//     } else {
//         Serial.printf("Current flight file     : %s\n", filename);
//     }
//     Serial.println("Flight state            : PAD / WAITING FOR LAUNCH");
//     Serial.println("Acceleration detection  : ACTIVE (100 Hz)");
//     if (previous_launch_data_present) {
//         Serial.println("Previous launch data    : DETECTED");
//         Serial.println("LED status              : RED + BLUE FLASHING");
//         Serial.println("Action                  : Type clr to wipe old data");
//     } else {
//         Serial.println("Previous launch data    : NONE");
//         Serial.println("LED status              : BLUE STEADY ON");
//     }
//     Serial.println("Commands                : clr | view");
//     Serial.println("=======================================================");
//     Serial.println();
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

// // ── SD event mirror ──────────────────────────────────────────────────────────
// void find_next_filename() {
//     for (int n = 1; n <= 9999; n++) {
//         snprintf(filename, sizeof(filename), "/flight_%04d.csv", n);
//         if (!SD.exists(filename)) break;
//     }
// }

// bool sd_init_event_log() {
//     Serial.print("[BOOT] SD event log init... ");
//     SPI.begin(18, 19, 23, SD_CS_PIN);

//     if (!SD.begin(SD_CS_PIN)) {
//         snprintf(filename, sizeof(filename), "NO_SD");
//         sd_ready = false;
//         sd_last_write_ok = false;
//         Serial.println("FAILED; continuing with EEPROM event logging.");
//         led_set_warning(LED_ERROR_SD);
//         return false;
//     }

//     find_next_filename();
//     File header_file = SD.open(filename, FILE_WRITE);
//     if (!header_file) {
//         sd_ready = false;
//         sd_last_write_ok = false;
//         Serial.printf("cannot create %s; continuing with EEPROM event logging.\n", filename);
//         led_set_warning(LED_ERROR_SD);
//         return false;
//     }

//     size_t bytes_written = header_file.println(
//         "time_since_arm_ms,time_since_arm_s,altitude_m,flight_state,event"
//     );
//     header_file.flush();
//     header_file.close();

//     sd_ready = (bytes_written > 0);
//     sd_last_write_ok = sd_ready;
//     if (sd_ready) {
//         Serial.printf("OK; event file=%s (opened only for individual events).\n", filename);
//     } else {
//         Serial.printf("header write failed for %s; continuing with EEPROM event logging.\n", filename);
//         led_set_warning(LED_ERROR_SD);
//     }
//     return sd_ready;
// }

// bool sd_store_flight_event(unsigned long time_ms, float altitude_m,
//                            FlightPhase state, const char *action) {
//     if (!sd_ready) {
//         Serial.println("[SD] Event not mirrored because SD is unavailable.");
//         led_set_warning(LED_ERROR_SD);
//         return false;
//     }

//     File event_file = SD.open(filename, FILE_APPEND);
//     if (!event_file) {
//         sd_last_write_ok = false;
//         Serial.printf("[SD ERROR] Cannot reopen %s for event append.\n", filename);
//         led_set_warning(LED_ERROR_SD);
//         return false;
//     }

//     char line[160];
//     snprintf(line, sizeof(line), "%lu,%.3f,%.3f,%s,%s",
//              time_ms,
//              time_ms / 1000.0f,
//              altitude_m,
//              phase_name(state),
//              action ? action : "STATE EVENT");

//     size_t bytes_written = event_file.println(line);
//     event_file.flush();
//     event_file.close();

//     sd_last_write_ok = (bytes_written > 0);
//     if (sd_last_write_ok) {
//         sd_events_written++;
//     } else {
//         Serial.printf("[SD ERROR] Event write failed at %lu ms; EEPROM copy remains primary.\n",
//                       time_ms);
//         led_set_warning(LED_ERROR_SD);
//     }
//     return sd_last_write_ok;
// }

// void store_flight_event(unsigned long time_ms, float altitude_m,
//                         FlightPhase state, const char *action) {
//     // EEPROM is deliberately committed first. SD failure must never prevent
//     // preservation of the event in flash-backed storage.
//     bool eeprom_ok = eeprom_store_flight_event(time_ms, altitude_m, state, action);
//     bool sd_ok = sd_store_flight_event(time_ms, altitude_m, state, action);

//     // Successful writes stay quiet to keep the monitor readable. Failures are
//     // still reported by the EEPROM/SD helper functions above.
//     (void)eeprom_ok;
//     (void)sd_ok;
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
//     float IKH_hh = 1.0f - K_h;
//     float IKH_vh = -K_v;

//     float tmp_hh = IKH_hh * PA_hh;
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
//     // Assert the initialising indication immediately at power-on.
//     led_begin_initialising();

//     Serial.begin(SERIAL_BAUD);
//     delay(500);
//     serial_banner();

//     // Display events retained from earlier flight tests before sensor startup.
//     eeprom_init_and_print_history();
//     if (!eeprom_ready) {
//         led_set_warning(LED_ERROR_EEPROM);
//     } else {
//         previous_launch_data_present = eeprom_has_previous_launch_data();
//         eeprom_print_history();

//         if (previous_launch_data_present) {
//             Serial.println("NOTICE: Previous launch data is retained.");
//         }
//     }

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
//         led_halt_with_error(LED_ERROR_BMI160, "BMI160 initialisation failed");
//     }
//     Serial.printf("[BOOT] BMI160 OK (chip ID 0x%02X, active address 0x%02X)\n", bmi160_read_byte(0x00), bmi160_addr);

//     // BMP280
//     Serial.print("[BOOT] BMP280 init... ");
//     uint8_t bmp_addr = 0x76;
//     if (!bmp.begin(0x76)) {          // try 0x76 first, then 0x77
//         bmp_addr = 0x77;
//         if (!bmp.begin(0x77)) {
//             Serial.println("FAILED! Check wiring / SDO address.");
//             led_halt_with_error(LED_ERROR_BMP280, "BMP280 initialisation failed");
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

//     // SD is only a secondary event mirror. Failure here must not stop arming;
//     // EEPROM remains available as the independent primary event recorder.
//     sd_init_event_log();

//     // Calibration: sample ground pressure and accel bias for PRE_LAUNCH_S
//     Serial.printf("[CAL] Calibrating for %.1f s. Keep rocket still and vertical...\n", PRE_LAUNCH_S);
//     float pres_sum = 0;
//     int   pres_cnt = 0;
//     unsigned long cal_start = millis();
//     unsigned long cal_end = cal_start + (unsigned long)(PRE_LAUNCH_S * 1000);

//     // Pre-fill baro buffer
//     for (int i = 0; i < BARO_SMOOTH_W; i++) baro_buf[i] = 44330.0f; // placeholder

//     while (millis() < cal_end) {
//         float p = bmp.readPressure();
//         pres_sum += p;
//         pres_cnt++;

//         BMI160Data imu = bmi160_read();
//         float axial_accel = get_axial_accel(imu);
//         accel_sum += axial_accel;
//         accel_count++;

//         delay(10);
//     }

//     if (pres_cnt <= 0 || accel_count <= 0 ||
//         !std::isfinite(pres_sum) || !std::isfinite(accel_sum)) {
//         led_halt_with_error(LED_ERROR_CALIBRATION, "invalid calibration samples");
//     }

//     ground_pressure_pa = pres_sum / pres_cnt;
//     ground_alt_m       = 0.0f;  // reference is ground

//     float mean_axial = accel_sum / accel_count;
//     if (!std::isfinite(ground_pressure_pa) ||
//         ground_pressure_pa < 30000.0f || ground_pressure_pa > 120000.0f ||
//         !std::isfinite(mean_axial)) {
//         led_halt_with_error(LED_ERROR_CALIBRATION, "calibration values outside valid range");
//     }

//     accel_bias_axial = mean_axial - G;   // axial should read +G at rest when rocket points up
//     bias_ready = true;

//     // Init baro smoothing buffer with ground altitude
//     for (int i = 0; i < BARO_SMOOTH_W; i++) baro_buf[i] = 0.0f;
//     baro_buf_idx = 0;

//     Serial.printf("[CAL] Done. Ground P=%.1f Pa | axial=%s%c | mean_axial=%.4f m/s^2 | accel_bias=%.4f m/s^2 | samples: pressure=%d, accel=%d\n",
//                   ground_pressure_pa, axial_axis_sign(), axial_axis_letter(),
//                   mean_axial, accel_bias_axial, pres_cnt, accel_count);
//     Serial.printf("[READY] Waiting for launch. BOOST threshold = %.1f m/s^2 net axial accel.\n", BOOST_THRESH);
//     Serial.printf("[READY] Apogee gated: phase=COAST, altitude>%.1f m, confirm=%d samples.\n",
//                   MIN_APOGEE_ALT_M, APOGEE_CONFIRM);

//     // This is the arming reference. All flight-event timestamps are elapsed
//     // milliseconds from this point because the hardware has no RTC/GPS clock.
//     arm_ms   = millis();
//     last_us  = micros();
//     store_flight_event(0, 0.0f, PHASE_PAD, "AVIONICS ARMED");

//     // setup() is complete. The next instruction entered is the live 100 Hz
//     // acceleration-monitoring loop, so BLUE now provides the armed indication.
//     led_set_armed();
//     serial_print_armed_summary();
// }

// // ── loop() ───────────────────────────────────────────────────────────────────
// void loop() {
//     // Maintain the LED state machine without blocking sensor/filter timing.
//     led_update();

//     // Serial commands are parsed without dynamic String allocation. EEPROM
//     // clearing is accepted only while the system remains safely on the pad.
//     terminal_poll_commands();

//     unsigned long now_us  = micros();
//     float dt              = (now_us - last_us) * 1e-6f;
//     last_us               = now_us;
//     if (dt <= 0 || dt > 0.5f) dt = 0.01f;  // guard

//     unsigned long t_ms = millis() - arm_ms;

//     // ── Read BMP280 ─────────────────────────────────────────────────────────
//     float pres_pa  = bmp.readPressure();
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
//             if (net_accel > BOOST_THRESH) {
//                 launch_confirm_count++;
//                 if (launch_confirm_count >= LAUNCH_CONFIRM) {
//                     phase = PHASE_BOOST;
//                 }
//             } else {
//                 launch_confirm_count = 0;
//             }
//             break; 
//         case PHASE_BOOST:
//             if (net_accel < COAST_THRESH)   phase = PHASE_COAST;
//             break;
//         case PHASE_COAST:
//             // Remain in COAST until the confirmed zero-crossing detector below
//             // records apogee and changes the state to DESCENT.
//             break;
//         case PHASE_DESCENT:
//             break;
//     }
//     if (phase != prev_phase) {
//         serial_phase_change(prev_phase, phase, t_ms, xA_h, xA_v, net_accel);

//         char action[24];
//         if (prev_phase == PHASE_PAD && phase == PHASE_BOOST) {
//             current_launch_started = true;
//             snprintf(action, sizeof(action), "LAUNCH DETECTED");
//         } else {
//             snprintf(action, sizeof(action), "%s -> %s",
//                      phase_name(prev_phase), phase_name(phase));
//         }
//         store_flight_event(t_ms, xA_h, phase, action);
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
//                 Serial.printf("\n*** APOGEE DETECTED: %.2f m at %.3f s | phase=%s | confirm=%d samples ***\n",
//                               apogee_h_m, apogee_t_ms / 1000.0f, phase_name(phase),
//                               APOGEE_CONFIRM);
//                 store_flight_event(apogee_t_ms, apogee_h_m,
//                                    PHASE_DESCENT,
//                                    "APOGEE -> DESCENT");
//             }
//         } else {
//             neg_vel_count = 0;
//         }
//     } else if (!apogee_allowed) {
//         neg_vel_count = 0;
//     }

//     // ── Loop timing: target 100 Hz ────────────────────────────────────────────
//     // BMP280 at 25Hz internally; filter runs at IMU rate (100Hz)
//     static unsigned long next_loop_us = 0;
//     if (next_loop_us == 0) next_loop_us = micros();
//     next_loop_us += 10000;  // 10 ms = 100 Hz
//     long wait_us = (long)(next_loop_us - micros());
//     if (wait_us > 0) delayMicroseconds((uint32_t)wait_us);
// }
