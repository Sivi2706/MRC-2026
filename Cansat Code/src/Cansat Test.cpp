// #include <Arduino.h>
// #include <Wire.h>
// #include <SPI.h>

// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BMP280.h>

// #include <LoRa.h>
// #include <TinyGPSPlus.h>

// // ============================================================
// //                    PIN DEFINITIONS
// // ============================================================

// // ---------------- I2C ----------------
// #define I2C_SDA 21
// #define I2C_SCL 22

// // ---------------- SPI ----------------
// #define SPI_SCK  18
// #define SPI_MISO 19
// #define SPI_MOSI 23

// // ---------------- SX1278 LoRa ----------------
// #define LORA_NSS   27
// #define LORA_DIO0  26
// #define LORA_RESET 25

// // 433 MHz SX1278
// #define LORA_FREQUENCY 433E6

// // ---------------- W25Q128 ----------------
// #define FLASH_CS 33

// // ---------------- NEO-6M GPS ----------------
// #define GPS_RX 16       // ESP32 RX <- GPS TX
// #define GPS_TX 17       // ESP32 TX -> GPS RX

// #define GPS_BAUD 9600


// // ============================================================
// //                       OBJECTS
// // ============================================================

// Adafruit_MPU6050 mpu;
// Adafruit_BMP280 bmp;

// TinyGPSPlus gps;

// // UART2 for GPS
// HardwareSerial GPS_Serial(2);


// // ============================================================
// //                    W25Q128 COMMANDS
// // ============================================================

// #define W25_READ_JEDEC_ID 0x9F

// SPISettings flashSPISettings(
//     1000000,     // 1 MHz
//     MSBFIRST,
//     SPI_MODE0
// );


// // ============================================================
// //                    HELPER FUNCTIONS
// // ============================================================

// void printSeparator()
// {
//     Serial.println();
//     Serial.println("==================================================");
// }


// // ============================================================
// //                       I2C SCANNER
// // ============================================================

// void scanI2C()
// {
//     Serial.println();
//     Serial.println("I2C BUS SCAN");
//     Serial.println("------------------------------------------");

//     int devicesFound = 0;

//     for (uint8_t address = 1; address < 127; address++)
//     {
//         Wire.beginTransmission(address);

//         uint8_t error = Wire.endTransmission();

//         if (error == 0)
//         {
//             Serial.print("I2C device found at 0x");

//             if (address < 16)
//                 Serial.print("0");

//             Serial.println(address, HEX);

//             devicesFound++;
//         }
//     }

//     if (devicesFound == 0)
//     {
//         Serial.println("FAIL: No I2C devices detected.");
//     }
//     else
//     {
//         Serial.print("PASS: ");
//         Serial.print(devicesFound);
//         Serial.println(" I2C device(s) detected.");
//     }
// }


// // ============================================================
// //                       MPU6050 TEST
// // ============================================================

// bool testMPU6050()
// {
//     Serial.println();
//     Serial.println("TESTING MPU6050");
//     Serial.println("------------------------------------------");

//     /*
//        IMPORTANT:
//        The Adafruit MPU6050 library uses the global Wire
//        interface by default.

//        Therefore:
//            mpu.begin(0x68)

//        is correct.

//        Do NOT use:
//            mpu.begin(0x68, &Wire)
//     */

//     if (!mpu.begin(0x68))
//     {
//         Serial.println("FAIL: MPU6050 not detected at 0x68.");
//         return false;
//     }

//     Serial.println("PASS: MPU6050 detected.");
//     Serial.println();

//     // Configure accelerometer
//     mpu.setAccelerometerRange(
//         MPU6050_RANGE_8_G
//     );

//     // Configure gyroscope
//     mpu.setGyroRange(
//         MPU6050_RANGE_500_DEG
//     );

//     // Configure filter
//     mpu.setFilterBandwidth(
//         MPU6050_BAND_21_HZ
//     );

//     delay(100);

//     // Read sensor
//     sensors_event_t a;
//     sensors_event_t g;
//     sensors_event_t temp;

//     mpu.getEvent(
//         &a,
//         &g,
//         &temp
//     );

//     Serial.println("MPU6050 readings:");

//     Serial.print("Acceleration X: ");
//     Serial.print(a.acceleration.x);
//     Serial.println(" m/s^2");

//     Serial.print("Acceleration Y: ");
//     Serial.print(a.acceleration.y);
//     Serial.println(" m/s^2");

//     Serial.print("Acceleration Z: ");
//     Serial.print(a.acceleration.z);
//     Serial.println(" m/s^2");

//     Serial.println();

//     Serial.print("Gyroscope X: ");
//     Serial.print(g.gyro.x);
//     Serial.println(" rad/s");

//     Serial.print("Gyroscope Y: ");
//     Serial.print(g.gyro.y);
//     Serial.println(" rad/s");

//     Serial.print("Gyroscope Z: ");
//     Serial.print(g.gyro.z);
//     Serial.println(" rad/s");

//     Serial.println();

//     Serial.print("Temperature: ");
//     Serial.print(temp.temperature);
//     Serial.println(" °C");

//     return true;
// }


// // ============================================================
// //                       BMP280 TEST
// // ============================================================

// bool testBMP280()
// {
//     Serial.println();
//     Serial.println("TESTING BMP280");
//     Serial.println("------------------------------------------");

//     bool detected = false;

//     /*
//        IMPORTANT:
//        bmp.begin() takes the I2C address.

//        Correct:
//            bmp.begin(0x76)

//        Incorrect:
//            bmp.begin(0x76, &Wire)
//     */

//     // Try address 0x76
//     if (bmp.begin(0x76))
//     {
//         Serial.println("PASS: BMP280 detected at 0x76.");
//         detected = true;
//     }

//     // Try address 0x77
//     if (!detected)
//     {
//         if (bmp.begin(0x77))
//         {
//             Serial.println("PASS: BMP280 detected at 0x77.");
//             detected = true;
//         }
//     }

//     if (!detected)
//     {
//         Serial.println("FAIL: BMP280 not detected.");
//         Serial.println("Check SDA, SCL, VCC and GND.");
//         return false;
//     }

//     // Configure BMP280

//     bmp.setSampling(
//         Adafruit_BMP280::MODE_NORMAL,

//         // Temperature oversampling
//         Adafruit_BMP280::SAMPLING_X2,

//         // Pressure oversampling
//         Adafruit_BMP280::SAMPLING_X16,

//         // Filter
//         Adafruit_BMP280::FILTER_X16,

//         // Standby
//         Adafruit_BMP280::STANDBY_MS_125
//     );

//     delay(100);

//     Serial.println();
//     Serial.println("BMP280 readings:");

//     Serial.print("Temperature: ");
//     Serial.print(
//         bmp.readTemperature()
//     );
//     Serial.println(" °C");

//     Serial.print("Pressure: ");
//     Serial.print(
//         bmp.readPressure() / 100.0
//     );
//     Serial.println(" hPa");

//     Serial.print("Approx altitude: ");
//     Serial.print(
//         bmp.readAltitude(1013.25)
//     );
//     Serial.println(" m");

//     return true;
// }


// // ============================================================
// //                  W25Q128 JEDEC ID
// // ============================================================

// uint32_t readFlashJEDEC()
// {
//     uint32_t id = 0;

//     uint8_t manufacturer;
//     uint8_t memoryType;
//     uint8_t capacity;

//     SPI.beginTransaction(
//         flashSPISettings
//     );

//     // Select W25Q128
//     digitalWrite(
//         FLASH_CS,
//         LOW
//     );

//     // JEDEC ID command
//     SPI.transfer(
//         W25_READ_JEDEC_ID
//     );

//     // Read 3 ID bytes
//     manufacturer = SPI.transfer(0x00);
//     memoryType  = SPI.transfer(0x00);
//     capacity    = SPI.transfer(0x00);

//     // Deselect W25Q128
//     digitalWrite(
//         FLASH_CS,
//         HIGH
//     );

//     SPI.endTransaction();

//     id =
//         ((uint32_t)manufacturer << 16) |
//         ((uint32_t)memoryType << 8) |
//         capacity;

//     return id;
// }


// // ============================================================
// //                       W25Q128 TEST
// // ============================================================

// bool testW25Q128()
// {
//     Serial.println();
//     Serial.println("TESTING W25Q128");
//     Serial.println("------------------------------------------");

//     uint32_t id = readFlashJEDEC();

//     uint8_t manufacturer =
//         (id >> 16) & 0xFF;

//     uint8_t memoryType =
//         (id >> 8) & 0xFF;

//     uint8_t capacity =
//         id & 0xFF;

//     Serial.print("Manufacturer ID: 0x");

//     if (manufacturer < 0x10)
//         Serial.print("0");

//     Serial.println(
//         manufacturer,
//         HEX
//     );

//     Serial.print("Memory Type:     0x");

//     if (memoryType < 0x10)
//         Serial.print("0");

//     Serial.println(
//         memoryType,
//         HEX
//     );

//     Serial.print("Capacity ID:     0x");

//     if (capacity < 0x10)
//         Serial.print("0");

//     Serial.println(
//         capacity,
//         HEX
//     );

//     Serial.print("JEDEC ID:        0x");

//     if (id < 0x100000)
//         Serial.print("0");

//     Serial.println(
//         id,
//         HEX
//     );

//     /*
//        Typical Winbond W25Q128:

//        Manufacturer = EF
//        Memory Type  = 40
//        Capacity     = 18

//        Therefore:

//        EF 40 18
//     */

//     if (
//         manufacturer == 0xEF &&
//         memoryType == 0x40 &&
//         capacity == 0x18
//     )
//     {
//         Serial.println();
//         Serial.println(
//             "PASS: W25Q128 detected."
//         );

//         Serial.println(
//             "Capacity: 128 Mbit / 16 MB"
//         );

//         return true;
//     }

//     /*
//        If the flash responded but has another
//        JEDEC ID, report it separately.
//     */

//     if (
//         id != 0xFFFFFF &&
//         id != 0x000000
//     )
//     {
//         Serial.println();
//         Serial.println(
//             "WARNING: SPI flash responded."
//         );

//         Serial.println(
//             "JEDEC ID does not match standard W25Q128."
//         );

//         return true;
//     }

//     Serial.println();
//     Serial.println(
//         "FAIL: W25Q128 did not respond."
//     );

//     return false;
// }


// // ============================================================
// //                       SX1278 TEST
// // ============================================================

// bool testLoRa()
// {
//     Serial.println();
//     Serial.println("TESTING SX1278 LoRa");
//     Serial.println("------------------------------------------");

//     // Tell LoRa library which pins are being used
//     LoRa.setPins(
//         LORA_NSS,
//         LORA_RESET,
//         LORA_DIO0
//     );

//     Serial.println(
//         "Initializing SX1278..."
//     );

//     Serial.println(
//         "Frequency: 433 MHz"
//     );

//     if (!LoRa.begin(LORA_FREQUENCY))
//     {
//         Serial.println();
//         Serial.println(
//             "FAIL: SX1278 not detected."
//         );

//         Serial.println();
//         Serial.println(
//             "Check:"
//         );

//         Serial.println(
//             "  SCK  -> GPIO18"
//         );

//         Serial.println(
//             "  MISO -> GPIO19"
//         );

//         Serial.println(
//             "  MOSI -> GPIO23"
//         );

//         Serial.println(
//             "  NSS  -> GPIO27"
//         );

//         Serial.println(
//             "  DIO0 -> GPIO26"
//         );

//         Serial.println(
//             "  RESET -> GPIO25"
//         );

//         return false;
//     }

//     Serial.println(
//         "PASS: SX1278 initialized."
//     );

//     // --------------------------------------------------------
//     // LoRa configuration
//     // --------------------------------------------------------

//     LoRa.setSpreadingFactor(7);

//     LoRa.setSignalBandwidth(
//         125E3
//     );

//     LoRa.setCodingRate4(5);

//     LoRa.enableCrc();

//     Serial.println();
//     Serial.println(
//         "LoRa configuration:"
//     );

//     Serial.println(
//         "  Frequency : 433 MHz"
//     );

//     Serial.println(
//         "  SF        : 7"
//     );

//     Serial.println(
//         "  Bandwidth : 125 kHz"
//     );

//     Serial.println(
//         "  Coding    : 4/5"
//     );

//     Serial.println(
//         "  CRC       : ENABLED"
//     );

//     return true;
// }


// // ============================================================
// //                       GPS TEST
// // ============================================================

// bool testGPS()
// {
//     Serial.println();
//     Serial.println("TESTING NEO-6M GPS");
//     Serial.println("------------------------------------------");

//     Serial.println(
//         "Waiting for GPS NMEA data..."
//     );

//     unsigned long startTime =
//         millis();

//     bool dataReceived = false;

//     /*
//        Wait up to 5 seconds for
//        NMEA data.
//     */

//     while (
//         millis() - startTime < 5000
//     )
//     {
//         while (
//             GPS_Serial.available()
//         )
//         {
//             char c =
//                 GPS_Serial.read();

//             if (gps.encode(c))
//             {
//                 dataReceived = true;
//             }
//         }
//     }

//     if (!dataReceived)
//     {
//         Serial.println();
//         Serial.println(
//             "WARNING: No valid GPS NMEA data received."
//         );

//         Serial.println();
//         Serial.println(
//             "Check:"
//         );

//         Serial.println(
//             "  GPS TX -> ESP32 GPIO16"
//         );

//         Serial.println(
//             "  GPS RX -> ESP32 GPIO17"
//         );

//         Serial.println(
//             "  GPS GND -> ESP32 GND"
//         );

//         Serial.println(
//             "  GPS power"
//         );

//         Serial.println(
//             "  Baud rate = 9600"
//         );

//         return false;
//     }

//     Serial.println(
//         "PASS: GPS NMEA data received."
//     );

//     // --------------------------------------------------------
//     // GPS position
//     // --------------------------------------------------------

//     if (gps.location.isValid())
//     {
//         Serial.println();
//         Serial.println(
//             "GPS position:"
//         );

//         Serial.print(
//             "Latitude:  "
//         );

//         Serial.println(
//             gps.location.lat(),
//             6
//         );

//         Serial.print(
//             "Longitude: "
//         );

//         Serial.println(
//             gps.location.lng(),
//             6
//         );
//     }
//     else
//     {
//         Serial.println();
//         Serial.println(
//             "GPS position: NO FIX"
//         );

//         Serial.println(
//             "NMEA communication works,"
//         );

//         Serial.println(
//             "but the GPS has not obtained"
//         );

//         Serial.println(
//             "a valid satellite fix yet."
//         );
//     }

//     // --------------------------------------------------------
//     // Satellites
//     // --------------------------------------------------------

//     if (gps.satellites.isValid())
//     {
//         Serial.print(
//             "Satellites: "
//         );

//         Serial.println(
//             gps.satellites.value()
//         );
//     }

//     // --------------------------------------------------------
//     // GPS altitude
//     // --------------------------------------------------------

//     if (gps.altitude.isValid())
//     {
//         Serial.print(
//             "GPS altitude: "
//         );

//         Serial.print(
//             gps.altitude.meters()
//         );

//         Serial.println(
//             " m"
//         );
//     }

//     return true;
// }


// // ============================================================
// //                           SETUP
// // ============================================================

// void setup()
// {
//     Serial.begin(
//         115200
//     );

//     // Give Serial Monitor time to start
//     delay(2000);

//     printSeparator();

//     Serial.println(
//         "ESP32 ROCKET AVIONICS TEST"
//     );

//     Serial.println(
//         "Component diagnostic program"
//     );

//     printSeparator();

//     // --------------------------------------------------------
//     // PIN CONFIGURATION DISPLAY
//     // --------------------------------------------------------

//     Serial.println(
//         "PIN CONFIGURATION"
//     );

//     Serial.println(
//         "--------------------------------------------------"
//     );

//     Serial.println(
//         "I2C:"
//     );

//     Serial.println(
//         "  SDA  = GPIO21"
//     );

//     Serial.println(
//         "  SCL  = GPIO22"
//     );

//     Serial.println();

//     Serial.println(
//         "SPI:"
//     );

//     Serial.println(
//         "  SCK  = GPIO18"
//     );

//     Serial.println(
//         "  MISO = GPIO19"
//     );

//     Serial.println(
//         "  MOSI = GPIO23"
//     );

//     Serial.println();

//     Serial.println(
//         "SX1278:"
//     );

//     Serial.println(
//         "  NSS   = GPIO27"
//     );

//     Serial.println(
//         "  DIO0  = GPIO26"
//     );

//     Serial.println(
//         "  RESET = GPIO25"
//     );

//     Serial.println();

//     Serial.println(
//         "W25Q128:"
//     );

//     Serial.println(
//         "  CS = GPIO33"
//     );

//     Serial.println();

//     Serial.println(
//         "NEO-6M:"
//     );

//     Serial.println(
//         "  RX = GPIO16"
//     );

//     Serial.println(
//         "  TX = GPIO17"
//     );

//     printSeparator();


//     // ========================================================
//     // INITIALIZE I2C
//     // ========================================================

//     Serial.println(
//         "INITIALIZING I2C..."
//     );

//     Wire.begin(
//         I2C_SDA,
//         I2C_SCL
//     );

//     delay(100);

//     Serial.println(
//         "I2C initialized."
//     );

//     scanI2C();

//     printSeparator();


//     // ========================================================
//     // INITIALIZE SPI
//     // ========================================================

//     Serial.println(
//         "INITIALIZING SPI..."
//     );

//     // W25Q128 chip select
//     pinMode(
//         FLASH_CS,
//         OUTPUT
//     );

//     // Deselect flash
//     digitalWrite(
//         FLASH_CS,
//         HIGH
//     );

//     // Start SPI
//     SPI.begin(
//         SPI_SCK,
//         SPI_MISO,
//         SPI_MOSI
//     );

//     Serial.println(
//         "SPI initialized."
//     );

//     printSeparator();


//     // ========================================================
//     // INITIALIZE GPS UART
//     // ========================================================

//     Serial.println(
//         "INITIALIZING GPS UART..."
//     );

//     GPS_Serial.begin(
//         GPS_BAUD,
//         SERIAL_8N1,
//         GPS_RX,
//         GPS_TX
//     );

//     Serial.println(
//         "GPS UART initialized."
//     );

//     printSeparator();


//     // ========================================================
//     // TEST MPU6050
//     // ========================================================

//     bool mpuOK =
//         testMPU6050();

//     printSeparator();


//     // ========================================================
//     // TEST BMP280
//     // ========================================================

//     bool bmpOK =
//         testBMP280();

//     printSeparator();


//     // ========================================================
//     // TEST W25Q128
//     // ========================================================

//     bool flashOK =
//         testW25Q128();

//     printSeparator();


//     // ========================================================
//     // TEST SX1278
//     // ========================================================

//     bool loraOK =
//         testLoRa();

//     printSeparator();


//     // ========================================================
//     // TEST GPS
//     // ========================================================

//     bool gpsOK =
//         testGPS();

//     printSeparator();


//     // ========================================================
//     // FINAL RESULTS
//     // ========================================================

//     Serial.println(
//         "FINAL TEST RESULTS"
//     );

//     Serial.println(
//         "=================================================="
//     );

//     Serial.print(
//         "MPU6050 : "
//     );

//     Serial.println(
//         mpuOK ? "PASS" : "FAIL"
//     );

//     Serial.print(
//         "BMP280  : "
//     );

//     Serial.println(
//         bmpOK ? "PASS" : "FAIL"
//     );

//     Serial.print(
//         "W25Q128 : "
//     );

//     Serial.println(
//         flashOK ? "PASS" : "FAIL"
//     );

//     Serial.print(
//         "SX1278  : "
//     );

//     Serial.println(
//         loraOK ? "PASS" : "FAIL"
//     );

//     Serial.print(
//         "NEO-6M  : "
//     );

//     Serial.println(
//         gpsOK ? "PASS" : "NO DATA / NO FIX"
//     );

//     Serial.println(
//         "=================================================="
//     );


//     if (
//         mpuOK &&
//         bmpOK &&
//         flashOK &&
//         loraOK
//     )
//     {
//         Serial.println(
//             "CORE SYSTEM: PASS"
//         );
//     }
//     else
//     {
//         Serial.println(
//             "CORE SYSTEM: CHECK FAILED DEVICE(S)"
//         );
//     }

//     Serial.println();

//     Serial.println(
//         "GPS note:"
//     );

//     Serial.println(
//         "GPS NMEA communication and GPS satellite"
//     );

//     Serial.println(
//         "fix are two different things."
//     );

//     Serial.println(
//         "A GPS can communicate correctly while"
//     );

//     Serial.println(
//         "still showing NO FIX."
//     );

//     printSeparator();
// }


// // ============================================================
// //                            LOOP
// // ============================================================

// void loop()
// {
//     // --------------------------------------------------------
//     // Continuously process GPS data
//     // --------------------------------------------------------

//     while (
//         GPS_Serial.available()
//     )
//     {
//         char c =
//             GPS_Serial.read();

//         gps.encode(c);
//     }


//     // --------------------------------------------------------
//     // Print live readings every 2 seconds
//     // --------------------------------------------------------

//     static unsigned long lastPrint = 0;

//     if (
//         millis() - lastPrint >= 2000
//     )
//     {
//         lastPrint =
//             millis();

//         Serial.println();
//         Serial.println(
//             "LIVE SENSOR DATA"
//         );

//         Serial.println(
//             "------------------------------------------"
//         );


//         // ====================================================
//         // MPU6050
//         // ====================================================

//         sensors_event_t a;
//         sensors_event_t g;
//         sensors_event_t temp;

//         mpu.getEvent(
//             &a,
//             &g,
//             &temp
//         );

//         Serial.println(
//             "MPU6050:"
//         );

//         Serial.print(
//             "  Accel X: "
//         );

//         Serial.print(
//             a.acceleration.x
//         );

//         Serial.println(
//             " m/s^2"
//         );

//         Serial.print(
//             "  Accel Y: "
//         );

//         Serial.print(
//             a.acceleration.y
//         );

//         Serial.println(
//             " m/s^2"
//         );

//         Serial.print(
//             "  Accel Z: "
//         );

//         Serial.print(
//             a.acceleration.z
//         );

//         Serial.println(
//             " m/s^2"
//         );

//         Serial.print(
//             "  Gyro X: "
//         );

//         Serial.print(
//             g.gyro.x
//         );

//         Serial.println(
//             " rad/s"
//         );

//         Serial.print(
//             "  Gyro Y: "
//         );

//         Serial.print(
//             g.gyro.y
//         );

//         Serial.println(
//             " rad/s"
//         );

//         Serial.print(
//             "  Gyro Z: "
//         );

//         Serial.print(
//             g.gyro.z
//         );

//         Serial.println(
//             " rad/s"
//         );


//         // ====================================================
//         // BMP280
//         // ====================================================

//         Serial.println(
//             "BMP280:"
//         );

//         Serial.print(
//             "  Temperature: "
//         );

//         Serial.print(
//             bmp.readTemperature()
//         );

//         Serial.println(
//             " °C"
//         );

//         Serial.print(
//             "  Pressure: "
//         );

//         Serial.print(
//             bmp.readPressure() / 100.0
//         );

//         Serial.println(
//             " hPa"
//         );

//         Serial.print(
//             "  Altitude: "
//         );

//         Serial.print(
//             bmp.readAltitude(1013.25)
//         );

//         Serial.println(
//             " m"
//         );


//         // ====================================================
//         // GPS
//         // ====================================================

//         Serial.println(
//             "GPS:"
//         );

//         if (
//             gps.location.isValid()
//         )
//         {
//             Serial.print(
//                 "  Latitude: "
//             );

//             Serial.println(
//                 gps.location.lat(),
//                 6
//             );

//             Serial.print(
//                 "  Longitude: "
//             );

//             Serial.println(
//                 gps.location.lng(),
//                 6
//             );
//         }
//         else
//         {
//             Serial.println(
//                 "  Position: NO FIX"
//             );
//         }

//         if (
//             gps.satellites.isValid()
//         )
//         {
//             Serial.print(
//                 "  Satellites: "
//             );

//             Serial.println(
//                 gps.satellites.value()
//             );
//         }

//         if (
//             gps.altitude.isValid()
//         )
//         {
//             Serial.print(
//                 "  GPS Altitude: "
//             );

//             Serial.print(
//                 gps.altitude.meters()
//             );

//             Serial.println(
//                 " m"
//             );
//         }

//         Serial.println(
//             "------------------------------------------"
//         );
//     }
// }