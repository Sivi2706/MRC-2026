#include <Arduino.h>

// ============================================================
//                 LoRa GPIO DEFINITIONS
// ============================================================

#define LORA_SCK     18
#define LORA_MISO   19
#define LORA_MOSI   23
#define LORA_NSS    27
#define LORA_DIO0   26
#define LORA_RESET  25


// ============================================================
//                 GPIO TEST STRUCTURE
// ============================================================

struct GpioTest
{
    const char* name;
    uint8_t pin;
};


// Order of testing
GpioTest pins[] =
{
    {"SCK",   LORA_SCK},
    {"MISO",  LORA_MISO},
    {"MOSI",  LORA_MOSI},
    {"NSS",   LORA_NSS},
    {"DIO0",  LORA_DIO0},
    {"RESET", LORA_RESET}
};

const int NUM_PINS =
    sizeof(pins) / sizeof(pins[0]);


// ============================================================
//                 SET ALL PINS LOW
// ============================================================

void allPinsLow()
{
    for (int i = 0; i < NUM_PINS; i++)
    {
        digitalWrite(
            pins[i].pin,
            LOW
        );
    }
}


// ============================================================
//                 WAIT FOR ENTER
// ============================================================

void waitForEnter()
{
    while (Serial.available() > 0)
    {
        Serial.read();
    }

    while (true)
    {
        if (Serial.available() > 0)
        {
            char c =
                Serial.read();

            if (c == '\n' || c == '\r')
            {
                delay(100);

                while (Serial.available() > 0)
                {
                    Serial.read();
                }

                return;
            }
        }

        delay(10);
    }
}


// ============================================================
//                         SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println();
    Serial.println(
        "=================================================="
    );

    Serial.println(
        "       SX1278 GPIO CONNECTION / SHORT TEST"
    );

    Serial.println(
        "=================================================="
    );

    Serial.println();

    Serial.println(
        "IMPORTANT:"
    );

    Serial.println(
        "SX1278 MUST BE DISCONNECTED."
    );

    Serial.println(
        "Only the ESP32 should be powered."
    );

    Serial.println();

    // --------------------------------------------------------
    // Configure pins
    // --------------------------------------------------------

    for (int i = 0; i < NUM_PINS; i++)
    {
        pinMode(
            pins[i].pin,
            OUTPUT
        );

        digitalWrite(
            pins[i].pin,
            LOW
        );
    }

    Serial.println(
        "All LoRa GPIO pins initialized LOW."
    );

    Serial.println(
        "Expected voltage initially: 0 V"
    );

    Serial.println();

    Serial.println(
        "Press ENTER to begin the test."
    );

    waitForEnter();


    // ========================================================
    //                 INDIVIDUAL PIN TEST
    // ========================================================

    for (int i = 0; i < NUM_PINS; i++)
    {
        // ----------------------------------------------------
        // Make absolutely sure every pin is LOW
        // ----------------------------------------------------

        allPinsLow();

        delay(100);


        // ----------------------------------------------------
        // Turn ON current GPIO
        // ----------------------------------------------------

        digitalWrite(
            pins[i].pin,
            HIGH
        );

        delay(200);


        Serial.println();
        Serial.println(
            "--------------------------------------------------"
        );

        Serial.print(
            "TEST "
        );

        Serial.print(
            i + 1
        );

        Serial.print(
            " / "
        );

        Serial.println(
            NUM_PINS
        );

        Serial.println(
            "--------------------------------------------------"
        );

        Serial.print(
            "GPIO: "
        );

        Serial.println(
            pins[i].pin
        );

        Serial.print(
            "LoRa signal: "
        );

        Serial.println(
            pins[i].name
        );

        Serial.println();

        Serial.println(
            "CURRENT GPIO = HIGH (~3.3 V)"
        );

        Serial.println();

        Serial.println(
            "Measure against ESP32 GND:"
        );

        Serial.println();

        Serial.print(
            "GPIO"
        );

        Serial.print(
            pins[i].pin
        );

        Serial.println(
            " should = ~3.3 V"
        );

        Serial.println();

        Serial.println(
            "ALL OTHER LoRa GPIOs should = ~0 V"
        );

        Serial.println();

        Serial.println(
            "Check for shorts between the pins."
        );

        Serial.println();

        Serial.println(
            "Press ENTER to turn this GPIO OFF"
        );

        Serial.println(
            "and continue to the next GPIO."
        );

        // ----------------------------------------------------
        // Wait for user
        // ----------------------------------------------------

        waitForEnter();


        // ----------------------------------------------------
        // Turn current GPIO OFF
        // ----------------------------------------------------

        digitalWrite(
            pins[i].pin,
            LOW
        );

        Serial.println();

        Serial.print(
            "GPIO"
        );

        Serial.print(
            pins[i].pin
        );

        Serial.println(
            " = LOW (0 V)"
        );

        Serial.println();

        delay(200);
    }


    // ========================================================
    //                 TEST COMPLETE
    // ========================================================

    allPinsLow();

    Serial.println();
    Serial.println(
        "=================================================="
    );

    Serial.println(
        "              TEST COMPLETE"
    );

    Serial.println(
        "=================================================="
    );

    Serial.println();

    Serial.println(
        "All LoRa GPIO pins are now LOW (0 V)."
    );

    Serial.println();

    Serial.println(
        "Expected final state:"
    );

    for (int i = 0; i < NUM_PINS; i++)
    {
        Serial.print(
            "GPIO"
        );

        Serial.print(
            pins[i].pin
        );

        Serial.print(
            " ("
        );

        Serial.print(
            pins[i].name
        );

        Serial.println(
            ") = 0 V"
        );
    }

    Serial.println();

    Serial.println(
        "If every pin behaved correctly,"
    );

    Serial.println(
        "the ESP32 GPIO outputs are working."
    );

    Serial.println(
        "=================================================="
    );
}


// ============================================================
//                          LOOP
// ============================================================

void loop()
{
    // Nothing required.
}