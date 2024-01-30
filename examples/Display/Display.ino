/*
   PAA3905 optical flow sensor display example

   Copyright (c) 2021 Tlera Corporiation and Simon D. Levy

   MIT License
 */

#include <SPI.h>

#include "PAA3905_FrameCapture.hpp"
#include "Debugger.hpp"

static const uint32_t FPS = 10;

static const uint8_t RESOLUTION = 0x2A;

PAA3905_FrameCapture _sensor(PAA3905::ORIENTATION_NORMAL, RESOLUTION);

void setup() 
{
    Serial.begin(115200);

    // Start SPI
    SPI.begin();

    // Check device ID as a test of SPI communications
    if (!_sensor.begin()) {
        Debugger::reportForever("PAA3905 initialization failed");
    }
}

void loop()
{
    static uint32_t _lastCaptureMsec;

    uint32_t msec = millis();

    if (msec - _lastCaptureMsec > 1000/FPS) {

        _lastCaptureMsec = msec;

        static uint8_t frameArray[1225];

        _sensor.captureFrame(frameArray);

        for (uint8_t j = 0; j < 35; j++) {

            for (uint8_t k = 0; k < 35; k++) {

                Serial.write(frameArray[j*35 + k]);
            }
        }

        Serial.write(0xFF); // sentinel byte
    }

}


