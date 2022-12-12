/*
   the PAA3905 optical flow sensor motion detection example

   Copyright (c) 2021 Tlera Corporiation and Simon D. Levy

   MIT License
 */

#include <SPI.h>
#include "PAA3905_FrameCapture.h"
#include "Debugger.hpp"

static const uint8_t CS_PIN  = 5; 

static const uint32_t FRAME_PERIOD_MSEC = 3000;

PAA3905_FrameCapture _sensor(CS_PIN,
        PAA3905_FrameCapture::ORIENTATION_NORMAL,
        0x2A); // resolution 0x00 to 0xFF

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

    if (msec - _lastCaptureMsec > FRAME_PERIOD_MSEC) {

        _lastCaptureMsec = msec;

        static uint8_t frameArray[1225];

        _sensor.captureFrame(frameArray);

        Debugger::printf("Frame time = %d ms\n", millis() - _lastCaptureMsec);

        for (uint8_t j = 0; j < 35; j++) {

            Debugger::printf("%2d ", j);

            for (uint8_t k = 0; k < 35; k++) {

                // Debugger::printf() would be too slow here
                Serial.print(frameArray[j*35 + k]);
                Serial.print(" ");  
            }
            Debugger::printf("\n");
        }
        Debugger::printf("\n");

        Debugger::printf("Frame time = %d ms\n", millis() - _lastCaptureMsec);
    }

} // loop


