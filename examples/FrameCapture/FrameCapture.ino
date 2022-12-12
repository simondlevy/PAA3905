/*
   the PAA3905 optical flow sensor motion detection example

   Copyright (c) 2021 Tlera Corporiation and Simon D. Levy

   MIT License
 */

#include <SPI.h>
#include "PAA3905.h"
#include "Debugger.hpp"

// Chip-select pin
static const uint8_t CS_PIN  = 5; 

// Frame rate
static const uint8_t FRAME_RATE = 0;

PAA3905 _sensor(CS_PIN,
        PAA3905::DETECTION_STANDARD,
        PAA3905::AUTO_MODE_01,
        PAA3905::ORIENTATION_NORMAL,
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
    static uint8_t iterations;

    iterations++;

    if (iterations >= 100) // capture one frame per 100 iterations (~5 sec) of navigation
    {
        iterations = 0;
        Debugger::printf("\nHold camera still for frame capture!\n");
        delay(4000);

        uint32_t frameTime = millis();
        static uint8_t frameArray[1225];

        _sensor.enterFrameCaptureMode();   
        _sensor.captureFrame(frameArray);
     
        _sensor.exitFrameCaptureMode(); // exit fram capture mode

        Debugger::printf("Frame time = %d ms\n", millis() - frameTime);

        for (uint8_t j = 0; j < 35; j++) {
            Serial.print(j); Serial.print(" "); 
            for (uint8_t k = 0; k < 35; k++)
            {
                Serial.print(frameArray[j*35 + k]); Serial.print(" ");  
            }
            Serial.println(" ");
        }
        Serial.println(" ");

        _sensor.exitFrameCaptureMode(); // exit fram capture mode
        Debugger::printf("Frame time = %d ms\n", millis() - frameTime);

        static const PAA3905::detectionMode_t DETECTION_MODE = PAA3905::DETECTION_STANDARD;
        static const PAA3905::autoMode_t AUTO_MODE           = PAA3905::AUTO_MODE_01;
        static const PAA3905::orientation_t ORIENTATION       = PAA3905::ORIENTATION_NORMAL;
        static const uint8_t RESOLUTION                       = 0x2A; // 0x00 to 0xFF

        // Return to navigation mode
        _sensor.reset(); // Reset PAA3905 to return all registers to default before configuring

        delay(50);

        _sensor.setMode(DETECTION_MODE, AUTO_MODE); // set modes
        _sensor.setResolution(RESOLUTION);         // set resolution fraction of default 0x2A
        _sensor.setOrientation(ORIENTATION);
        _sensor.clearInterrupt();
        Debugger::printf("Back in Navigation mode!\n");
    }

} // loop


