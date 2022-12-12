/*
   the PAA3905 optical flow sensor motion detection example

   Copyright (c) 2021 Tlera Corporiation and Simon D. Levy

   MIT License
 */

#include <SPI.h>
#include "PAA3905.h"
#include "Debugger.hpp"

// Pins
static const uint8_t CS_PIN  = 5; 
static const uint8_t MOT_PIN = 32; 

PAA3905 _sensor(CS_PIN,
        PAA3905::DETECTION_STANDARD,
        PAA3905::AUTO_MODE_01,
        PAA3905::ORIENTATION_NORMAL,
        0x2A); // resolution 0x00 to 0xFF

static volatile bool gotMotionInterrupt;

void motionInterruptHandler()
{
    gotMotionInterrupt = true;
}

void setup() 
{
    Serial.begin(115200);

    delay(4000);

    // Start SPI
    SPI.begin();

    delay(1000);

    // Check device ID as a test of SPI communications
    if (!_sensor.begin()) {
        Debugger::reportForever("PAA3905 initialization failed");
    }

    Debugger::printf("Resolution is %0.1f CPI per meter height\n", _sensor.getResolution());

    pinMode(MOT_PIN, INPUT); // data ready interrupt
    attachInterrupt(MOT_PIN, motionInterruptHandler, FALLING); // data ready interrupt active LOW 

} // setup


void loop()
{
    static uint8_t iterations;

    iterations++;

    // Navigation
    if (gotMotionInterrupt) {

        gotMotionInterrupt = false;

        _sensor.readBurstMode(); // use burst mode to read all of the data
    }

    // Frame capture
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

        for (uint8_t ii = 0; ii < 35; ii++) {
            Serial.print(ii); Serial.print(" "); 
            for (uint8_t jj = 0; jj < 35; jj++)
            {
                Serial.print(frameArray[ii*35 + jj]); Serial.print(" ");  
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

    delay(50); // limit reporting to 20 Hz

} // loop


