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
    // Navigation
    if (gotMotionInterrupt) {

        gotMotionInterrupt = false;

        _sensor.readBurstMode(); // use burst mode to read all of the data
    }

    if (_sensor.motionDataAvailable()) { 

        if (_sensor.challengingSurfaceDetected()) {
            Debugger::printf("Challenging surface detected!\n");
        }

        int16_t deltaX = _sensor.getDeltaX();
        int16_t deltaY = _sensor.getDeltaY();

        uint8_t surfaceQuality = _sensor.getSurfaceQuality();
        uint8_t rawDataSum = _sensor.getRawDataSum();
        uint8_t rawDataMax = _sensor.getRawDataMax();
        uint8_t rawDataMin = _sensor.getRawDataMin();

        uint32_t shutter = _sensor.getShutter();

        PAA3905::lightMode_t lightMode = _sensor.getLightMode();

        static const char * light_mode_names[4] = {"Bright", "Low", "Super-low", "Unknown"};
        Debugger::printf("\n%s light mode\n", light_mode_names[lightMode]);

        // Don't report X,Y if surface quality and shutter are under thresholds
        if (_sensor.dataAboveThresholds(lightMode, surfaceQuality, shutter)) {
            Debugger::printf("X: %d  Y: %d\n", deltaX, deltaY);
        }
        else {
            Debugger::printf("Data is below thresholds for X,Y reporting\n");
        }

        Debugger::printf("Number of Valid Features: %d, shutter: 0x%X\n",
                4*surfaceQuality, shutter);
        Debugger::printf("Max raw data: %d  Min raw data: %d  Avg raw data: %d\n",
                rawDataMax, rawDataMin, rawDataSum);
    }

    delay(50); // limit reporting to 20 Hz

} // loop


