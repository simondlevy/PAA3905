/*
   the PAA3905 optical flow sensor motion detection example

   Copyright (c) 2021 Tlera Corporiation and Simon D. Levy

   MIT License
 */

#include <SPI.h>

#include "PAA3905_MotionCapture.h"
#include "Debugger.hpp"

static const uint8_t CS_PIN  = 5; 
static const uint8_t MOT_PIN = 32; 

PAA3905_MotionCapture _sensor(CS_PIN,
        PAA3905_MotionCapture::DETECTION_STANDARD,
        PAA3905_MotionCapture::AUTO_MODE_01,
        PAA3905_MotionCapture::ORIENTATION_NORMAL,
        0x2A); // resolution 0x00 to 0xFF

static volatile bool gotMotionInterrupt;

void motionInterruptHandler()
{
    gotMotionInterrupt = true;
}

void setup() 
{
    Serial.begin(115200);

    // Start SPI
    SPI.begin();

    delay(100);

    // Check device ID as a test of SPI communications
    if (!_sensor.begin()) {
        Debugger::reportForever("PAA3905 initialization failed");
    }

    Debugger::printf("Resolution is %0.1f CPI per meter height\n", _sensor.getResolution());

    pinMode(MOT_PIN, INPUT); 
    attachInterrupt(MOT_PIN, motionInterruptHandler, FALLING);
} 

void loop()
{
    if (gotMotionInterrupt) {

        gotMotionInterrupt = false;

        _sensor.readBurstMode(); // use burst mode to read all of the data

        if (_sensor.motionDataAvailable()) { 

            static uint32_t _count;

            Debugger::printf("\n%05d ---------------------------------\n", _count++);

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

            PAA3905_MotionCapture::lightMode_t lightMode = _sensor.getLightMode();

            static const char * light_mode_names[4] = {"Bright", "Low", "Super-low", "Unknown"};
            Debugger::printf("%s light mode\n", light_mode_names[lightMode]);

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
    }

} // loop


