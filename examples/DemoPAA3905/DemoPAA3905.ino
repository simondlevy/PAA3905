/*
   Demo of the PAA3905 optical flow sensor


   This sketch configures and reads data from the PAA3905 optical flow sensor. 
   The sensor uses standard SPI for communications at a maximum serial port
   speed of 2 MHz. The sensor data ready is signaled by an active LOW
   interrupt.

   This sensor offers two sensitivities: standard detection and enhanced
   detection for rough terrain at > 15 cm height. The sensor can automatically
   switch between bright (>60 lux), low light (>30 lux), and super low light (> 5
   lux) conditions. Bright and low light modes work at 126 frames per second. The
   super low light mode is limited to 50 frames per second. 

   The sensor uses typically 3.5 mA in operation and has a 12 uA shutdown mode
   The sensor can operate in navigate mode producing delta X and Y values which
   are proportional to lateral velocity.  The limiting speed is determined by the
   maximum 7.2 rads/sec flow rate and by distance to the measurement surface; 80
   mm is the minimum measurement distance. So at 80 mm the maxium speed is 0.576
   m/s (1.25 mph), at 2 meter distance (~drone height) the maximum speed is 14.4
   m/s (32 mph), etc. 

   The sensor can also operate in raw data (frame grab) mode producing 35 x 35
   pixel images from the sensor at a frame rate of ~15 Hz. This makes the PAA3905
   an inexpensive, low-resolution, infrared-sensitive video camera.

   Copyright (c) 2021 Tlera Corporiation and Simon D. Levy

   MIT License

 */
#include <SPI.h>
#include "PAA3905.h"
#include "Debugger.hpp"

// Pin definitions
static const uint8_t CS_PIN  = 10;  // default chip select for SPI
static const uint8_t MOT_PIN =  8;  // use as data ready interrupt

// PAA3905 configuration
static uint8_t mode = standardDetectionMode; // mode choices are standardDetectionMode (default) or enhancedDetectionMode
static uint8_t autoMode = autoMode01;        // choices are autoMode01 and autoMode012 (includes superLowLight mode)
static uint8_t pixelRes = 0x2A;  // choices are from 0x00 to 0xFF
static float resolution;         // calculated (approximate) resolution (counts per delta) per meter of height
static uint8_t orientation, orient; // for X invert 0x80, for Y invert 0x40, for X and Y swap, 0x20, for all three 0XE0 (bits 5 - 7 only)
static int16_t deltaX, deltaY;
static uint32_t Shutter;
static volatile bool motionDetect;
static uint8_t statusCheck;
static uint8_t frameArray[1225], dataArray[14], SQUAL, RawDataSum, RawDataMin, RawDataMax;

static PAA3905 sensor(CS_PIN);

static void interruptHandler()
{
    motionDetect = true;
}

void setup() 
{
    Serial.begin(115200);
    delay(4000);

    pinMode(MOT_PIN, INPUT); // data ready interrupt

    // Configure SPI Flash chip select
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    SPI.begin(); // initiate SPI 
    delay(1000);

    sensor.begin();  // Prepare SPI port 

    // Check device ID as a test of SPI communications
    if (!sensor.checkID()) {
        Debugger::reportForever("Initialization of the sensor sensor failed");
    }

    sensor.reset(); // Reset PAA3905 to return all registers to default before configuring

    sensor.setMode(mode, autoMode);         // set modes

    sensor.setResolution(pixelRes);         // set resolution fraction of default 0x2A
    resolution = (sensor.getResolution() + 1) * 200.0f/8600.0f; // == 1 if pixelRes == 0x2A
    Debugger::printf("Resolution is: %f CPI per meter height", resolution * 11.914f, 1);

    sensor.setOrientation(orient);
    orientation = sensor.getOrientation();
    if (orientation & 0x80) {
        Debugger::printf("X direction inverted!\n");
    }
    if (orientation & 0x40) {
        Debugger::printf("Y direction inverted!\n");
    }
    if (orientation & 0x20) {
        Debugger::printf("X and Y swapped!\n");
    }

    attachInterrupt(MOT_PIN, interruptHandler, FALLING); // data ready interrupt active LOW 

    statusCheck = sensor.status();          // clear interrupt before entering main loop

    //  sensor.shutdown();                    // enter lowest power mode until ready to use

} // setup

void loop() {

    static uint8_t iterations;

    iterations++;

    // Navigation
    if (motionDetect){
        motionDetect = false;
        sensor.readBurstMode(dataArray); // use burst mode to read all of the data
    }

    if (dataArray[0] & 0x80) {   // Check if motion data available

        if (dataArray[0]  & 0x01) {
            Debugger::printf("Challenging surface detected!\n");
        }

        deltaX = ((int16_t)dataArray[3] << 8) | dataArray[2];
        deltaY = ((int16_t)dataArray[5] << 8) | dataArray[4];
        SQUAL = dataArray[7];      // surface quality
        RawDataSum = dataArray[8];
        RawDataMax = dataArray[9];
        RawDataMin = dataArray[10];
        Shutter = ((uint32_t)dataArray[11] << 16) | ((uint32_t)dataArray[12] << 8) | dataArray[13];
        Shutter &= 0x7FFFFF; // 23-bit positive integer 

        //   mode =    sensor.getMode();
        mode = (dataArray[1] & 0xC0) >> 6;  // mode is bits 6 and 7 
        // Don't report data if under thresholds
        if ((mode == bright       ) && (SQUAL < 25) && (Shutter >= 0x00FF80)) deltaX = deltaY = 0;
        if ((mode == lowlight     ) && (SQUAL < 70) && (Shutter >= 0x00FF80)) deltaX = deltaY = 0;
        if ((mode == superlowlight) && (SQUAL < 85) && (Shutter >= 0x025998)) deltaX = deltaY = 0;

        // Report mode
        switch (mode) {
            case bright:
                Debugger::printf("Bright Mode\n");
                break;
            case lowlight:
                Debugger::printf("Low Light Mode\n");
                break;
            case superlowlight:
                Debugger::printf("Super Low Light Mode\n");
                break;
            default:
                Debugger::printf("Unknown Mode\n");
        }

        // Data and Diagnostics output
        Debugger::printf("X: %d , Y: %d\n", deltaX, deltaY);
        Debugger::printf("Number of Valid Features: %d, Shutter: 0x%02X\n", 4*SQUAL, Shutter);
        Debugger::printf("Max Raw Data: %d, Min Raw Data: %d, Avg RawData: %d\n\n",
                RawDataMax, RawDataMin, RawDataSum);
    }

    // Frame capture
    if (iterations >= 100) { // capture one frame per 100 iterations (~5 sec) of navigation
    
        iterations = 0;
        Serial.println("Hold camera still for frame capture!");
        delay(4000);

        uint32_t frameTime = millis();
        sensor.enterFrameCaptureMode();   
        sensor.captureFrame(frameArray);
        sensor.exitFrameCaptureMode(); // exit fram capture mode
        Serial.print("Frame time = "); Serial.print(millis() - frameTime); Serial.println(" ms"); Serial.println(" ");

        for (uint8_t ii = 0; ii < 35; ii++) { // plot the frame data on the serial monitor (TFT display would be better)
        
            for (uint8_t jj = 0; jj < 35; jj++) {
                uint8_t frameval = frameArray[ii*35 + jj];
                Serial.print(frameval);
                Serial.print(" ");  
            }
            Serial.println(" ");
        }
        Serial.println(" ");

        sensor.exitFrameCaptureMode(); // exit fram capture mode
        Debugger::printf("Frame time = %d ms\n", millis() - frameTime);

        // Return to navigation mode
        sensor.reset(); // Reset PAA3905 to return all registers to default before configuring
        delay(50);
        sensor.setMode(mode, autoMode);         // set modes
        sensor.setResolution(pixelRes);         // set resolution fraction of default 0x2A
        sensor.setOrientation(orient);          // set orientation
        statusCheck = sensor.status();          // clear interrupt before entering main loop
        Debugger::printf("Back in Navigation mode!\n");
    }

    delay(50); // limit reporting to 20 Hz

    } // loop
