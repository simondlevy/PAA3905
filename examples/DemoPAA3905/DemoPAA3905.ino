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

   The sensor uses typically 3.5 mA in operation and has a 12 uA shutdown mode.

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

// Pins
static const uint8_t CS_PIN  = 10; 
static const uint8_t MOT_PIN =  8; 

// Sensor configuration
static const PAA3905::detection_mode_t DETECTION_MODE = PAA3905::DETECTION_STANDARD;
static const PAA3905::auto_mode_t AUTO_MODE           = PAA3905::AUTO_MODE_01;
static const PAA3905::orientation_t ORIENTATION       = PAA3905::ORIENTATION_NORMAL;
static const uint8_t RESOLUTION                       = 0x2A; // 0x00 to 0xFF

PAA3905 sensor(CS_PIN);

static volatile bool motionDetect;
void myIntHandler()
{
    motionDetect = true;
}

void setup() 
{
    Serial.begin(115200);

    delay(4000);

    // Configure SPI Flash chip select
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    // Start SPI
    SPI.begin();
    delay(1000);

    // Check device ID as a test of SPI communications
    if (!sensor.begin()) {
        Debugger::reportForever("Initialization of the sensor sensor failed");
    }

    sensor.reset(); // Reset PAA3905 to return all registers to default before configuring

    sensor.setMode(DETECTION_MODE, AUTO_MODE);         // set modes

    sensor.setResolution(RESOLUTION);         // set resolution fraction of default 0x2A

    Debugger::printf("Resolution is %0.1f CPI per meter height\n", sensor.getResolution());

    sensor.setOrientation(ORIENTATION);
    uint8_t orientation = sensor.getOrientation();
    if (orientation & PAA3905::ORIENTATION_XINVERT) {
        Debugger::printf("X direction inverted!\n");
    }
    if (orientation & PAA3905::ORIENTATION_YINVERT) {
        Debugger::printf("Y direction inverted!\n");
    }
    if (orientation & PAA3905::ORIENTATION_SWAP) {
        Debugger::printf("X and Y swapped!\n");
    }

    pinMode(MOT_PIN, INPUT); // data ready interrupt
    attachInterrupt(MOT_PIN, myIntHandler, FALLING); // data ready interrupt active LOW 

    sensor.status();          // clear interrupt before entering main loop

} // setup

void loop()
{
    static uint8_t iterations;
    static uint8_t mode;
    static uint8_t dataArray[14];

    iterations++;

    // Navigation
    if (motionDetect) {

        motionDetect = false;

        sensor.readBurstMode(dataArray); // use burst mode to read all of the data
    }

    if (dataArray[0] & 0x80) {   // Check if motion data available

        if (dataArray[0]  & 0x01) Serial.println("Challenging surface detected!");

        int16_t deltaX = ((int16_t)dataArray[3] << 8) | dataArray[2];
        int16_t deltaY = ((int16_t)dataArray[5] << 8) | dataArray[4];
        uint8_t surfaceQuality = dataArray[7];      // surface quality
        uint8_t rawDataSum = dataArray[8];
        uint8_t rawDataMax = dataArray[9];
        uint8_t rawDataMin = dataArray[10];
        uint32_t shutter = ((uint32_t)dataArray[11] << 16) | ((uint32_t)dataArray[12] << 8) | dataArray[13];
        shutter &= 0x7FFFFF; // 23-bit positive integer 

        //   mode =    sensor.getMode();
        mode = (dataArray[1] & 0xC0) >> 6;  // mode is bits 6 and 7 
        // Don't report data if under thresholds
        if ((mode == bright       ) && (surfaceQuality < 25) && (shutter >= 0x00FF80)) deltaX = deltaY = 0;
        if ((mode == lowlight     ) && (surfaceQuality < 70) && (shutter >= 0x00FF80)) deltaX = deltaY = 0;
        if ((mode == superlowlight) && (surfaceQuality < 85) && (shutter >= 0x025998)) deltaX = deltaY = 0;

        // Report mode
        if (mode == bright)        Serial.println("Bright Mode"); 
        if (mode == lowlight)      Serial.println("Low Light Mode"); 
        if (mode == superlowlight) Serial.println("Super Low Light Mode"); 
        if (mode == unknown)       Serial.println("Unknown Mode"); 

        // Data and Diagnostics output
        Serial.print("X: ");Serial.print(deltaX);Serial.print(", Y: ");Serial.println(deltaY);
        Serial.print("Number of Valid Features: ");Serial.print(4*surfaceQuality);
        Serial.print(", shutter: 0x");Serial.println(shutter, HEX);
        Serial.print("Max raw Data: ");Serial.print(rawDataMax);Serial.print(", Min raw Data: ");Serial.print(rawDataMin);
        Serial.print(", Avg raw Data: ");Serial.println(rawDataSum); Serial.println(" ");
    }

    // Frame capture
    if (iterations >= 100) // capture one frame per 100 iterations (~5 sec) of navigation
    {
        iterations = 0;
        Serial.println("Hold camera still for frame capture!");
        delay(4000);

        uint32_t frameTime = millis();
        static uint8_t frameArray[1225];
        sensor.enterFrameCaptureMode();   
        sensor.captureFrame(frameArray);
        sensor.exitFrameCaptureMode(); // exit fram capture mode
        Serial.print("Frame time = "); Serial.print(millis() - frameTime); Serial.println(" ms"); Serial.println(" ");

        for (uint8_t ii = 0; ii < 35; ii++) // plot the frame data on the serial monitor (TFT display would be better)
        {
            Serial.print(ii); Serial.print(" "); 
            for (uint8_t jj = 0; jj < 35; jj++)
            {
                Serial.print(frameArray[ii*35 + jj]); Serial.print(" ");  
            }
            Serial.println(" ");
        }
        Serial.println(" ");

        sensor.exitFrameCaptureMode(); // exit fram capture mode
        Serial.print("Frame time = "); Serial.print(millis() - frameTime); Serial.println(" ms"); Serial.println(" ");

        // Return to navigation mode
        sensor.reset(); // Reset PAA3905 to return all registers to default before configuring
        delay(50);
        sensor.setMode(mode, AUTO_MODE);         // set modes
        sensor.setResolution(RESOLUTION);         // set resolution fraction of default 0x2A
        sensor.setOrientation(ORIENTATION);
        sensor.status();          // clear interrupt before entering main loop
        Serial.println("Back in Navigation mode!");
    }

    //  STM32L4.sleep();
    delay(50); // limit reporting to 20 Hz

    } // end of main loop


