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

    iterations++;

    // Navigation
    if (motionDetect) {

        motionDetect = false;

        sensor.readBurstMode(); // use burst mode to read all of the data
    }

    if (sensor.motionDataAvailable()) { 

        if (sensor.challengingSurfaceDetected()) {
            Debugger::printf("Challenging surface detected!\n");
        }

        int16_t deltaX = sensor.getDeltaX();
        int16_t deltaY = sensor.getDeltaY();

        uint8_t surfaceQuality = sensor.getSurfaceQuality();
        uint8_t rawDataSum = sensor.getRawDataSum();
        uint8_t rawDataMax = sensor.getRawDataMax();
        uint8_t rawDataMin = sensor.getRawDataMin();

        uint32_t shutter = sensor.getShutter();

        PAA3905::light_mode_t lightMode = sensor.getLightMode();

        static const char * light_mode_names[4] = {"Bright", "Low", "Super-low", "Unknown"};
        Debugger::printf("\n%s light mode\n", light_mode_names[lightMode]);

        // Don't report X,Y if surface quality and shutter are under thresholds
        if (sensor.dataAboveThresholds(lightMode, surfaceQuality, shutter)) {
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

    // Frame capture
    if (iterations >= 100) // capture one frame per 100 iterations (~5 sec) of navigation
    {
        iterations = 0;
        Debugger::printf("\nHold camera still for frame capture!\n");
        delay(4000);

        uint32_t frameTime = millis();
        static uint8_t frameArray[1225];
        sensor.enterFrameCaptureMode();   
        sensor.captureFrame(frameArray);
        sensor.exitFrameCaptureMode(); // exit fram capture mode
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

        sensor.exitFrameCaptureMode(); // exit fram capture mode
        Debugger::printf("Frame time = %d ms\n", millis() - frameTime);

        // Return to navigation mode
        sensor.reset(); // Reset PAA3905 to return all registers to default before configuring
        delay(50);
        sensor.setMode(DETECTION_MODE, AUTO_MODE); // set modes
        sensor.setResolution(RESOLUTION);         // set resolution fraction of default 0x2A
        sensor.setOrientation(ORIENTATION);
        sensor.status();          // clear interrupt before entering main loop
        Debugger::printf("Back in Navigation mode!\n");
    }

    //  STM32L4.sleep();
    delay(50); // limit reporting to 20 Hz

} // loop


