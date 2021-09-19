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

// Sensor configuration
static PAA3905::detection_mode_t DETECTION_MODE = PAA3905::DETECTION_STANDARD;
static PAA3905::auto_mode_t      AUTO_MODE      = PAA3905::AUTO_MODE_01; 
static PAA3905::orientation_t    ORIENTATION    = PAA3905::ORIENTATION_NORMAL;
static uint8_t                   RESOLUTION     = 0x2A;

static PAA3905 sensor(CS_PIN);

static volatile bool gotMotionInterrupt;
static void interruptHandler()
{
    gotMotionInterrupt = true;
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

    // Reset PAA3905 to return all registers to default before configuring
    sensor.reset(); 

    sensor.setMode(DETECTION_MODE, AUTO_MODE); 

    sensor.setResolution(RESOLUTION);         
    float resolution = (sensor.getResolution() + 1) * 200.0f/8600.0f; 
    Debugger::printf("Resolution is: %f CPI per meter height", resolution * 11.914f);

    sensor.setOrientation(ORIENTATION);
    uint8_t orientation = sensor.getOrientation();
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

    sensor.status();          // clear interrupt before entering main loop

    //  sensor.shutdown();                    // enter lowest power mode until ready to use

} // setup

void loop() {

    static uint8_t frameArray[1225];
    static uint8_t iterations;

    iterations++;

    if (gotMotionInterrupt){

        gotMotionInterrupt = false;

        sensor.readBurstMode(); // use burst mode to read all of the data
    }

    if (sensor.motionDataAvailable()) {

        if (sensor.challengingSurfaceDetected()) {
            Debugger::printf("Challenging surface detected!\n");
        }

        uint16_t deltaX = sensor.getDeltaX();
        uint16_t deltaY = sensor.getDeltaY();
        uint8_t squal = sensor.getSurfaceQuality();
        uint8_t rawDataSum = sensor.getRawDataSum();
        uint8_t rawDataMax = sensor.getRawDataMax();
        uint8_t rawDataMin = sensor.getRawDataMin();
        uint32_t shutter = sensor.getShutter(); // 23-bit positive integer 
        uint8_t lightMode = sensor.getLightMode();

        // Don't report data if under thresholds
        if ((lightMode == PAA3905::LIGHT_BRIGHT) && (squal < 25) && (shutter >= 0x00FF80)) deltaX = deltaY = 0;
        if ((lightMode == PAA3905::LIGHT_LOW) && (squal < 70) && (shutter >= 0x00FF80)) deltaX = deltaY = 0;
        if ((lightMode == PAA3905::LIGHT_SUPER_LOW) && (squal < 85) && (shutter >= 0x025998)) deltaX = deltaY = 0;

        // Report mode
        switch (lightMode) {
            case PAA3905::LIGHT_BRIGHT:
                Debugger::printf("Bright Mode\n");
                break;
            case PAA3905::LIGHT_LOW:
                Debugger::printf("Low Light Mode\n");
                break;
            case PAA3905::LIGHT_SUPER_LOW:
                Debugger::printf("Super Low Light Mode\n");
                break;
            default:
                Debugger::printf("Unknown Mode\n");
        }

        // Data and Diagnostics output
        Debugger::printf("X: %d , Y: %d\n", deltaX, deltaY);
        Debugger::printf("Number of Valid Features: %d, shutter: 0x%02X\n", 4*squal, shutter);
        Debugger::printf("Max raw Data: %d, Min raw Data: %d, Avg rawData: %d\n\n",
                rawDataMax, rawDataMin, rawDataSum);
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
        sensor.setMode(DETECTION_MODE, AUTO_MODE);         // set modes
        sensor.setResolution(RESOLUTION);         // set resolution fraction of default 0x2A
        sensor.setOrientation(ORIENTATION);     // set orientation
        sensor.status();          // clear interrupt before entering main loop
        Debugger::printf("Back in Navigation mode!\n");
    }

    delay(50); // limit reporting to 20 Hz

    } // loop
