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

// Pin definitions
#define CSPIN  10  // default chip select for SPI
#define MOSI   11  // SPI MOSI pin on Ladybug required for frame capture
#define MOT     8  // use as data ready interrupt

// PAA3905 configuration

static PAA3905::detection_mode_t DETECTION_MODE = PAA3905::DETECTION_STANDARD;

static uint8_t autoMode = autoMode01;        
static uint8_t pixelRes = 0x2A;  
static float resolution;        
static uint8_t orientation, orient; 
static int16_t deltaX, deltaY;
static uint32_t Shutter;
static volatile bool motionDetect;
static uint8_t statusCheck;
static uint8_t frameArray[1225], dataArray[14], SQUAL, RawDataSum = 0, RawDataMin = 0, RawDataMax = 0;
static uint8_t iterations;
static uint32_t frameTime;

PAA3905 opticalFlow(CSPIN); // Instantiate PAA3905

void setup() 
{
    Serial.begin(115200);

    delay(4000);

    pinMode(MOT, INPUT); // data ready interrupt

    // Configure SPI Flash chip select
    pinMode(CSPIN, OUTPUT);
    digitalWrite(CSPIN, HIGH);

    SPI.begin(); // initiate SPI 
    delay(1000);

    opticalFlow.begin();  // Prepare SPI port 

    // Check device ID as a test of SPI communications
    if (!opticalFlow.checkID()) {
        Serial.println("Initialization of the opticalFlow sensor failed");
        while(1) { }
    }

    opticalFlow.reset(); // Reset PAA3905 to return all registers to default before configuring

    opticalFlow.setMode(DETECTION_MODE, autoMode);         // set modes

    opticalFlow.setResolution(pixelRes);         // set resolution fraction of default 0x2A
    resolution = (opticalFlow.getResolution() + 1) * 200.0f/8600.0f; // == 1 if pixelRes == 0x2A
    Serial.print("Resolution is: "); Serial.print(resolution * 11.914f, 1); Serial.println(" CPI per meter height"); Serial.println(" ");

    opticalFlow.setOrientation(orient);
    orientation = opticalFlow.getOrientation();
    if(orientation & 0x80) Serial.println("X direction inverted!"); Serial.println(" ");
    if(orientation & 0x40) Serial.println("Y direction inverted!"); Serial.println(" ");
    if(orientation & 0x20) Serial.println("X and Y swapped!"); Serial.println(" ");

    attachInterrupt(MOT, myIntHandler, FALLING); // data ready interrupt active LOW 

    statusCheck = opticalFlow.status();          // clear interrupt before entering main loop

} // setup

void loop()
{
    static uint8_t mode;
    iterations++;

    // Navigation
    if(motionDetect){
        motionDetect = false;

        //   statusCheck = opticalFlow.status(); // clear interrupt
        //   if(statusCheck & 0x01) Serial.println("Challenging surface detected!");
        //   if(statusCheck & 0x80) { //Check that motion data is available
        //   opticalFlow.readMotionCount(&deltaX, &deltaY, &SQUAL, &Shutter);  // read some of the data
        opticalFlow.readBurstMode(dataArray); // use burst mode to read all of the data
    }

    if(dataArray[0] & 0x80) {   // Check if motion data available

        if(dataArray[0]  & 0x01) Serial.println("Challenging surface detected!");

        deltaX = ((int16_t)dataArray[3] << 8) | dataArray[2];
        deltaY = ((int16_t)dataArray[5] << 8) | dataArray[4];
        SQUAL = dataArray[7];      // surface quality
        RawDataSum = dataArray[8];
        RawDataMax = dataArray[9];
        RawDataMin = dataArray[10];
        Shutter = ((uint32_t)dataArray[11] << 16) | ((uint32_t)dataArray[12] << 8) | dataArray[13];
        Shutter &= 0x7FFFFF; // 23-bit positive integer 

        //   mode =    opticalFlow.getMode();
        mode = (dataArray[1] & 0xC0) >> 6;  // mode is bits 6 and 7 
        // Don't report data if under thresholds
        if((mode == bright       ) && (SQUAL < 25) && (Shutter >= 0x00FF80)) deltaX = deltaY = 0;
        if((mode == lowlight     ) && (SQUAL < 70) && (Shutter >= 0x00FF80)) deltaX = deltaY = 0;
        if((mode == superlowlight) && (SQUAL < 85) && (Shutter >= 0x025998)) deltaX = deltaY = 0;

        // Report mode
        if(mode == bright)        Serial.println("Bright Mode"); 
        if(mode == lowlight)      Serial.println("Low Light Mode"); 
        if(mode == superlowlight) Serial.println("Super Low Light Mode"); 
        if(mode == unknown)       Serial.println("Unknown Mode"); 

        // Data and Diagnostics output
        Serial.print("X: ");Serial.print(deltaX);Serial.print(", Y: ");Serial.println(deltaY);
        Serial.print("Number of Valid Features: ");Serial.print(4*SQUAL);
        Serial.print(", Shutter: 0x");Serial.println(Shutter, HEX);
        Serial.print("Max Raw Data: ");Serial.print(RawDataMax);Serial.print(", Min Raw Data: ");Serial.print(RawDataMin);
        Serial.print(", Avg Raw Data: ");Serial.println(RawDataSum); Serial.println(" ");
    }

    // Frame capture
    if(iterations >= 100) // capture one frame per 100 iterations (~5 sec) of navigation
    {
        iterations = 0;
        Serial.println("Hold camera still for frame capture!");
        delay(4000);

        frameTime = millis();
        opticalFlow.enterFrameCaptureMode();   
        opticalFlow.captureFrame(frameArray);
        opticalFlow.exitFrameCaptureMode(); // exit fram capture mode
        Serial.print("Frame time = "); Serial.print(millis() - frameTime); Serial.println(" ms"); Serial.println(" ");

        for(uint8_t ii = 0; ii < 35; ii++) // plot the frame data on the serial monitor (TFT display would be better)
        {
            Serial.print(ii); Serial.print(" "); 
            for(uint8_t jj = 0; jj < 35; jj++)
            {
                Serial.print(frameArray[ii*35 + jj]); Serial.print(" ");  
            }
            Serial.println(" ");
        }
        Serial.println(" ");

        opticalFlow.exitFrameCaptureMode(); // exit fram capture mode
        Serial.print("Frame time = "); Serial.print(millis() - frameTime); Serial.println(" ms"); Serial.println(" ");

        // Return to navigation mode
        opticalFlow.reset(); // Reset PAA3905 to return all registers to default before configuring
        delay(50);
        opticalFlow.setMode(mode, autoMode);         // set modes
        opticalFlow.setResolution(pixelRes);         // set resolution fraction of default 0x2A
        opticalFlow.setOrientation(orient);          // set orientation
        statusCheck = opticalFlow.status();          // clear interrupt before entering main loop
        Serial.println("Back in Navigation mode!");
    }

    //  STM32L4.sleep();
    delay(50); // limit reporting to 20 Hz

    } // end of main loop


    void myIntHandler()
    {
        motionDetect = true;
    }
