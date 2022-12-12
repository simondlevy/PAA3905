/* PAA3905_FrameCapture Optical Flow Sensor
 *
 * Copyright (c) 2021 Tlera Corporation and Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "PAA3905.h"

class PAA3905_FrameCapture : public PAA3905 {

    public:

        PAA3905_FrameCapture(
                SPIClass & spi,
                const uint8_t csPin,
                const orientation_t orientation,
                const uint8_t resolution) 
            : PAA3905(spi, csPin, orientation, resolution)
        { 
        }

        void captureFrame(uint8_t * frameArray)
        {  
            // make sure not in superlowlight mode for frame capture
            setMode(DETECTION_STANDARD, AUTO_MODE_01); 

            writeByteDelay(0x7F, 0x00);
            writeByteDelay(0x67, 0x25);
            writeByteDelay(0x55, 0x20);
            writeByteDelay(0x7F, 0x13);
            writeByteDelay(0x42, 0x01);
            writeByteDelay(0x7F, 0x00);
            writeByteDelay(0x0F, 0x11);
            writeByteDelay(0x0F, 0x13);
            writeByteDelay(0x0F, 0x11);

            uint8_t tempStatus = 0;

            // wait for grab status bit 0 to equal 1
            while( !(tempStatus & 0x01) ) {
                tempStatus = readByte(RAWDATA_GRAB_STATUS); 
            } 

            writeByteDelay(RAWDATA_GRAB, 0xFF); // start frame capture mode

            for (uint8_t j = 0; j < 35; j++) {

                for (uint8_t k = 0; k < 35; k++) {

                    // read the 1225 data into array
                    frameArray[j*35 + k] = readByte(RAWDATA_GRAB); 
                }
            }
        }

    protected:

       virtual void initMode(void) override 
       {
           // mode will be set in captureFrame()
       }

    private:

       enum {

           RAWDATA_SUM           = 0x08,
           MAX_RAWDATA           = 0x09,
           MIN_RAWDATA           = 0x0A,
           RAWDATA_GRAB_STATUS   = 0x10,
           RAWDATA_GRAB          = 0x13
       };

}; // class PAA3905_FrameCapture
