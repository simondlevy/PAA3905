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

        PAA3905_FrameCapture(uint8_t csPin, orientation_t orientation, uint8_t resolution) 
            : PAA3905(csPin, orientation, resolution)
        { 
        }

        bool begin(void) 
        {
            // Configure SPI Flash chip select
            pinMode(m_csPin, OUTPUT);
            digitalWrite(m_csPin, HIGH);

            // Setup SPI port
            // 2 MHz max SPI clock frequency
            SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3)); 

            // Make sure the SPI bus is reset
            digitalWrite(m_csPin, HIGH);
            delay(1);
            digitalWrite(m_csPin, LOW);
            delay(1);
            digitalWrite(m_csPin, HIGH);
            delay(1);

            SPI.endTransaction();

            // Return all registers to default before configuring
            reset(); 

            setResolution(m_resolution);        

            setOrientation(m_orientation);

            // Clear interrupt
            readByte(MOTION); // clears motion interrupt

            return readByte(PRODUCT_ID) == 0xA2 &&
                readByte(INVERSE_PRODUCT_ID) == 0x5D;
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

            for (uint8_t ii = 0; ii < 35; ii++) {

                for (uint8_t jj = 0; jj < 35; jj++) {

                    // read the 1225 data into array
                    frameArray[ii*35 + jj] = readByte(RAWDATA_GRAB); 
                }
            }
        }

    private:

        static const uint8_t RAWDATA_SUM           = 0x08;
        static const uint8_t MAX_RAWDATA           = 0x09;
        static const uint8_t MIN_RAWDATA           = 0x0A;
        static const uint8_t RAWDATA_GRAB_STATUS   = 0x10;
        static const uint8_t RAWDATA_GRAB          = 0x13;

}; // class PAA3905_FrameCapture
