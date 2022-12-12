/* PAA3905_Motion Optical Flow Sensor
 *
 * Copyright (c) 2021 Tlera Corporation and Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "PAA3905.h"

class PAA3905_MotionCapture : public PAA3905 {

    public:

        PAA3905_MotionCapture(uint8_t csPin,
                detectionMode_t detectionMode, 
                autoMode_t autoMode,     
                orientation_t orientation,
                uint8_t resolution) : PAA3905(csPin, orientation, resolution)
        { 
            m_detectionMode = detectionMode; 
            m_autoMode = autoMode;     
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

            setMode(m_detectionMode, m_autoMode);

            setResolution(m_resolution);        

            setOrientation(m_orientation);

            // Clear interrupt
            readByte(MOTION); // clears motion interrupt

            return readByte(PRODUCT_ID) == 0xA2 &&
                readByte(INVERSE_PRODUCT_ID) == 0x5D;
        }

        void readMotionCount(int16_t *deltaX, int16_t *deltaY, uint8_t *squal, uint32_t *Shutter)
        {
            *deltaX =  ((int16_t) readByte(DELTA_X_H) << 8) | readByte(DELTA_X_L);
            *deltaY =  ((int16_t) readByte(DELTA_Y_H) << 8) | readByte(DELTA_X_L);
            *squal =   readByte(SQUAL);
            *Shutter = ((uint32_t)readByte(SHUTTER_H) << 16) |
                ((uint32_t)readByte(SHUTTER_M) << 8) | readByte(SHUTTER_L);
        }

        void readBurstMode(void)
        {
            SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

            digitalWrite(m_csPin, LOW);
            delayMicroseconds(1);

            SPI.transfer(MOTION_BURST); // start burst mode
            digitalWrite(MOSI, HIGH); // hold MOSI high during burst read
            delayMicroseconds(2);

            for (uint8_t ii = 0; ii < 14; ii++) {
                m_data[ii] = SPI.transfer(0);
            }
            digitalWrite(MOSI, LOW); // return MOSI to LOW
            digitalWrite(m_csPin, HIGH);
            delayMicroseconds(1);

            SPI.endTransaction();
        }

        bool motionDataAvailable(void)
        {
            return m_data[0] & 0x80;
        }

        bool challengingSurfaceDetected(void)
        {
            return m_data[0] & 0x01;
        }

        int16_t getDeltaX(void)
        {
            return ((int16_t)m_data[3] << 8) | m_data[2];
        }

        int16_t getDeltaY(void)
        {
            return ((int16_t)m_data[5] << 8) | m_data[4];
        }

        uint8_t getSurfaceQuality(void)
        {
            return m_data[7];
        }

        uint8_t getRawDataSum(void)
        {
            return m_data[8];
        }

        uint8_t getRawDataMax(void)
        {
            return m_data[9];
        }

        uint8_t getRawDataMin(void)
        {
            return m_data[10];
        }

        uint32_t getShutter(void)
        {
            // 23-bit positive integer  
            return (((uint32_t)m_data[11] << 16) | ((uint32_t)m_data[12] << 8) | m_data[13]) & 0x7FFFFF; 
        }

        void setMode(uint8_t mode, uint8_t autoMode) 
        {
            reset();

            switch(mode) {
                case 0: // standard detection
                    standardDetection();
                    break;

                case 1: // enhanced detection
                    enhancedDetection();
                    break;
            }

            if (autoMode == AUTO_MODE_012){
                writeByteDelay(0x7F, 0x08);
                writeByteDelay(0x68, 0x02);
                writeByteDelay(0x7F, 0x00);
            }
            else
            {
                writeByteDelay(0x7F, 0x08);
                writeByteDelay(0x68, 0x01);
                writeByteDelay(0x7F, 0x00);
            }
        }

        void reset()
        {
            // Power up reset
            writeByte(POWER_UP_RESET, 0x5A);
            delay(1); 
            // Read the motion registers one time to clear
            for (uint8_t ii = 0; ii < 5; ii++)
            {
                readByte(MOTION + ii);
                delayMicroseconds(2);
            }
        }

        void shutdown()
        {
            // Enter shutdown mode
            writeByte(SHUTDOWN, 0xB6);
        }

        void powerup()
        { // exit from shutdown mode
            digitalWrite(m_csPin, HIGH);
            delay(1);
            digitalWrite(m_csPin, LOW); // reset the SPI port
            delay(1);
            // Wakeup
            writeByte(SHUTDOWN, 0xC7); // exit shutdown mode
            delay(1);
            writeByte(SHUTDOWN, 0x00); // clear shutdown register
            delay(1);
            // Read the motion registers one time to clear
            for (uint8_t ii = 0; ii < 5; ii++)
            {
                readByte(0x02 + ii);
                delayMicroseconds(2);
            }
        }

        lightMode_t getLightMode() 
        {
            return (lightMode_t)((m_data[1] & 0xC0) >> 6);  // mode is bits 6 and 7 
        }

       bool dataAboveThresholds(lightMode_t lightMode, uint8_t surfaceQuality, uint32_t shutter)
        {
            switch (lightMode) {
                case LIGHT_MODE_BRIGHT:
                    if (surfaceQuality < 25 && shutter >= 0x00FF80) {
                        return false;
                    }
                    break;
                case LIGHT_MODE_LOW:
                    if (surfaceQuality < 70 && shutter >= 0x00FF80) {
                        return false;
                    }
                    break;
                case LIGHT_MODE_SUPERLOW:
                    if (surfaceQuality < 85 && shutter >= 0x025998) {
                        return false;
                    }
                    break;
                default:
                    break;
            }

            return true;
        }

    private:

        static const uint8_t DELTA_X_L             = 0x03;
        static const uint8_t DELTA_X_H             = 0x04;
        static const uint8_t DELTA_Y_L             = 0x05;
        static const uint8_t DELTA_Y_H             = 0x06;

        detectionMode_t m_detectionMode; 
        autoMode_t      m_autoMode; 
        uint8_t         m_data[14];

        // Performance optimization registers for the three different modes
        void standardDetection() // default
        {
            writeByteDelay(0x7F, 0x00); // 1
            writeByteDelay(0x51, 0xFF);
            writeByteDelay(0x4E, 0x2A);
            writeByteDelay(0x66, 0x3E);
            writeByteDelay(0x7F, 0x14);
            writeByteDelay(0x7E, 0x71);
            writeByteDelay(0x55, 0x00);
            writeByteDelay(0x59, 0x00);
            writeByteDelay(0x6F, 0x2C);
            writeByteDelay(0x7F, 0x05); // 10

            writeByteDelay(0x4D, 0xAC); // 11
            writeByteDelay(0x4E, 0x32);
            writeByteDelay(0x7F, 0x09);
            writeByteDelay(0x5C, 0xAF);
            writeByteDelay(0x5F, 0xAF);
            writeByteDelay(0x70, 0x08);
            writeByteDelay(0x71, 0x04);
            writeByteDelay(0x72, 0x06);
            writeByteDelay(0x74, 0x3C);
            writeByteDelay(0x75, 0x28); // 20

            writeByteDelay(0x76, 0x20); //  21
            writeByteDelay(0x4E, 0xBF);
            writeByteDelay(0x7F, 0x03);
            writeByteDelay(0x64, 0x14);
            writeByteDelay(0x65, 0x0A);
            writeByteDelay(0x66, 0x10);
            writeByteDelay(0x55, 0x3C);
            writeByteDelay(0x56, 0x28);
            writeByteDelay(0x57, 0x20);
            writeByteDelay(0x4A, 0x2D); // 30

            writeByteDelay(0x4B, 0x2D); // 31
            writeByteDelay(0x4E, 0x4B);
            writeByteDelay(0x69, 0xFA);
            writeByteDelay(0x7F, 0x05);
            writeByteDelay(0x69, 0x1F);
            writeByteDelay(0x47, 0x1F);
            writeByteDelay(0x48, 0x0C);
            writeByteDelay(0x5A, 0x20);
            writeByteDelay(0x75, 0x0F);
            writeByteDelay(0x4A, 0x0F);  // 40

            writeByteDelay(0x42, 0x02);  // 41
            writeByteDelay(0x45, 0x03);
            writeByteDelay(0x65, 0x00);
            writeByteDelay(0x67, 0x76);
            writeByteDelay(0x68, 0x76);
            writeByteDelay(0x6A, 0xC5);
            writeByteDelay(0x43, 0x00);
            writeByteDelay(0x7F, 0x06);
            writeByteDelay(0x4A, 0x18);
            writeByteDelay(0x4B, 0x0C); // 50

            writeByteDelay(0x4C, 0x0C); // 51 
            writeByteDelay(0x4D, 0x0C);  
            writeByteDelay(0x46, 0x0A);
            writeByteDelay(0x59, 0xCD);
            writeByteDelay(0x7F, 0x0A);
            writeByteDelay(0x4A, 0x2A);
            writeByteDelay(0x48, 0x96);
            writeByteDelay(0x52, 0xB4);
            writeByteDelay(0x7F, 0x00);
            writeByteDelay(0x5B, 0xA0); // 60

        } // standardDetection

        void enhancedDetection()    
        {
            writeByteDelay(0x7F, 0x00); // 1
            writeByteDelay(0x51, 0xFF);
            writeByteDelay(0x4E, 0x2A);
            writeByteDelay(0x66, 0x26);
            writeByteDelay(0x7F, 0x14);
            writeByteDelay(0x7E, 0x71);
            writeByteDelay(0x55, 0x00);
            writeByteDelay(0x59, 0x00);
            writeByteDelay(0x6F, 0x2C);
            writeByteDelay(0x7F, 0x05); // 10

            writeByteDelay(0x4D, 0xAC); // 11
            writeByteDelay(0x4E, 0x65);
            writeByteDelay(0x7F, 0x09);
            writeByteDelay(0x5C, 0xAF);
            writeByteDelay(0x5F, 0xAF);
            writeByteDelay(0x70, 0x00);
            writeByteDelay(0x71, 0x00);
            writeByteDelay(0x72, 0x00);
            writeByteDelay(0x74, 0x14);
            writeByteDelay(0x75, 0x14); // 20

            writeByteDelay(0x76, 0x06); //  21
            writeByteDelay(0x4E, 0x8F);
            writeByteDelay(0x7F, 0x03);
            writeByteDelay(0x64, 0x00);
            writeByteDelay(0x65, 0x00);
            writeByteDelay(0x66, 0x00);
            writeByteDelay(0x55, 0x14);
            writeByteDelay(0x56, 0x14);
            writeByteDelay(0x57, 0x06);
            writeByteDelay(0x4A, 0x20); // 30

            writeByteDelay(0x4B, 0x20); // 31
            writeByteDelay(0x4E, 0x32);
            writeByteDelay(0x69, 0xFE);
            writeByteDelay(0x7F, 0x05);
            writeByteDelay(0x69, 0x14);
            writeByteDelay(0x47, 0x14);
            writeByteDelay(0x48, 0x1C);
            writeByteDelay(0x5A, 0x20);
            writeByteDelay(0x75, 0xE5);
            writeByteDelay(0x4A, 0x05);  // 40

            writeByteDelay(0x42, 0x04);  // 41
            writeByteDelay(0x45, 0x03);
            writeByteDelay(0x65, 0x00);
            writeByteDelay(0x67, 0x50);
            writeByteDelay(0x68, 0x50);
            writeByteDelay(0x6A, 0xC5);
            writeByteDelay(0x43, 0x00);
            writeByteDelay(0x7F, 0x06);
            writeByteDelay(0x4A, 0x1E);
            writeByteDelay(0x4B, 0x1E); // 50

            writeByteDelay(0x4C, 0x34); // 51 
            writeByteDelay(0x4D, 0x34);  
            writeByteDelay(0x46, 0x32);
            writeByteDelay(0x59, 0x0D);
            writeByteDelay(0x7F, 0x0A);
            writeByteDelay(0x4A, 0x2A);
            writeByteDelay(0x48, 0x96);
            writeByteDelay(0x52, 0xB4);
            writeByteDelay(0x7F, 0x00);
            writeByteDelay(0x5B, 0xA0); // 60

        } // enhancedDetection

        // XXX useful?
        void exitFrameCaptureMode()
        {
            writeByteDelay(0x7F, 0x00);
            writeByteDelay(0x55, 0x00);
            writeByteDelay(0x7F, 0x13);
            writeByteDelay(0x42, 0x00);
            writeByteDelay(0x7F, 0x00);
            writeByteDelay(0x67, 0xA5);
        }
 
}; // class PAA3905_Motion
