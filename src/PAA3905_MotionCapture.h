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

        PAA3905_MotionCapture(
                SPIClass & spi,
                uint8_t csPin,
                detectionMode_t detectionMode, 
                autoMode_t autoMode,     
                orientation_t orientation,
                uint8_t resolution) : PAA3905(spi, csPin, orientation, resolution)
        { 
            m_detectionMode = detectionMode; 
            m_autoMode = autoMode;     
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

    protected:

       virtual void initMode(void) override 
       {
            setMode(m_detectionMode, m_autoMode);
       }

    private:

        static const uint8_t DELTA_X_L    = 0x03;
        static const uint8_t DELTA_X_H    = 0x04;
        static const uint8_t DELTA_Y_L    = 0x05;
        static const uint8_t DELTA_Y_H    = 0x06;
        static const uint8_t SQUAL        = 0x07;
        static const uint8_t SHUTTER_L    = 0x0B;
        static const uint8_t SHUTTER_M    = 0x0C;
        static const uint8_t SHUTTER_H    = 0x0D;
        static const uint8_t MOTION_BURST = 0x16;

        detectionMode_t m_detectionMode; 
        autoMode_t      m_autoMode; 
        uint8_t         m_data[14];

}; // class PAA3905_Motion
