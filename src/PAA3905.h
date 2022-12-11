/* PAA3905 Optical Flow Sensor
 *
 * Copyright (c) 2021 Tlera Corporation and Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>

class PAA3905 {

    public:

        typedef enum {
            LIGHT_MODE_BRIGHT,
            LIGHT_MODE_LOW,
            LIGHT_MODE_SUPERLOW,
            LIGHT_MODE_UNKNOWN
        } light_mode_t;

        typedef enum {
            DETECTION_STANDARD,
            DETECTION_ENHANCED
        } detection_mode_t;

        typedef enum {
            AUTO_MODE_01,
            AUTO_MODE_012,
        } auto_mode_t;

        typedef enum {
            ORIENTATION_NORMAL  = 0x00,
            ORIENTATION_XINVERT = 0x80,
            ORIENTATION_YINVERT = 0x40,
            ORIENTATION_SWAP    = 0x20,
        } orientation_t;

        PAA3905(uint8_t cspin)
            : _cs(cspin)
        { 
        }

        bool begin(void) 
        {
            // Setup SPI port
            // 2 MHz max SPI clock frequency
            SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3)); 

            // Make sure the SPI bus is reset
            digitalWrite(_cs, HIGH);
            delay(1);
            digitalWrite(_cs, LOW);
            delay(1);
            digitalWrite(_cs, HIGH);
            delay(1);

            SPI.endTransaction();

            return readByte(PAA3905_PRODUCT_ID) == 0xA2 &&
                readByte(PAA3905_INVERSE_PRODUCT_ID) == 0x5D;
        }

        uint8_t status()
        {
            uint8_t temp = readByte(PAA3905_MOTION); // clears motion interrupt
            return temp;
        }

        void initRegisters(uint8_t mode)
        {
            switch(mode)
            {
                case 0: // standard detection
                    standardDetection();
                    break;

                case 1: // enhanced detection
                    enhancedDetection();
                    break;
            }
        }

        void readMotionCount(int16_t *deltaX, int16_t *deltaY, uint8_t *SQUAL, uint32_t *Shutter)
        {
            *deltaX =  ((int16_t) readByte(PAA3905_DELTA_X_H) << 8) | readByte(PAA3905_DELTA_X_L);
            *deltaY =  ((int16_t) readByte(PAA3905_DELTA_Y_H) << 8) | readByte(PAA3905_DELTA_X_L);
            *SQUAL =   readByte(PAA3905_SQUAL);
            *Shutter = ((uint32_t)readByte(PAA3905_SHUTTER_H) << 16) |
                ((uint32_t)readByte(PAA3905_SHUTTER_M) << 8) | readByte(PAA3905_SHUTTER_L);
        }

        void readBurstMode(void)
        {
            SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

            digitalWrite(_cs, LOW);
            delayMicroseconds(1);

            SPI.transfer(PAA3905_MOTION_BURST); // start burst mode
            digitalWrite(MOSI, HIGH); // hold MOSI high during burst read
            delayMicroseconds(2);

            for (uint8_t ii = 0; ii < 14; ii++) {
                _data[ii] = SPI.transfer(0);
            }
            digitalWrite(MOSI, LOW); // return MOSI to LOW
            digitalWrite(_cs, HIGH);
            delayMicroseconds(1);

            SPI.endTransaction();
        }


        bool motionDataAvailable(void)
        {
            return _data[0] & 0x80;
        }

        bool challengingSurfaceDetected(void)
        {
            return _data[0] & 0x01;
        }

        int16_t getDeltaX(void)
        {
            return ((int16_t)_data[3] << 8) | _data[2];
        }

        int16_t getDeltaY(void)
        {
            return ((int16_t)_data[5] << 8) | _data[4];
        }

        uint8_t getSurfaceQuality(void)
        {
            return _data[7];
        }

        uint8_t getRawDataSum(void)
        {
            return _data[8];
        }

        uint8_t getRawDataMax(void)
        {
            return _data[9];
        }

        uint8_t getRawDataMin(void)
        {
            return _data[10];
        }

        uint32_t getShutter(void)
        {
            // 23-bit positive integer  
            return (((uint32_t)_data[11] << 16) | ((uint32_t)_data[12] << 8) | _data[13]) & 0x7FFFFF; 
        }

        void setMode(uint8_t mode, uint8_t autoSwitch);

        void setOrientation(uint8_t orient);

        uint8_t getOrientation();

        void setResolution(uint8_t res);

        float getResolution();

        void reset();

        void shutdown();

        void powerup();

        light_mode_t getLightMode();

        void enterFrameCaptureMode();

        void captureFrame(uint8_t * frameArray);

        void exitFrameCaptureMode();

        static bool dataAboveThresholds(
                light_mode_t lightMode,
                uint8_t surfaceQuality,
                uint32_t shutter);

    private:

        static const uint8_t PAA3905_PRODUCT_ID            = 0x00; // default value = 0xA2
        static const uint8_t PAA3905_REVISION_ID           = 0x01;
        static const uint8_t PAA3905_MOTION                = 0x02;
        static const uint8_t PAA3905_DELTA_X_L             = 0x03;
        static const uint8_t PAA3905_DELTA_X_H             = 0x04;
        static const uint8_t PAA3905_DELTA_Y_L             = 0x05;
        static const uint8_t PAA3905_DELTA_Y_H             = 0x06;
        static const uint8_t PAA3905_SQUAL                 = 0x07;
        static const uint8_t PAA3905_RAWDATA_SUM           = 0x08;
        static const uint8_t PAA3905_MAX_RAWDATA           = 0x09;
        static const uint8_t PAA3905_MIN_RAWDATA           = 0x0A;
        static const uint8_t PAA3905_SHUTTER_L             = 0x0B;
        static const uint8_t PAA3905_SHUTTER_M             = 0x0C;
        static const uint8_t PAA3905_SHUTTER_H             = 0x0D;
        static const uint8_t PAA3905_RAWDATA_GRAB_STATUS   = 0x10;
        static const uint8_t PAA3905_RAWDATA_GRAB          = 0x13;
        static const uint8_t PAA3905_OBSERVATION           = 0x15;
        static const uint8_t PAA3905_MOTION_BURST          = 0x16;
        static const uint8_t PAA3905_POWER_UP_RESET        = 0x3A;
        static const uint8_t PAA3905_SHUTDOWN              = 0x3B;
        static const uint8_t PAA3905_RESOLUTION            = 0x4E;
        static const uint8_t PAA3905_ORIENTATION           = 0x5B;
        static const uint8_t PAA3905_INVERSE_PRODUCT_ID    = 0x5F ;// default value = 0x5D

        uint8_t _cs = 0;

        uint8_t _data[14] = {};

        void writeByte(uint8_t reg, uint8_t value);

        void writeByteDelay(uint8_t reg, uint8_t value);

        uint8_t readByte(uint8_t reg);

        void standardDetection(void);

        void enhancedDetection(void);
};
