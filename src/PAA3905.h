/* PAA3905_Motion Optical Flow Sensor
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
        } lightMode_t;

        typedef enum {
            DETECTION_STANDARD,
            DETECTION_ENHANCED
        } detectionMode_t;

        typedef enum {
            AUTO_MODE_01,
            AUTO_MODE_012,
        } autoMode_t;

        typedef enum {
            ORIENTATION_NORMAL  = 0x00,
            ORIENTATION_XINVERT = 0x80,
            ORIENTATION_YINVERT = 0x40,
            ORIENTATION_SWAP    = 0x20,
        } orientation_t;

        bool begin(void) 
        {
            // Configure SPI Flash chip select
            pinMode(m_csPin, OUTPUT);
            digitalWrite(m_csPin, HIGH);

            // Setup SPI port
            // 2 MHz max SPI clock frequency
            m_spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3)); 

            // Make sure the SPI bus is reset
            digitalWrite(m_csPin, HIGH);
            delay(1);
            digitalWrite(m_csPin, LOW);
            delay(1);
            digitalWrite(m_csPin, HIGH);
            delay(1);

            m_spi->endTransaction();

            // Return all registers to default before configuring
            reset(); 

            initMode();

            setResolution(m_resolution);        
            setOrientation(m_orientation);

            // Clear interrupt
            readByte(MOTION); // clears motion interrupt

            return readByte(PRODUCT_ID) == 0xA2 &&
                readByte(INVERSE_PRODUCT_ID) == 0x5D;
        }

        float getResolution() 
        {
            return (readByte(RESOLUTION) + 1) * 200.0f / 8600 * 11.914;
        }

    protected:

        uint8_t m_csPin;

        PAA3905(
                SPIClass & spi,
                const uint8_t csPin,
                const orientation_t orientation,
                const uint8_t resolution)
        { 
            m_spi = &spi;
            m_csPin = csPin;
            m_orientation = orientation;
            m_resolution = resolution;
        }

        virtual void initMode(void) = 0;

        void setMode(const uint8_t mode, const uint8_t autoMode) 
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

        void writeByte(const uint8_t reg, const uint8_t value) 
        {
            m_spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
            digitalWrite(m_csPin, LOW);
            delayMicroseconds(1);

            m_spi->transfer(reg | 0x80);
            delayMicroseconds(10);
            m_spi->transfer(value);
            delayMicroseconds(1);

            digitalWrite(m_csPin, HIGH);
            m_spi->endTransaction();
        }

        void writeByteDelay(const uint8_t reg, const uint8_t value)
        {
            writeByte(reg, value);
            delayMicroseconds(11);
        }

        uint8_t readByte(const uint8_t reg) 
        {
            m_spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
            digitalWrite(m_csPin, LOW);
            delayMicroseconds(1);

            m_spi->transfer(reg & 0x7F);
            delayMicroseconds(2);

            uint8_t temp = m_spi->transfer(0);
            delayMicroseconds(1);

            digitalWrite(m_csPin, HIGH);
            m_spi->endTransaction();

            return temp;
        }

        /*
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
        */

    private:

        static const uint8_t PRODUCT_ID          = 0x00; // default value = 0xA2
        static const uint8_t MOTION              = 0x02;
        static const uint8_t POWER_UP_RESET      = 0x3A;
        static const uint8_t SHUTDOWN            = 0x3B;
        static const uint8_t RESOLUTION          = 0x4E;
        static const uint8_t ORIENTATION         = 0x5B;
        static const uint8_t INVERSE_PRODUCT_ID  = 0x5F ;// default value = 0x5D

        SPIClass * m_spi;

        orientation_t m_orientation;

        uint8_t       m_resolution;

        void setResolution(const uint8_t res) 
        {
            writeByte(RESOLUTION, res);
        }

        void setOrientation(const uint8_t orient) 
        {
            writeByte(ORIENTATION, orient);
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

}; // class PAA3905
