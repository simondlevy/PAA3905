/* PAA3905 Optical Flow Sensor

 * Copyright (c) 2021 Tlera Corporation and Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include <Arduino.h>

class PAA3905 {

    public:

        typedef enum {
            DETECTION_STANDARD,
            DETECTION_ENHANCED
        } detection_mode_t;

        typedef enum {
            AUTO_MODE_01,
            AUTO_MODE_012,
        } auto_mode_t;

        typedef enum {
            LIGHT_BRIGHT,
            LIGHT_LOW,
            LIGHT_SUPER_LOW,
            LIGHT_UKNOWN
        } light_mode_t;

        PAA3905(uint8_t cspin);

        boolean begin(void);

        uint8_t status();

        void initRegisters(uint8_t mode);

        void readMotionCount(int16_t *deltaX, int16_t *deltaY, uint8_t *SQUAL, uint32_t *Shutter);

        void readBurstMode(uint8_t * dataArray); 

        boolean checkID();

        void setMode(uint8_t mode, uint8_t autoSwitch);

        void setOrientation(uint8_t orient);

        uint8_t getOrientation();

        void setResolution(uint8_t res);

        uint8_t getResolution();

        void reset();

        void shutdown();

        void powerup();
        
        uint8_t getMode();

        void enterFrameCaptureMode();

        uint8_t captureFrame(uint8_t * frameArray);

        void exitFrameCaptureMode();

    private:

        static const uint8_t  PAA3905_PRODUCT_ID            = 0x00; // default value = 0xA2
        static const uint8_t  PAA3905_REVISION_ID           = 0x01;
        static const uint8_t  PAA3905_MOTION                = 0x02;
        static const uint8_t  PAA3905_DELTA_X_L             = 0x03;
        static const uint8_t  PAA3905_DELTA_X_H             = 0x04;
        static const uint8_t  PAA3905_DELTA_Y_L             = 0x05;
        static const uint8_t  PAA3905_DELTA_Y_H             = 0x06;
        static const uint8_t  PAA3905_SQUAL                 = 0x07;
        static const uint8_t  PAA3905_RAWDATA_SUM           = 0x08;
        static const uint8_t  PAA3905_MAX_RAWDATA           = 0x09;
        static const uint8_t  PAA3905_MIN_RAWDATA           = 0x0A;
        static const uint8_t  PAA3905_SHUTTER_L             = 0x0B;
        static const uint8_t  PAA3905_SHUTTER_M             = 0x0C;
        static const uint8_t  PAA3905_SHUTTER_H             = 0x0D;
        static const uint8_t  PAA3905_RAWDATA_GRAB_STATUS   = 0x10;
        static const uint8_t  PAA3905_RAWDATA_GRAB          = 0x13;
        static const uint8_t  PAA3905_OBSERVATION           = 0x15;
        static const uint8_t  PAA3905_MOTION_BURST          = 0x16;
        static const uint8_t  PAA3905_POWER_UP_RESET        = 0x3A;
        static const uint8_t  PAA3905_SHUTDOWN              = 0x3B;
        static const uint8_t  PAA3905_RESOLUTION            = 0x4E;
        static const uint8_t  PAA3905_ORIENTATION           = 0x5B;
        static const uint8_t  PAA3905_INVERSE_PRODUCT_ID    = 0x5F; // default value = 0x5D

        uint8_t _cs, _mode;

        void writeByte(uint8_t reg, uint8_t value);

        void writeByteDelay(uint8_t reg, uint8_t value);

        uint8_t readByte(uint8_t reg);

        void standardDetection(void);

        void enhancedDetection(void);
};
