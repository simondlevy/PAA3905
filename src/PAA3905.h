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

        typedef enum {
            ORIENTATION_NORMAL  = 0x00,
            ORIENTATION_XINVERT = 0x80,
            ORIENTATION_YINVERT = 0x40,
            ORIENTATION_SWAP    = 0x20,
        } orientation_t;

        PAA3905(uint8_t cspin);

        void begin(void);

        uint8_t status();

        void initRegisters(uint8_t mode);

        void readBurstMode(void); 

        bool motionDataAvailable(void);

        bool challengingSurfaceDetected(void);

        uint16_t getDeltaX(void);

        uint16_t getDeltaY(void);

        uint8_t getSurfaceQuality(void);

        uint8_t getRawDataSum(void);
        
        uint8_t getRawDataMax(void);

        uint8_t getRawDataMin(void);

        uint8_t getLightMode(void);

        boolean checkID();

        void setMode(uint8_t mode, uint8_t autoSwitch);

        void setOrientation(uint8_t orient);

        uint8_t getOrientation();

        void setResolution(uint8_t res);

        float getResolution();

        void reset();

        void shutdown();

        void powerup();

        uint8_t getMode();

        void enterFrameCaptureMode();

        void captureFrame(uint8_t * frameArray);

        void exitFrameCaptureMode();

        bool goodQuality(uint8_t lightMode, uint8_t shutterQuality);

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

        uint8_t _cs = 0;

        uint8_t _data[14] = {};

        void writeByte(uint8_t reg, uint8_t value);

        void writeByteDelay(uint8_t reg, uint8_t value);

        uint8_t readByte(uint8_t reg);

        void standardDetection(void);

        void enhancedDetection(void);
};
