/* PAA3905 Optical Flow Sensor
 *
 * Copyright (c) 2021 Tlera Corporation and Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "Arduino.h"

#define PAA3905_PRODUCT_ID            0x00 // default value 0xA2
#define PAA3905_REVISION_ID           0x01
#define PAA3905_MOTION                0x02
#define PAA3905_DELTA_X_L             0x03
#define PAA3905_DELTA_X_H             0x04
#define PAA3905_DELTA_Y_L             0x05
#define PAA3905_DELTA_Y_H             0x06
#define PAA3905_SQUAL                 0x07
#define PAA3905_RAWDATA_SUM           0x08
#define PAA3905_MAX_RAWDATA           0x09
#define PAA3905_MIN_RAWDATA           0x0A
#define PAA3905_SHUTTER_L             0x0B
#define PAA3905_SHUTTER_M             0x0C
#define PAA3905_SHUTTER_H             0x0D
#define PAA3905_RAWDATA_GRAB_STATUS   0x10
#define PAA3905_RAWDATA_GRAB          0x13
#define PAA3905_OBSERVATION           0x15
#define PAA3905_MOTION_BURST          0x16
#define PAA3905_POWER_UP_RESET        0x3A
#define PAA3905_SHUTDOWN              0x3B
#define PAA3905_RESOLUTION            0x4E
#define PAA3905_ORIENTATION           0x5B
#define PAA3905_INVERSE_PRODUCT_ID    0x5F // default value 0x5D

#define standardDetectionMode      0
#define enhancedDetectionMode      1

#define autoMode012       0
#define autoMode01        1

#define bright        0
#define lowlight      1
#define superlowlight 2
#define unknown       3

class PAA3905 {
public:
  PAA3905(uint8_t cspin);
  void begin(void);
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
  void captureFrame(uint8_t * frameArray);
  void exitFrameCaptureMode();

private:
  uint8_t _cs, _mode;
  void writeByte(uint8_t reg, uint8_t value);
  void writeByteDelay(uint8_t reg, uint8_t value);
  uint8_t readByte(uint8_t reg);
  void standardDetection(void);
  void enhancedDetection(void);
};
