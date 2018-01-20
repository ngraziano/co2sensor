#pragma once
#ifndef LIB_CCS811_H
#define LIB_CCS811_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define CCS811_ADDRESS (0x5A)
#define CCS811_ADDRESS_ALT (0x5B)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum class Ccs811Register : uint8_t {
  status = 0x00,
  meas_mode = 0x01,
  alg_result_data = 0x02,
  raw_data = 0x03,
  env_data = 0x05,
  ntc = 0x06,
  thresholds = 0x10,
  baseline = 0x11,
  hw_id = 0x20,
  hw_version = 0x21,
  fw_boot_version = 0x23,
  fw_app_version = 0x24,
  error_id = 0xe0,
  app_erase = 0xf1,
  app_data = 0xf2,
  app_verify = 0xf3,
  app_start = 0xf4,
  sw_reset = 0xff,
};

enum class Ccs811Drive : uint8_t {
  mode_idle = 0x00,
  mode_1sec = 0x01,
  mode_10sec = 0x02,
  mode_60sec = 0x03,
  mode_250ms = 0x04,
};

/*=========================================================================*/

#define CCS811_HW_ID_CODE 0x81

#define CCS811_REF_RESISTOR 100000

class CCS811 {
public:
  // constructors
  CCS811(void){};
  ~CCS811(void){};

  bool begin(uint8_t addr = CCS811_ADDRESS);

  uint8_t setEnvironmentalData(uint8_t humidity, double temperature);

  // calculate temperature based on the NTC register
  double calculateTemperature();

  uint8_t setThresholds(uint16_t low_med, uint16_t med_high,
                     uint8_t hysteresis = 50);

  uint8_t SWReset();

  uint8_t setDriveMode(Ccs811Drive mode);
  uint8_t enableInterrupt();
  uint8_t disableInterrupt();

  uint16_t getTVOC() { return _TVOC; }
  uint16_t geteCO2() { return _eCO2; }

  void setTempOffset(float offset) { _tempOffset = offset; }

  bool isI2CError() { return _lastI2cError == 0; }

  // check if data is available to be read
  bool available();
  bool readData();

  bool checkError();

private:
  uint8_t _i2caddr;
  float _tempOffset;

  uint16_t _TVOC;
  uint16_t _eCO2;
  uint8_t _lastI2cError;

  uint8_t read(Ccs811Register reg, uint8_t *buf, uint8_t num);
  uint8_t write(Ccs811Register reg, uint8_t *buf, uint8_t num);
  void _i2c_init();

  /*=========================================================================
          REGISTER BITFIELDS
      -----------------------------------------------------------------------*/
  // The status register
  struct status {

    uint8_t
        /* 0: no error
         * 1: error has occurred
         */
        ERROR : 1,

        // reserved
        : 2,

        /* 0: no samples are ready
         * 1: samples are ready
         */
        DATA_READY : 1,
        // Application status
        APP_VALID : 1,

        // reserved
        : 2,

        /* 0: boot mode, new firmware can be loaded
         *  1: application mode, can take measurements
         */
        FW_MODE : 1;

    void set(uint8_t data) {
      ERROR = data & 0x01;
      DATA_READY = (data >> 3) & 0x01;
      APP_VALID = (data >> 4) & 0x01;
      FW_MODE = (data >> 7) & 0x01;
    }
  };
  status _status;

  // measurement and conditions register
  struct meas_mode {
    uint8_t
        // reserved
        : 2,

        /*
        0: interrupt mode operates normally
        1: Interrupt mode (if enabled) only asserts the nINT signal (driven low)
           if the new ALG_RESULT_DATA crosses one of the thresholds set in the
           THRESHOLDS register by more than the hysteresis value (also in the
           THRESHOLDS register)
        */
        INT_THRESH : 1,
        /*
        0: int disabled
        1: The nINT signal is asserted (driven low) when a new sample is ready
           in ALG_RESULT_DATA. The nINT signal will stop being driven low when
           ALG_RESULT_DATA is read on the I²C interface.
        */
        INT_DATARDY : 1,
        // mode
        DRIVE_MODE : 3;

    uint8_t get() {
      return (INT_THRESH << 2) | (INT_DATARDY << 3) | (DRIVE_MODE << 4);
    }
  };
  meas_mode _meas_mode;

  struct error_id {
    /* The CCS811 received an I²C write request addressed to this station but
       with invalid register address ID */
    uint8_t WRITE_REG_INVALID : 1;

    /* The CCS811 received an I²C read request to a mailbox ID that is invalid
     */
    uint8_t READ_REG_INVALID : 1;

    /* The CCS811 received an I²C request to write an unsupported mode to
            MEAS_MODE */
    uint8_t MEASMODE_INVALID : 1;

    /* The sensor resistance measurement has reached or exceeded the maximum
            range */
    uint8_t MAX_RESISTANCE : 1;

    /* The Heater current in the CCS811 is not in range */
    uint8_t HEATER_FAULT : 1;

    /*  The Heater voltage is not being applied correctly */
    uint8_t HEATER_SUPPLY : 1;

    void set(uint8_t data) {
      WRITE_REG_INVALID = data & 0x01;
      READ_REG_INVALID = (data & 0x02) >> 1;
      MEASMODE_INVALID = (data & 0x04) >> 2;
      MAX_RESISTANCE = (data & 0x08) >> 3;
      HEATER_FAULT = (data & 0x10) >> 4;
      HEATER_SUPPLY = (data & 0x20) >> 5;
    }
  };
  error_id _error_id;

  /*=========================================================================*/
};

#endif // HEADER_H