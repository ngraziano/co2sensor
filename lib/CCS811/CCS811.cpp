#include "CCS811.h"
bool CCS811::begin(uint8_t addr) {
  _i2caddr = addr;
  _i2c_init();

  SWReset();
  delay(100);

  uint8_t id;
  _lastI2cError = read(Ccs811Register::hw_id, &id, 1);
  // check that the HW id is correct
  if (_lastI2cError != 0 || id != CCS811_HW_ID_CODE)
    return false;

  // try to start the app
  _lastI2cError = this->write(Ccs811Register::app_start, NULL, 0);
  delay(100);

  // make sure there are no errors and we have entered application mode
  if (checkError())
    return false;
  if (!_status.FW_MODE)
    return false;

  disableInterrupt();

  // default to read every second
  setDriveMode(Ccs811Drive::mode_1sec);

  return true;
}

uint8_t CCS811::setDriveMode(Ccs811Drive mode) {
  _meas_mode.DRIVE_MODE = static_cast<uint8_t>(mode);
  uint8_t value = _meas_mode.get();
  _lastI2cError = this->write(Ccs811Register::meas_mode, &value, 1);
  return _lastI2cError;
}

uint8_t CCS811::enableInterrupt() {
  _meas_mode.INT_DATARDY = 1;
  uint8_t value = _meas_mode.get();
  _lastI2cError = this->write(Ccs811Register::meas_mode, &value, 1);
  return _lastI2cError;
}

uint8_t CCS811::disableInterrupt() {
  _meas_mode.INT_DATARDY = 0;
  uint8_t value = _meas_mode.get();
  _lastI2cError = this->write(Ccs811Register::meas_mode, &value, 1);
  return _lastI2cError;
}

bool CCS811::available() {
  uint8_t status;
  _lastI2cError = read(Ccs811Register::status, &status, 1);
  if (_lastI2cError == 0) {
    _status.set(status);
    if (!_status.DATA_READY)
      return false;
    else
      return true;
  }
  return false;
}

bool CCS811::readData() {
  // eCO2 H, eCO2 L, TVOC H, TVOC, L, STATUS, ERROR_ID, RAW1, RAW2
  uint8_t buf[8];
  // read data and status in one read.
  _lastI2cError = this->read(Ccs811Register::alg_result_data, buf, 8);

  if (_lastI2cError != 0)
    return false;

  _status.set(buf[4]);

  if (!_status.DATA_READY || _status.ERROR)
    return false;

  _eCO2 = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
  _TVOC = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);
  return true;
}

uint8_t CCS811::setEnvironmentalData(uint8_t humidity, double temperature) {
  /* Humidity is stored as an unsigned 16 bits in 1/512%RH. The
  default value is 50% = 0x64, 0x00. As an example 48.5%
  humidity would be 0x61, 0x00.*/

  /* Temperature is stored as an unsigned 16 bits integer in 1/512
  degrees; there is an offset: 0 maps to -25°C. The default value is
  25°C = 0x64, 0x00. As an example 23.5% temperature would be
  0x61, 0x00.
  The internal algorithm uses these values (or default values if
  not set by the application) to compensate for changes in
  relative humidity and ambient temperature.*/

  uint8_t hum_perc = humidity << 1;

  float fractional = modf(temperature, &temperature);
  uint16_t temp_high = (((uint16_t)temperature + 25) << 9);
  uint16_t temp_low = ((uint16_t)(fractional / 0.001953125) & 0x1FF);

  uint16_t temp_conv = (temp_high | temp_low);

  uint8_t buf[] = {hum_perc, 0x00, (uint8_t)((temp_conv >> 8) & 0xFF),
                   (uint8_t)(temp_conv & 0xFF)};

  _lastI2cError = this->write(Ccs811Register::env_data, buf, 4);
  return _lastI2cError;
}

// calculate temperature based on the NTC register
double CCS811::calculateTemperature() {
  uint8_t buf[4];
  _lastI2cError = this->read(Ccs811Register::ntc, buf, 4);
  if (_lastI2cError != 0) {
    return NAN;
  }

  uint32_t vref = ((uint32_t)buf[0] << 8) | buf[1];
  uint32_t vntc = ((uint32_t)buf[2] << 8) | buf[3];

  // from ams ccs811 app note
  uint32_t rntc = vntc * CCS811_REF_RESISTOR / vref;

  double ntc_temp;
  ntc_temp = log((double)rntc / CCS811_REF_RESISTOR); // 1
  ntc_temp /= 3380;                                   // 2
  ntc_temp += 1.0 / (25 + 273.15);                    // 3
  ntc_temp = 1.0 / ntc_temp;                          // 4
  ntc_temp -= 273.15;                                 // 5
  return ntc_temp - _tempOffset;
}

uint8_t CCS811::setThresholds(uint16_t low_med, uint16_t med_high,
                              uint8_t hysteresis) {
  uint8_t buf[] = {(uint8_t)((low_med >> 8) & 0xF), (uint8_t)(low_med & 0xF),
                   (uint8_t)((med_high >> 8) & 0xF), (uint8_t)(med_high & 0xF),
                   hysteresis};

  _lastI2cError = this->write(Ccs811Register::thresholds, buf, 5);
  return _lastI2cError;
}

uint8_t CCS811::SWReset() {
  // reset sequence from the datasheet
  uint8_t seq[] = {0x11, 0xE5, 0x72, 0x8A};
  _lastI2cError = this->write(Ccs811Register::sw_reset, seq, 4);
  return _lastI2cError;
}

bool CCS811::checkError() {
  uint8_t status;
  _lastI2cError = read(Ccs811Register::status, &status, 1);
  if (_lastI2cError == 0) {
    _status.set(status);
    return _status.ERROR;
  }
  return true;
}

void CCS811::_i2c_init() { Wire.begin(); }

uint8_t CCS811::read(Ccs811Register reg, uint8_t *buf, uint8_t num) {
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    Wire.requestFrom(_i2caddr, num);
    for (int i = 0; i < num && Wire.available(); i++) {
      buf[i] = Wire.read();
    }
  }
  return result;
}

uint8_t CCS811::write(Ccs811Register reg, uint8_t *buf, uint8_t num) {
  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t *)buf, num);
  return Wire.endTransmission();
}