
#include "i2c_util.h"

#include <Wire.h>

#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf(__VA_ARGS__)
#define DEBUG_MSG_PP(msg, ...) DEBUG_ESP_PORT.printf_P(PSTR(msg), ##__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#define DEBUG_MSG_PP(msg, ...)
#endif

namespace i2c_util {
void i2crecovery(uint8_t sdaPin, uint8_t sclPin) {

  pinMode(sdaPin, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins
                                 // Inputs with pullup.
  pinMode(sclPin, INPUT_PULLUP);
  delay(2500);
  boolean slc_low = (digitalRead(sclPin) == LOW); // Check is SCL is Low.
  if (slc_low) { // If it is held low Arduno cannot become the I2C master.
    DEBUG_MSG_PP("SCL held low, can not recover\n");
    return; // I2C bus error. Could not clear SCL clock line held low
  }

  boolean sda_low = (digitalRead(sdaPin) == LOW); // vi. Check SDA input.
  int clockCount = 20;                            // > 2x9 clock

  while (sda_low && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(sclPin, INPUT);  // release SCL pullup so that when made output
                             // it will be LOW
    pinMode(sclPin, OUTPUT); // then clock SCL Low
    delayMicroseconds(10);   //  for >5uS
    pinMode(sclPin, INPUT);  // release SCL LOW
    pinMode(slc_low, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock
    // stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    slc_low = (digitalRead(sclPin) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (
        slc_low &&
        (counter > 0)) { //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      slc_low = (digitalRead(sclPin) == LOW);
    }
    if (slc_low) { // still low after 2 sec error
      DEBUG_MSG_PP("SCL held low, can not recover by slave clock stretch "
                 "for >2sec\n",0);
      return; // I2C bus error. Could not clear. SCL clock line held low
              // by slave clock stretch for >2sec
    }
    sda_low =
        (digitalRead(sdaPin) == LOW); //   and check SDA input again and loop
  }
  if (sda_low) { // still low
    DEBUG_MSG_PP("Could not clear. SDA data line held low\n");
    return; // I2C bus error. Could not clear. SDA data line held low
  }
  // else pull SDA line low for Start or Repeated Start
  pinMode(sdaPin, INPUT);  // remove pullup.
  pinMode(sdaPin, OUTPUT); // and then make it LOW i.e. send an I2C Start or
                           // Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same
  // function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening
  /// Stop.
  delayMicroseconds(10);  // wait >5uS
  pinMode(sdaPin, INPUT); // remove output low
  pinMode(sdaPin,
          INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS

  DEBUG_MSG_PP("bus recovery done, restart communication in 2s");
  // return to power up mode
  pinMode(sdaPin, INPUT);
  pinMode(sclPin, INPUT);
  delay(2000);
  Wire.begin(sdaPin, sclPin);
}
} // namespace i2c_util