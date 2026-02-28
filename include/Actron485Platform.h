#pragma once

#include <cstdint>
#include "Actron485Stream.h"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define ACTRON485_HAS_ARDUINO 1
#else
#define ACTRON485_HAS_ARDUINO 0
#include <driver/gpio.h>
#include <esp_timer.h>
#endif

namespace Actron485 {

inline unsigned long platformMillis() {
#if ACTRON485_HAS_ARDUINO
  return millis();
#else
  return static_cast<unsigned long>(esp_timer_get_time() / 1000ULL);
#endif
}

inline void platformPinModeOutput(uint8_t pin) {
#if ACTRON485_HAS_ARDUINO
  pinMode(pin, OUTPUT);
#else
  gpio_reset_pin(static_cast<gpio_num_t>(pin));
  gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT);
#endif
}

inline void platformDigitalWrite(uint8_t pin, bool high) {
#if ACTRON485_HAS_ARDUINO
  digitalWrite(pin, high ? HIGH : LOW);
#else
  gpio_set_level(static_cast<gpio_num_t>(pin), high ? 1 : 0);
#endif
}

inline SerialStream *platformInitDefaultSerial(uint8_t rxPin, uint8_t txPin) {
#if ACTRON485_HAS_ARDUINO
  Serial1.begin(4800, SERIAL_8N1, rxPin, txPin);
  return &Serial1;
#else
  (void) rxPin;
  (void) txPin;
  return nullptr;
#endif
}

inline void platformSwitchSharedUartToTx(uint8_t rxPin, uint8_t txPin) {
#if ACTRON485_HAS_ARDUINO
  if (rxPin != txPin) {
    return;
  }
  pinMatrixOutDetach(rxPin, false, false);
  pinMode(txPin, OUTPUT);
  pinMatrixOutAttach(txPin, U1TXD_OUT_IDX, false, false);
#else
  (void) rxPin;
  (void) txPin;
#endif
}

inline void platformSwitchSharedUartToRx(uint8_t rxPin, uint8_t txPin) {
#if ACTRON485_HAS_ARDUINO
  if (rxPin != txPin) {
    return;
  }
  pinMatrixOutDetach(txPin, false, false);
  pinMode(rxPin, INPUT);
  pinMatrixOutAttach(rxPin, U1RXD_IN_IDX, false, false);
#else
  (void) rxPin;
  (void) txPin;
#endif
}

}  // namespace Actron485
