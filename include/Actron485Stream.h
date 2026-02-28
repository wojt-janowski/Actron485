#pragma once

#include <cstddef>
#include <cstdint>

#if __has_include(<Arduino.h>)
#include <Arduino.h>

namespace Actron485 {
using SerialStream = ::Stream;
using LogSink = ::Print;
}  // namespace Actron485

#else

#include <cmath>
#include <cstdio>
#include <cstring>

namespace Actron485 {

class LogSink {
 public:
  virtual ~LogSink() = default;
  virtual size_t write(uint8_t data) = 0;
  virtual size_t write(const uint8_t *data, size_t size) {
    size_t written = 0;
    for (size_t i = 0; i < size; i++) {
      written += this->write(data[i]);
    }
    return written;
  }

  size_t print(const char *text) {
    if (text == nullptr) {
      return this->print("(null)");
    }
    return this->write(reinterpret_cast<const uint8_t *>(text), std::strlen(text));
  }

  size_t print(char value) { return this->write(static_cast<uint8_t>(value)); }
  size_t print(unsigned char value) { return this->print(static_cast<unsigned long>(value)); }
  size_t print(int value) { return this->print(static_cast<long>(value)); }
  size_t print(unsigned int value) { return this->print(static_cast<unsigned long>(value)); }
  size_t print(long value) { return this->print_signed(static_cast<long long>(value)); }
  size_t print(unsigned long value) { return this->print_unsigned(static_cast<unsigned long long>(value)); }
  size_t print(long long value) { return this->print_signed(value); }
  size_t print(unsigned long long value) { return this->print_unsigned(value); }
  size_t print(bool value) { return this->print(value ? "1" : "0"); }

  size_t print(float value) { return this->print(static_cast<double>(value), 2); }
  size_t print(float value, int digits) { return this->print(static_cast<double>(value), digits); }
  size_t print(double value) { return this->print(value, 2); }
  size_t print(double value, int digits) {
    if (std::isnan(value)) {
      return this->print("nan");
    }
    if (std::isinf(value)) {
      return this->print(value < 0 ? "-inf" : "inf");
    }

    if (digits < 0) {
      digits = 0;
    } else if (digits > 9) {
      digits = 9;
    }

    char format[8];
    std::snprintf(format, sizeof(format), "%%.%df", digits);

    char buffer[64];
    int count = std::snprintf(buffer, sizeof(buffer), format, value);
    if (count <= 0) {
      return 0;
    }
    size_t length = static_cast<size_t>(count);
    if (length >= sizeof(buffer)) {
      length = sizeof(buffer) - 1;
    }
    return this->write(reinterpret_cast<const uint8_t *>(buffer), length);
  }

  size_t println() { return this->print("\n"); }
  size_t println(const char *text) {
    size_t written = this->print(text);
    written += this->println();
    return written;
  }

  template<typename T> size_t println(const T &value) {
    size_t written = this->print(value);
    written += this->println();
    return written;
  }

 protected:
  size_t print_signed(long long value) {
    char buffer[32];
    int count = std::snprintf(buffer, sizeof(buffer), "%lld", value);
    if (count <= 0) {
      return 0;
    }
    size_t length = static_cast<size_t>(count);
    if (length >= sizeof(buffer)) {
      length = sizeof(buffer) - 1;
    }
    return this->write(reinterpret_cast<const uint8_t *>(buffer), length);
  }

  size_t print_unsigned(unsigned long long value) {
    char buffer[32];
    int count = std::snprintf(buffer, sizeof(buffer), "%llu", value);
    if (count <= 0) {
      return 0;
    }
    size_t length = static_cast<size_t>(count);
    if (length >= sizeof(buffer)) {
      length = sizeof(buffer) - 1;
    }
    return this->write(reinterpret_cast<const uint8_t *>(buffer), length);
  }
};

class SerialStream : public LogSink {
 public:
  ~SerialStream() override = default;

  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() = 0;
};

}  // namespace Actron485

#endif
