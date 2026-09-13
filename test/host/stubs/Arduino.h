#pragma once
#include <cstddef>
#include <cstdint>
#include "esp_timer.h"
constexpr int OUTPUT=1, INPUT=0, HIGH=1, LOW=0, SERIAL_8N1=0;
constexpr int U1TXD_OUT_IDX=0, U1RXD_IN_IDX=0;
inline unsigned long millis() { return esp_timer_get_time()/1000; }
inline void pinMode(int,int) {}
inline void digitalWrite(int,int) {}
inline void pinMatrixOutDetach(int,bool,bool) {}
inline void pinMatrixOutAttach(int,int,bool,bool) {}
class Print {
 public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t)=0;
    virtual size_t write(const uint8_t *data, size_t size) {
        for (size_t i=0;i<size;++i) write(data[i]); return size;
    }
    template<class T> size_t print(T) { return 0; }
    template<class T> size_t print(T,int) { return 0; }
    template<class T> size_t println(T) { return 0; }
    size_t println() { return 0; }
};
class Stream : public Print {
 public:
    virtual int available()=0;
    virtual int read()=0;
    virtual int peek()=0;
    virtual void flush()=0;
};
struct UnusedHardwareSerial : Stream {
    void begin(int,int,int,int) {}
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override {}
    size_t write(uint8_t) override { return 1; }
};
inline UnusedHardwareSerial Serial1;
