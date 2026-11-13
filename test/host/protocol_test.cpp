#include "Actron485.h"
#include <deque>
#include <vector>
#include <cstdio>
#include <cstdlib>
using namespace Actron485;
int64_t test_time_us = 1000000;
struct Bus : SerialStream {
    std::deque<uint8_t> incoming;
    std::vector<uint8_t> outgoing;
    int available() override { return incoming.size(); }
    int peek() override { return incoming.empty() ? -1 : incoming.front(); }
    int read() override { int v = peek(); if (v >= 0) incoming.pop_front(); return v; }
    void flush() override {}
    size_t write(uint8_t b) override { outgoing.push_back(b); return 1; }
};
int failures = 0;
void check(bool ok, const char *description) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", description); ++failures; }
}
std::vector<uint8_t> withCrc(std::vector<uint8_t> frame) {
    uint16_t crc = 0xffff;
    for (auto b : frame) {
        crc ^= b;
        for (int i = 0; i < 8; ++i) crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
    frame.push_back(crc & 255); frame.push_back(crc >> 8);
    return frame;
}
void feed(Controller &c, Bus &bus, std::vector<uint8_t> frame) {
    frame = withCrc(frame);
    bus.incoming.insert(bus.incoming.end(), frame.begin(), frame.end());
    c.loop(); test_time_us += 40000; c.loop();
}
int main() {
    // Static storage matches the zero-initialised embedded controller instance.
    static Bus bus;
    static Controller c(bus, 0);
    c.setSlaveResponderMode(3, true);
    c.stateMessage2.initialised = true;
    for (auto &on : c.stateMessage2.zoneOn) on = false;
    c.stateMessage2.zoneOn[6] = true;
    c.setOperatingMode(OperatingMode::FanOnly);
    const FanMode speeds[] = {FanMode::Low, FanMode::Medium, FanMode::High, FanMode::Esp};
    const uint16_t captured[] = {0x0928, 0x113d, 0x2159, 0x0300};
    for (int i = 0; i < 4; ++i) {
        c.setFanSpeed(speeds[i]);
        check(c.getSlaveRegister(2) == 0x0800, "fan-only mode matches capture");
        check(c.getSlaveRegister(3) == captured[i], "fan command matches original controller capture");
        check(c.getSlaveRegister(4) == 0x4000, "fan-only zone 7 bitmap has zero low byte");
    }
    // Continuous fan sets reg 3 hi bit 2 (capture: 0x09 -> 0x0D) and must clear again,
    // including after a speed change while continuous.
    c.setFanSpeed(FanMode::Low);
    c.setContinuousFanMode(true);
    check(c.getContinuousFanMode() && c.getSlaveRegister(3) == 0x0d28, "continuous fan sets flag");
    c.setFanSpeed(FanMode::Medium);
    check(c.getContinuousFanMode() && c.getSlaveRegister(3) == 0x153d, "speed change keeps continuous");
    c.setContinuousFanMode(false);
    check(!c.getContinuousFanMode() && c.getSlaveRegister(3) == 0x113d, "continuous fan turns off");
    check(c.getFanSpeed() == FanMode::Medium, "turning continuous off keeps the speed");
    c.setOperatingMode(OperatingMode::Heat);
    check(c.getSlaveRegister(4) == 0x4023, "thermal zone encoding retained");
    c.setSystemOn(false);
    check(c.getSlaveRegister(2) == 0, "off mode cleared");
    check(c.getSlaveRegister(4) == 0x0023, "off clears all zones");
    c.setAutoComfortRange(20,22);
    c.setOperatingMode(OperatingMode::Auto);
    std::vector<uint8_t> echo={11,16,0,2,0,10,20};
    echo.resize(27,0); echo[7]=2; echo[9]=0x40; echo[10]=0x23;
    feed(c,bus,echo);
    check(c.getOperatingMode()==OperatingMode::Auto, "downstream thermal branch cannot overwrite selected Auto");
    check(c.getAutoTargetLow()==20 && c.getAutoTargetHigh()==22, "echo preserves comfort range");
    c.setSystemOn(false);
    c.setSlaveResponderMode(3, false);
    bus.outgoing.clear();
    struct Capture { uint16_t mode, fan; OperatingMode decoded; CompressorMode activity; };
    const Capture cases[] = {
        {0x0243, 0x0300, OperatingMode::Cool, CompressorMode::Cooling},
        {0x0248, 0x0300, OperatingMode::Cool, CompressorMode::Cooling},
        {0x0200, 0x0100, OperatingMode::Cool, CompressorMode::Idle},
        {0x0223, 0x0300, OperatingMode::Cool, CompressorMode::Cooling},
        {0x0164, 0x2159, OperatingMode::Heat, CompressorMode::Heating},
        {0x0100, 0x0100, OperatingMode::Heat, CompressorMode::Idle},
        {0x0800, 0x0928, OperatingMode::FanOnly, CompressorMode::Idle},
        {0x0800, 0x113d, OperatingMode::FanOnly, CompressorMode::Idle},
        {0x0800, 0x2159, OperatingMode::FanOnly, CompressorMode::Idle},
        {0x0800, 0x0300, OperatingMode::FanOnly, CompressorMode::Idle},
    };
    for (const auto &v : cases) {
        feed(c, bus, {3,3,0,2,0,2});
        feed(c, bus, {3,3,4,uint8_t(v.mode >> 8),uint8_t(v.mode),uint8_t(v.fan >> 8),uint8_t(v.fan)});
        check(c.stateMessage2.operatingMode == v.decoded, "decode active thermal branch, not inferred Auto setting");
        check(c.stateMessage2.compressorMode == v.activity, "decode demand from reg 2 low byte");
        check(c.stateMessage2.fanActive == ((v.fan & 0x3a00) != 0), "all observed fan flags recognised");
    }
    // Slave 1 (outdoor unit) replies start with 0x01, the legacy IndoorBoard1
    // type byte. A 41-byte reply takes ~85 ms at 4800 baud, so it reaches the
    // parser over several loop() passes that are each more than the 5 ms break
    // apart. It must still be framed as one Modbus response.
    feed(c, bus, {1,3,0,120,0,18});
    std::vector<uint8_t> reply{1,3,36};
    for (int i = 0; i < 18; ++i) { reply.push_back(0); reply.push_back(0); }
    reply[3 + 10*2] = 0x41; reply[3 + 10*2 + 1] = 0xc4;  // reg 130/131 = 24.5f
    reply = withCrc(reply);
    for (size_t at = 0; at < reply.size(); at += 16) {
        bus.incoming.insert(bus.incoming.end(), reply.begin() + at, reply.begin() + std::min(at + 16, reply.size()));
        c.loop(); test_time_us += 16000;
    }
    c.loop(); test_time_us += 40000; c.loop();
    check(c.getOutdoorTemperature() == 24.5, "outdoor-unit reply split across loop passes is decoded");
    check(c.getOutdoorUnitRequests() == 1 && c.getOutdoorUnitReplies() == 1, "outdoor-unit requests and replies counted");
    check(bus.outgoing.empty(), "passive capture never transmits");
    if (failures) return EXIT_FAILURE;
    std::puts("All captured protocol regression tests passed");
}
