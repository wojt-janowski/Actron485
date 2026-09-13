#include "Actron485.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
using namespace Actron485;
int64_t test_time_us = 1000000;
struct NullBus : SerialStream {
 int available() override { return 0; }
 int read() override { return -1; }
 int peek() override { return -1; }
 void flush() override {}
 size_t write(uint8_t) override { return 1; }
};
int main() {
 static NullBus bus;
 static Controller c(bus, 0);
 c.setSlaveResponderMode(3,true);
 for (auto &on : c.stateMessage2.zoneOn) on=false;
 c.stateMessage2.zoneOn[6]=true;
 c.setZoneSetpointTemperatureCustom(7,26,false);
 c.setZoneCurrentTemperature(7,22);
 c.setOperatingMode(OperatingMode::Heat);
 // Allow startup delay, keeping the sensor fresh.
 for(int i=0;i<240;++i) {
  test_time_us+=1000000;
  c.setZoneCurrentTemperature(7,22);
  c.loop();
 }
 if ((c.getSlaveRegister(2)&255)==0) {
  std::fputs("FAIL: fresh zone below Heat target never requests demand\n",stderr);
  return EXIT_FAILURE;
 }
 c.setSystemOn(false);
 if(c.getSlaveRegister(2)!=0 || c.getSlaveRegister(4)!=0x23) return EXIT_FAILURE;
 // Fresh data must come from sensor input, never an echoed bus temperature.
 c.setOperatingMode(OperatingMode::Cool);
 for(auto &on:c.stateMessage2.zoneOn) on=false;
 c.stateMessage2.zoneOn[6]=true;
 c.setZoneSetpointTemperatureCustom(7,20,false);
 for(int i=0;i<240;++i) {
  test_time_us+=1000000; c.setZoneCurrentTemperature(7,25); c.loop();
 }
 assert((c.getSlaveRegister(2)>>8)==2 && (c.getSlaveRegister(2)&255)>0);
 c.setZoneCurrentTemperature(7,20);
 assert((c.getSlaveRegister(2)&255)==0);
 c.setAutoComfortRange(20,22);
 c.setOperatingMode(OperatingMode::Auto);
 c.setFanSpeed(FanMode::Esp);
 c.stateMessage2.zoneOn[7]=false;
 c.stateMessage2.zoneOn[6]=true;
 c.setZoneSetpointTemperatureCustom(7,21,false);
 for(int i=0;i<240;++i) {
  test_time_us+=1000000; c.setZoneCurrentTemperature(7,24); c.loop();
 }
 assert(c.getOperatingMode()==OperatingMode::Auto);
 assert(c.getFanSpeed()==FanMode::Esp);
 assert((c.getSlaveRegister(2)>>8)==2 && (c.getSlaveRegister(2)&255)>0);
 assert((c.getSlaveRegister(11)>>8)==44); // upper target 22, not midpoint 21
 test_time_us+=121000000; c.loop();
 assert((c.getSlaveRegister(2)&255)==0);
 assert(!c.isZoneSensorFresh(7));
 assert(!c.setAutoComfortRange(24,23));
 assert(!c.setAutoComfortRange(NAN,23));
 assert(!c.setAutoComfortRange(20,20.5));
 assert(c.getAutoTargetLow()==20 && c.getAutoTargetHigh()==22);
 c.setZoneCurrentTemperature(7,22);
 c.zoneTemperature[6]=50; // simulate a passive-echo value left in display state
 assert(c.getZoneCurrentTemperature(7)==22);
 c.setSystemOn(false);
 assert(c.getSlaveRegister(2)==0 && c.getSlaveRegister(4)==0x23);
 c.setOperatingMode(OperatingMode::Heat);
 for(auto &on:c.stateMessage2.zoneOn) on=false;
 c.stateMessage2.zoneOn[6]=true;
 c.zoneSetpoint[6]=NAN;
 for(int i=0;i<240;++i) {
  test_time_us+=1000000; c.setZoneCurrentTemperature(7,10); c.loop();
 }
 assert((c.getSlaveRegister(2)&255)==0);
 c.setZoneSetpointTemperatureCustom(7,22,false);
 c.setZoneSetpointTemperatureCustom(7,NAN,false);
 assert(c.getZoneSetpointTemperature(7)==22);
 c.setZoneSetpointTemperature(7,24,false);
 assert(c.getZoneSetpointTemperature(7)==24);
 c.setControlZone(7,true);
 c.setAutoComfortRange(20,22);
 c.setOperatingMode(OperatingMode::Auto);
 c.setZoneSetpointTemperature(7,25,false);
 assert(c.getMasterSetpoint()==21);
 assert(c.getAutoTargetLow()==20 && c.getAutoTargetHigh()==22);
 assert(c.getZoneSetpointTemperature(7)==25);
 c.setMasterSetpoint(30);
 assert(c.getMasterSetpoint()==29);
 assert(c.getAutoTargetLow()==28 && c.getAutoTargetHigh()==30);
 std::puts("Thermal controller integration tests passed");
}
