#include "QueThermostat.h"
#include <cassert>
#include <cstdio>
using namespace Actron485;
int64_t test_time_us=0;
using Mode=OperatingMode;
std::array<QueThermostat::Zone,8> one(double temp,double lo=26,double hi=28) {
 std::array<QueThermostat::Zone,8> z{}; z[6]={true,true,temp,lo,hi}; return z;
}
int main() {
 QueThermostat t;
 t.reset(0);
 auto z=one(22);
 assert(t.step(179999,Mode::Heat,z).demand==0);
 assert(t.step(181000,Mode::Heat,z).branch==Mode::Heat);
 for(uint32_t n=182000;n<=210000;n+=1000) t.step(n,Mode::Heat,z);
 assert(t.output().demand==100);
 z[6].temperature=26;
 assert(t.step(211000,Mode::Heat,z).demand==0);
 z[6].temperature=25.6;
 assert(t.step(400000,Mode::Heat,z).demand==0);
 z[6].temperature=25.5;
 assert(t.step(401000,Mode::Heat,z).demand>0);
 z[6].fresh=false;
 assert(t.step(402000,Mode::Heat,z).demand==0);
 z[6].fresh=true;
 assert(t.step(403000,Mode::Heat,z).demand==0);
 t.reset(0); z=one(30,20,22);
 assert(t.step(180000,Mode::Cool,z).branch==Mode::Cool);
 for(uint32_t n=181000;n<=210000;n+=1000) t.step(n,Mode::Cool,z);
 assert(t.output().demand==72);
 z[6].temperature=22;
 assert(t.step(211000,Mode::Cool,z).demand==0);
 t.reset(0); z=one(22);
 assert(t.step(180000,Mode::Auto,z).branch==Mode::Heat);
 z[6].temperature=29;
 assert(t.step(181000,Mode::Auto,z).demand==0);
 assert(t.step(360999,Mode::Auto,z).demand==0);
 assert(t.step(362000,Mode::Auto,z).branch==Mode::Cool);
 z[6].temperature=27;
 assert(t.step(363000,Mode::Auto,z).demand==0);
 // Both exact comfort boundaries and the interior are standby from rest.
 for(double temp : {26.0,27.0,28.0}) {
  t.reset(0); assert(t.step(200000,Mode::Auto,one(temp)).demand==0);
 }
 // Opposing zones: choose largest error initially, retain current branch.
 t.reset(0); z=one(22); z[0]={true,true,32,26,28};
 assert(t.step(180000,Mode::Auto,z).branch==Mode::Heat);
 z[0].temperature=35;
 assert(t.step(181000,Mode::Auto,z).branch==Mode::Heat);
 z[0].fresh=false;
 assert(t.step(182000,Mode::Auto,z).demand==0);
 z[0].enabled=false;
 assert(t.step(400000,Mode::Auto,z).branch==Mode::Heat);
 // No valid reading, invalid bounds and no enabled zones cannot call.
 for(double invalid : {double(NAN), double(INFINITY), -1.0, 61.0}) {
  t.reset(0); assert(t.step(200000,Mode::Heat,one(invalid)).demand==0);
 }
 t.reset(0); assert(t.step(200000,Mode::Auto,one(20,28,26)).demand==0);
 t.reset(0); z=one(22); z[6].enabled=false;
 assert(t.step(200000,Mode::Heat,z).demand==0);
 // OFF and fan-only immediately stop thermal demand.
 for(Mode mode : {Mode::Off,Mode::FanOnly}) {
  t.reset(0); z=one(22); t.step(180000,Mode::Heat,z);
  assert(t.step(180001,mode,z).demand==0);
 }
 // Frequent callbacks must preserve fractional ramp progress.
 t.reset(0); z=one(22); t.step(180000,Mode::Heat,z);
 for(uint32_t n=180100;n<=182000;n+=100) t.step(n,Mode::Heat,z);
 assert(t.output().demand>=14);
 // millis rollover must not bypass or extend restart protection.
 uint32_t start=0xffff0000u; t.reset(start);
 assert(t.step(start+179999u,Mode::Heat,z).demand==0);
 assert(t.step(start+181000u,Mode::Heat,z).demand>0);
 std::puts("Thermostat mode, bounds, sensor, timing and rollover tests passed");
}
