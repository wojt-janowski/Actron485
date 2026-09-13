#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include "Actron485Models.h"

namespace Actron485 {
// Experimental replacement policy, NOT a recovered Actron control law.
// Protocol captures establish example values, not that the byte is percent.
class QueThermostat {
 public:
  struct Zone {
    bool enabled{false};
    bool fresh{false};
    double temperature{NAN};
    double heat_target{22};
    double cool_target{22};
  };
  struct Output {
    OperatingMode branch{OperatingMode::Off};
    uint8_t demand{0};
    const char *reason{"off"};
  };
  static constexpr uint32_t MIN_OFF_MS = 180000;
  static constexpr double START_ERROR = 0.5;

  void reset(uint32_t now) {
    output_ = {};
    demand_ = 0;
    stopped_at_ = last_step_ = now;
    initialized_ = true;
  }
  const Output &output() const { return output_; }

  Output step(uint32_t now, OperatingMode selected, const std::array<Zone,8> &zones) {
    if (!initialized_) reset(now);
    const uint32_t dt = now - last_step_;
    last_step_ = now;
    if (selected != OperatingMode::Heat && selected != OperatingMode::Cool &&
        selected != OperatingMode::Auto) return stop(now, "off");
    double heat = -1000, cool = -1000;
    bool enabled = false;
    for (const auto &z : zones) {
      if (!z.enabled) continue;
      enabled = true;
      if (!z.fresh || !std::isfinite(z.temperature) || z.temperature < 0 ||
          z.temperature > 60 || !std::isfinite(z.heat_target) ||
          !std::isfinite(z.cool_target) || z.heat_target < 16 ||
          z.cool_target > 30 || z.heat_target > z.cool_target)
        return stop(now, "sensor_or_target_invalid");
      heat = std::max(heat, z.heat_target-z.temperature);
      cool = std::max(cool, z.temperature-z.cool_target);
    }
    if (!enabled) return stop(now, "no_enabled_zones");
    OperatingMode desired = OperatingMode::Off;
    // Retain an active Auto branch until its enabled zones satisfy it.
    if (selected == OperatingMode::Heat || selected == OperatingMode::Cool) {
      const double error = selected == OperatingMode::Heat ? heat : cool;
      if (error >= START_ERROR || (output_.branch == selected && error > 0)) desired = selected;
    } else if (output_.branch == OperatingMode::Heat && heat > 0) desired=OperatingMode::Heat;
    else if (output_.branch == OperatingMode::Cool && cool > 0) desired=OperatingMode::Cool;
    else if (heat >= START_ERROR && heat >= cool) desired=OperatingMode::Heat;
    else if (cool >= START_ERROR) desired=OperatingMode::Cool;
    if (desired == OperatingMode::Off) return stop(now, "at_target");
    if (output_.branch != OperatingMode::Off && output_.branch != desired)
      return stop(now, "changeover_delay");
    if (output_.branch == OperatingMode::Off && uint32_t(now-stopped_at_) < MIN_OFF_MS) {
      output_.reason="restart_delay";
      return output_;
    }
    const double error = desired == OperatingMode::Heat ? heat : cool;
    // Initial tuning: 25 demand units per degree, floor 20, captured upper
    // limits Heat=100 and Cool=72. No integral wind-up or maximum replay.
    const double cap = desired == OperatingMode::Heat ? 100.0 : 72.0;
    const double target = std::clamp(error*25.0,20.0,cap);
    // Slew upward by at most 5 units/s. Downward corrections are immediate.
    const double next = std::min(target, demand_+std::min<uint32_t>(dt,1000)*0.005);
    output_.branch=desired;
    demand_=next;
    output_.demand=uint8_t(next);
    output_.reason="calling";
    return output_;
  }
 private:
  Output stop(uint32_t now, const char *reason) {
    if (output_.branch != OperatingMode::Off) stopped_at_=now;
    demand_=0;
    output_={OperatingMode::Off,0,reason};
    return output_;
  }
  Output output_{};
  double demand_{0};
  uint32_t stopped_at_{0},last_step_{0};
  bool initialized_{false};
};
}
