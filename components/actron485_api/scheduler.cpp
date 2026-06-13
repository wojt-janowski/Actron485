#include "scheduler.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <ArduinoJson.h>

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/time/real_time_clock.h"

#include "actron485_api.h"

namespace esphome {
namespace actron485_api {

static const char *const TAG = "actron485.sched";

// Bumped only if the on-flash layout changes incompatibly.
static constexpr uint32_t kSchedMagic = 0xAC773CED;
static constexpr uint32_t kSchedVersion = 1;
// Re-evaluate at most this often. Calendar rules use a catch-up window so the
// coarse tick never drops an occurrence; weather-gated rules just re-test.
static constexpr uint32_t kEvalIntervalMs = 30000;

// ---- enum <-> string helpers -------------------------------------------------

static const char *mode_to_str(ActMode m) {
  switch (m) {
    case ActMode::Off: return "off";
    case ActMode::Heat: return "heat";
    case ActMode::Cool: return "cool";
    case ActMode::Auto: return "auto";
    case ActMode::FanOnly: return "fan_only";
    default: return nullptr;
  }
}
static ActMode str_to_mode(const char *s) {
  if (!s) return ActMode::None;
  if (!strcmp(s, "off")) return ActMode::Off;
  if (!strcmp(s, "heat")) return ActMode::Heat;
  if (!strcmp(s, "cool")) return ActMode::Cool;
  if (!strcmp(s, "auto")) return ActMode::Auto;
  if (!strcmp(s, "fan_only")) return ActMode::FanOnly;
  return ActMode::None;
}
static const char *fan_to_str(ActFan f) {
  switch (f) {
    case ActFan::Auto: return "auto";
    case ActFan::Low: return "low";
    case ActFan::Medium: return "medium";
    case ActFan::High: return "high";
    default: return nullptr;
  }
}
static ActFan str_to_fan(const char *s) {
  if (!s) return ActFan::None;
  if (!strcmp(s, "auto")) return ActFan::Auto;
  if (!strcmp(s, "low")) return ActFan::Low;
  if (!strcmp(s, "medium")) return ActFan::Medium;
  if (!strcmp(s, "high")) return ActFan::High;
  return ActFan::None;
}
static const char *source_to_str(CondSource s) {
  switch (s) {
    case CondSource::OutdoorTemp: return "outdoor_temp";
    case CondSource::ZoneTemp: return "zone_temp";
    case CondSource::Humidity: return "humidity";
    case CondSource::ForecastTodayMax: return "forecast_today_max";
    case CondSource::ForecastTodayMin: return "forecast_today_min";
    case CondSource::ForecastTodayPop: return "forecast_today_pop";
    default: return "outdoor_temp";
  }
}
static bool str_to_source(const char *s, CondSource &out) {
  if (!s) return false;
  if (!strcmp(s, "outdoor_temp")) { out = CondSource::OutdoorTemp; return true; }
  if (!strcmp(s, "zone_temp")) { out = CondSource::ZoneTemp; return true; }
  if (!strcmp(s, "humidity")) { out = CondSource::Humidity; return true; }
  if (!strcmp(s, "forecast_today_max")) { out = CondSource::ForecastTodayMax; return true; }
  if (!strcmp(s, "forecast_today_min")) { out = CondSource::ForecastTodayMin; return true; }
  if (!strcmp(s, "forecast_today_pop")) { out = CondSource::ForecastTodayPop; return true; }
  return false;
}
static const char *op_to_str(CondOp o) {
  switch (o) {
    case CondOp::Lt: return "lt";
    case CondOp::Gt: return "gt";
    case CondOp::Between: return "between";
    default: return "lt";
  }
}
static bool str_to_op(const char *s, CondOp &out) {
  if (!s) return false;
  if (!strcmp(s, "lt")) { out = CondOp::Lt; return true; }
  if (!strcmp(s, "gt")) { out = CondOp::Gt; return true; }
  if (!strcmp(s, "between")) { out = CondOp::Between; return true; }
  return false;
}

// days_mask bit0=Sun .. bit6=Sat
static const char *const kDayNames[7] = {"sun", "mon", "tue", "wed", "thu", "fri", "sat"};
static int day_name_to_index(const char *s) {
  for (int i = 0; i < 7; i++) {
    if (s && !strcmp(s, kDayNames[i])) return i;
  }
  return -1;
}

// Sunrise/sunset as a LOCAL minute-of-day for the date in `t`, via the NOAA
// solar equation (accurate to ~1 min). Self-contained — no weather provider.
// `t.timestamp` (UTC) vs the local wall fields gives the TZ offset, so DST is
// handled for free. Returns false if the sun doesn't rise/set that day (polar).
static bool compute_solar_local_min(const ESPTime &t, float lat_deg, float lon_deg,
                                    SolarEvent event, int offset_min, int &out_min) {
  // NB: do not name a local `PI` — Arduino.h #defines PI as a macro.
  const double kPi = 3.14159265358979323846;
  const double kDeg = kPi / 180.0;
  double gamma = 2.0 * kPi / 365.0 * (double) (t.day_of_year - 1);
  double eqtime = 229.18 * (0.000075 + 0.001868 * cos(gamma) - 0.032077 * sin(gamma)
                            - 0.014615 * cos(2 * gamma) - 0.040849 * sin(2 * gamma));
  double decl = 0.006918 - 0.399912 * cos(gamma) + 0.070257 * sin(gamma)
                - 0.006758 * cos(2 * gamma) + 0.000907 * sin(2 * gamma)
                - 0.002697 * cos(3 * gamma) + 0.00148 * sin(3 * gamma);
  double lat = lat_deg * kDeg;
  double zenith = 90.833 * kDeg;  // geometric sunrise/sunset incl. refraction
  double cosH = (cos(zenith) - sin(lat) * sin(decl)) / (cos(lat) * cos(decl));
  if (cosH > 1.0 || cosH < -1.0) return false;  // polar day/night — no event
  double ha = acos(cosH) / kDeg;                 // hour angle, degrees
  double noon_utc = 720.0 - 4.0 * lon_deg - eqtime;  // minutes from UTC midnight
  double ev_utc = (event == SolarEvent::Sunrise) ? (noon_utc - 4.0 * ha)
                                                 : (noon_utc + 4.0 * ha);

  // Local-UTC offset (minutes) inferred from this ESPTime.
  long utc_sod = (long) (t.timestamp % 86400);
  long loc_sod = (long) t.hour * 3600 + (long) t.minute * 60 + (long) t.second;
  long off = (loc_sod - utc_sod) / 60;
  while (off <= -720) off += 1440;
  while (off > 840) off -= 1440;

  long m = (long) llround(ev_utc) + off + offset_min;
  m %= 1440;
  if (m < 0) m += 1440;
  out_min = (int) m;
  return true;
}

// ---- lifecycle ---------------------------------------------------------------

void Scheduler::setup() {
  pref_ = global_preferences->make_preference<SchedulesBlob>(kSchedMagic);
  if (pref_.load(&blob_) && blob_.magic == kSchedMagic && blob_.version == kSchedVersion) {
    if (blob_.count > SCHED_MAX) blob_.count = SCHED_MAX;
    loaded_ = true;
    ESP_LOGCONFIG(TAG, "Loaded %u schedule(s) from NVS (away=%u)", blob_.count, blob_.away_active);
  } else {
    memset(&blob_, 0, sizeof(blob_));
    blob_.magic = kSchedMagic;
    blob_.version = kSchedVersion;
    blob_.next_id = 1;
    blob_.count = 0;
    loaded_ = true;
    ESP_LOGCONFIG(TAG, "No saved schedules; starting empty");
  }
}

void Scheduler::save_() {
  blob_.magic = kSchedMagic;
  blob_.version = kSchedVersion;
  pref_.save(&blob_);
}

Schedule *Scheduler::find_(uint16_t id) {
  int i = index_of_(id);
  return i < 0 ? nullptr : &blob_.records[i];
}
int Scheduler::index_of_(uint16_t id) {
  for (int i = 0; i < blob_.count; i++) {
    if (blob_.records[i].id == id) return i;
  }
  return -1;
}

// ---- evaluation --------------------------------------------------------------

uint32_t Scheduler::occurrence_key_(uint32_t local_day, uint16_t minute) const {
  return local_day * 1440u + minute;
}
bool Scheduler::fired_already_(uint16_t id, uint32_t occurrence) {
  for (auto &m : fired_) {
    if (m.id == id) return m.occurrence == occurrence;
  }
  return false;
}
void Scheduler::mark_fired_(uint16_t id, uint32_t occurrence) {
  for (auto &m : fired_) {
    if (m.id == id) { m.occurrence = occurrence; return; }
  }
  fired_.push_back({id, occurrence});
}

bool Scheduler::condition_value_(const Schedule &s, const Condition &c, float &live) {
  switch (c.source) {
    case CondSource::OutdoorTemp:
      return parent_->weather_current_temp(live);
    case CondSource::ZoneTemp: {
      if (c.zone < 1 || c.zone > 8) return false;
      double t = parent_->controller()->getZoneCurrentTemperature(c.zone);
      if (std::isnan(t) || t <= 0.0) return false;  // 0/NaN = no reading
      live = (float) t;
      return true;
    }
    case CondSource::Humidity: {
      if (c.zone < 1 || c.zone > 8) return false;
      float h = parent_->zone_humidity(c.zone);
      if (std::isnan(h)) return false;
      live = h;
      return true;
    }
    case CondSource::ForecastTodayMax:
    case CondSource::ForecastTodayMin:
    case CondSource::ForecastTodayPop: {
      float mn, mx; int pop;
      if (!parent_->forecast_today(mn, mx, pop)) return false;
      if (c.source == CondSource::ForecastTodayMax) { if (std::isnan(mx)) return false; live = mx; }
      else if (c.source == CondSource::ForecastTodayMin) { if (std::isnan(mn)) return false; live = mn; }
      else { live = (float) pop; }
      return true;
    }
  }
  return false;
}

bool Scheduler::conditions_pass_(const Schedule &s) {
  for (int i = 0; i < s.condition_count && i < SCHED_MAX_CONDITIONS; i++) {
    const Condition &c = s.conditions[i];
    float live;
    if (!condition_value_(s, c, live)) return false;  // fail-safe: missing input
    switch (c.op) {
      case CondOp::Lt: if (!(live < c.value1)) return false; break;
      case CondOp::Gt: if (!(live > c.value1)) return false; break;
      case CondOp::Between: if (!(live >= c.value1 && live <= c.value2)) return false; break;
    }
  }
  return true;
}

void Scheduler::apply_action_(const Action &a) {
  if (a.power != TriBool::Unset) parent_->apply_system_on(a.power == TriBool::True);
  if (a.mode != ActMode::None) {
    Actron485::OperatingMode m = Actron485::OperatingMode::Off;
    switch (a.mode) {
      case ActMode::Off: m = Actron485::OperatingMode::Off; break;
      case ActMode::Heat: m = Actron485::OperatingMode::Heat; break;
      case ActMode::Cool: m = Actron485::OperatingMode::Cool; break;
      case ActMode::Auto: m = Actron485::OperatingMode::Auto; break;
      case ActMode::FanOnly: m = Actron485::OperatingMode::FanOnly; break;
      default: break;
    }
    parent_->apply_operating_mode(m);
  }
  if (a.fan != ActFan::None) {
    Actron485::FanMode f = Actron485::FanMode::Esp;
    switch (a.fan) {
      case ActFan::Auto: f = Actron485::FanMode::Esp; break;
      case ActFan::Low: f = Actron485::FanMode::Low; break;
      case ActFan::Medium: f = Actron485::FanMode::Medium; break;
      case ActFan::High: f = Actron485::FanMode::High; break;
      default: break;
    }
    parent_->apply_fan_speed(f);
  }
  if (a.continuous != TriBool::Unset) parent_->apply_continuous_fan(a.continuous == TriBool::True);
  if (a.quiet != TriBool::Unset) parent_->apply_quiet_mode(a.quiet == TriBool::True);
  if (!std::isnan(a.setpoint)) parent_->apply_master_setpoint(a.setpoint);
  for (int z = 0; z < 8; z++) {
    if (a.zone_on[z] != TriBool::Unset) parent_->apply_zone_on(z + 1, a.zone_on[z] == TriBool::True);
    if (!std::isnan(a.zone_setpoint[z])) parent_->apply_zone_setpoint(z + 1, a.zone_setpoint[z]);
  }
}

void Scheduler::snapshot_expected_(const Action &a) {
  expected_ = Expected{};
  if (a.power != TriBool::Unset) expected_.power = (a.power == TriBool::True) ? 1 : 0;
  if (a.mode != ActMode::None) expected_.mode = (int) a.mode;
  if (!std::isnan(a.setpoint)) expected_.setpoint = a.setpoint;
  for (int z = 0; z < 8; z++) {
    if (a.zone_on[z] != TriBool::Unset) expected_.zone_on[z] = (a.zone_on[z] == TriBool::True) ? 1 : 0;
    if (!std::isnan(a.zone_setpoint[z])) expected_.zone_setpoint[z] = a.zone_setpoint[z];
  }
}

// Returns true if live state has diverged from what the active schedule set.
bool Scheduler::detect_override_() {
  if (active_id_ == 0) return false;
  auto *ctrl = parent_->controller();
  if (ctrl == nullptr) return false;
  const float kTol = 0.25f;

  if (expected_.power >= 0) {
    if ((int) ctrl->getSystemOn() != expected_.power) return true;
  }
  if (expected_.mode >= 0) {
    // Map live OperatingMode back to ActMode for comparison.
    int live = (int) ActMode::None;
    switch (ctrl->getOperatingMode()) {
      case Actron485::OperatingMode::Off: live = (int) ActMode::Off; break;
      case Actron485::OperatingMode::Heat: live = (int) ActMode::Heat; break;
      case Actron485::OperatingMode::Cool: live = (int) ActMode::Cool; break;
      case Actron485::OperatingMode::Auto: live = (int) ActMode::Auto; break;
      case Actron485::OperatingMode::FanOnly: live = (int) ActMode::FanOnly; break;
      default: live = -2; break;
    }
    if (live != expected_.mode) return true;
  }
  if (!std::isnan(expected_.setpoint)) {
    if (std::fabs((float) ctrl->getMasterSetpoint() - expected_.setpoint) > kTol) return true;
  }
  for (int z = 0; z < 8; z++) {
    if (expected_.zone_on[z] >= 0 && (int) ctrl->getZoneOn(z + 1) != expected_.zone_on[z]) return true;
    if (!std::isnan(expected_.zone_setpoint[z])) {
      if (std::fabs((float) ctrl->getZoneSetpointTemperature(z + 1) - expected_.zone_setpoint[z]) > kTol)
        return true;
    }
  }
  return false;
}

void Scheduler::loop() {
  if (!loaded_) return;
  const uint32_t nowms = millis();
  if (last_eval_ms_ != 0 && (nowms - last_eval_ms_) < kEvalIntervalMs) return;
  last_eval_ms_ = nowms;

  auto *rtc = parent_->get_time();
  if (rtc == nullptr) return;
  ESPTime t = rtc->now();
  if (!t.is_valid()) return;  // clock not synced yet — never fire blind

  const uint32_t now_unix = (uint32_t) t.timestamp;

  // Away: auto-resume at return time, otherwise suspend all firing.
  if (blob_.away_active) {
    if (blob_.away_return_at != 0 && now_unix >= blob_.away_return_at) {
      blob_.away_active = 0;
      blob_.away_return_at = 0;
      save_();
      ESP_LOGI(TAG, "Away ended — schedules resumed");
    } else {
      active_id_ = 0;
      overridden_ = false;
      return;
    }
  }

  const int dow = (t.day_of_week >= 1 && t.day_of_week <= 7) ? (t.day_of_week - 1) : 0;  // 0=Sun
  const uint16_t now_min = (uint16_t)(t.hour * 60 + t.minute);
  const uint32_t local_day = (uint32_t) t.year * 366u + t.day_of_year;

  // Home location for solar triggers (computed on-device; no weather call).
  const bool loc_ok = parent_->weather_location_set();
  const float lat = parent_->weather_latitude();
  const float lon = parent_->weather_longitude();

  // Collect schedules that fire this tick (trigger matched + conditions pass).
  // Apply in priority-ascending order so the highest priority wins on overlap.
  struct Fired { int idx; int8_t priority; };
  std::vector<Fired> fires;

  for (int i = 0; i < blob_.count; i++) {
    Schedule &s = blob_.records[i];
    if (!s.enabled) continue;

    bool trigger = false;
    uint32_t occ = 0;
    bool is_timer = (s.trigger_type == TriggerType::Timer);

    if (s.trigger_type == TriggerType::Calendar || s.trigger_type == TriggerType::Solar) {
      if (!(s.days_mask & (1u << dow))) continue;
      // Effective fire minute: a fixed clock time, or today's computed
      // sunrise/sunset ± offset.
      int eff_min;
      if (s.trigger_type == TriggerType::Calendar) {
        eff_min = s.calendar_min;
      } else {
        if (!loc_ok) continue;  // no location → can't compute solar; skip safely
        if (!compute_solar_local_min(t, lat, lon, (SolarEvent) s.solar_event, s.solar_offset, eff_min))
          continue;  // polar day/night
      }
      // Fire if now is within [eff, eff+catchup) and not yet fired this day.
      if ((int) now_min >= eff_min && (int) now_min < eff_min + SCHED_CATCHUP_MINUTES) {
        occ = occurrence_key_(local_day, (uint16_t) eff_min);
        if (!fired_already_(s.id, occ)) trigger = true;
      }
    } else {  // Timer
      if (now_unix >= s.timer_fire_at) trigger = true;
    }

    if (!trigger) continue;

    bool pass = conditions_pass_(s);
    if (pass) {
      fires.push_back({i, s.priority});
      if (!is_timer) mark_fired_(s.id, occ);
    } else if (!is_timer) {
      // Calendar miss: still mark so we don't re-test every tick this minute.
      mark_fired_(s.id, occ);
    }
    // One-shot timers are consumed once their fire time passes, whether or not
    // the conditions gated the action — handled in the delete pass below.
  }

  if (!fires.empty()) {
    std::sort(fires.begin(), fires.end(),
              [](const Fired &a, const Fired &b) { return a.priority < b.priority; });
    for (auto &f : fires) apply_action_(blob_.records[f.idx].action);
    const Schedule &top = blob_.records[fires.back().idx];
    active_id_ = top.id;
    snapshot_expected_(top.action);
    overridden_ = false;
    ESP_LOGI(TAG, "Applied schedule #%u (%s)", top.id, top.name);
  } else {
    // No new firing — check whether the user has overridden the active rule.
    overridden_ = detect_override_();
  }

  // Delete consumed one-shot timers (fire time elapsed). Done after apply so a
  // timer that fired this tick is removed once.
  bool removed = false;
  for (int i = blob_.count - 1; i >= 0; i--) {
    Schedule &s = blob_.records[i];
    if (s.trigger_type == TriggerType::Timer && now_unix >= s.timer_fire_at) {
      if (active_id_ == s.id) active_id_ = 0;
      for (int j = i; j < blob_.count - 1; j++) blob_.records[j] = blob_.records[j + 1];
      blob_.count--;
      removed = true;
    }
  }
  if (removed) save_();
}

uint32_t Scheduler::next_event_(uint16_t &which_id) {
  which_id = 0;
  auto *rtc = parent_->get_time();
  if (rtc == nullptr) return 0;
  ESPTime t = rtc->now();
  if (!t.is_valid()) return 0;
  const uint32_t now_unix = (uint32_t) t.timestamp;
  const int today_dow = (t.day_of_week >= 1 && t.day_of_week <= 7) ? (t.day_of_week - 1) : 0;
  const int now_min = t.hour * 60 + t.minute;
  const bool loc_ok = parent_->weather_location_set();
  const float lat = parent_->weather_latitude();
  const float lon = parent_->weather_longitude();

  uint32_t best = 0;
  for (int i = 0; i < blob_.count; i++) {
    const Schedule &s = blob_.records[i];
    if (!s.enabled) continue;
    uint32_t cand = 0;
    if (s.trigger_type == TriggerType::Timer) {
      if (s.timer_fire_at > now_unix) cand = s.timer_fire_at;
    } else {
      // Calendar/solar minute. Approximates future days with today's solar
      // time and ignores the once-a-year DST shift — a display hint, not a
      // fire decision.
      int eff = s.calendar_min;
      if (s.trigger_type == TriggerType::Solar) {
        if (!loc_ok ||
            !compute_solar_local_min(t, lat, lon, (SolarEvent) s.solar_event, s.solar_offset, eff))
          continue;
      }
      for (int d = 0; d <= 7; d++) {
        int dow = (today_dow + d) % 7;
        if (!(s.days_mask & (1u << dow))) continue;
        if (d == 0 && eff <= now_min) continue;
        cand = now_unix + (uint32_t) d * 86400u + (uint32_t)(eff - now_min) * 60u;
        break;
      }
    }
    if (cand != 0 && (best == 0 || cand < best)) { best = cand; which_id = s.id; }
  }
  return best;
}

// ---- JSON serialization ------------------------------------------------------

void Scheduler::schedule_to_json(const Schedule &s, void *json_obj) {
  JsonObject o = *reinterpret_cast<JsonObject *>(json_obj);
  o["id"] = s.id;
  o["name"] = s.name;
  o["enabled"] = (bool) s.enabled;
  o["priority"] = (int) s.priority;

  JsonObject trig = o["trigger"].to<JsonObject>();
  if (s.trigger_type == TriggerType::Timer) {
    trig["type"] = "timer";
    trig["fire_at"] = s.timer_fire_at;
  } else if (s.trigger_type == TriggerType::Solar) {
    trig["type"] = "solar";
    trig["event"] = (s.solar_event == (uint8_t) SolarEvent::Sunset) ? "sunset" : "sunrise";
    trig["offset"] = (int) s.solar_offset;
    JsonArray days = trig["days"].to<JsonArray>();
    for (int d = 0; d < 7; d++) {
      if (s.days_mask & (1u << d)) days.add(kDayNames[d]);
    }
  } else {
    trig["type"] = "calendar";
    char hhmm[6];
    snprintf(hhmm, sizeof(hhmm), "%02u:%02u", s.calendar_min / 60, s.calendar_min % 60);
    trig["time"] = hhmm;
    JsonArray days = trig["days"].to<JsonArray>();
    for (int d = 0; d < 7; d++) {
      if (s.days_mask & (1u << d)) days.add(kDayNames[d]);
    }
  }

  JsonArray conds = o["conditions"].to<JsonArray>();
  for (int i = 0; i < s.condition_count && i < SCHED_MAX_CONDITIONS; i++) {
    const Condition &c = s.conditions[i];
    JsonObject co = conds.add<JsonObject>();
    co["source"] = source_to_str(c.source);
    co["op"] = op_to_str(c.op);
    co["value"] = c.value1;
    if (c.op == CondOp::Between) co["value2"] = c.value2;
    if (c.source == CondSource::ZoneTemp || c.source == CondSource::Humidity) co["zone"] = c.zone;
  }

  JsonObject act = o["action"].to<JsonObject>();
  if (s.action.power != TriBool::Unset) act["power"] = (s.action.power == TriBool::True);
  if (s.action.mode != ActMode::None) act["mode"] = mode_to_str(s.action.mode);
  if (!std::isnan(s.action.setpoint)) act["setpoint"] = s.action.setpoint;
  if (s.action.fan != ActFan::None) act["fan"] = fan_to_str(s.action.fan);
  if (s.action.continuous != TriBool::Unset) act["continuous"] = (s.action.continuous == TriBool::True);
  if (s.action.quiet != TriBool::Unset) act["quiet"] = (s.action.quiet == TriBool::True);
  bool any_zone = false;
  for (int z = 0; z < 8; z++) {
    if (s.action.zone_on[z] != TriBool::Unset || !std::isnan(s.action.zone_setpoint[z])) { any_zone = true; break; }
  }
  if (any_zone) {
    JsonArray zs = act["zones"].to<JsonArray>();
    for (int z = 0; z < 8; z++) {
      if (s.action.zone_on[z] == TriBool::Unset && std::isnan(s.action.zone_setpoint[z])) continue;
      JsonObject zo = zs.add<JsonObject>();
      zo["n"] = z + 1;
      if (s.action.zone_on[z] != TriBool::Unset) zo["on"] = (s.action.zone_on[z] == TriBool::True);
      if (!std::isnan(s.action.zone_setpoint[z])) zo["setpoint"] = s.action.zone_setpoint[z];
    }
  }
}

bool Scheduler::json_to_schedule(const std::string &body, Schedule &s, std::string &err) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { err = "invalid JSON"; return false; }
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull()) { err = "expected an object"; return false; }

  // Defaults (id preserved by caller).
  const char *name = o["name"] | "";
  strncpy(s.name, name, SCHED_NAME_MAX);
  s.name[SCHED_NAME_MAX] = '\0';
  s.enabled = (o["enabled"] | true) ? 1 : 0;
  s.priority = (int8_t)(int) (o["priority"] | 0);

  JsonObject trig = o["trigger"];
  if (trig.isNull()) { err = "missing trigger"; return false; }
  const char *ttype = trig["type"] | "";
  if (!strcmp(ttype, "timer")) {
    s.trigger_type = TriggerType::Timer;
    if (trig["fire_at"].is<uint32_t>()) {
      s.timer_fire_at = trig["fire_at"].as<uint32_t>();
    } else if (trig["fire_in_sec"].is<long>()) {
      auto *rtc = parent_->get_time();
      uint32_t now_unix = (rtc && rtc->now().is_valid()) ? (uint32_t) rtc->now().timestamp : 0;
      if (now_unix == 0) { err = "clock not synced; cannot schedule a relative timer"; return false; }
      s.timer_fire_at = now_unix + (uint32_t) trig["fire_in_sec"].as<long>();
    } else {
      err = "timer needs fire_in_sec or fire_at";
      return false;
    }
    s.days_mask = 0;
    s.calendar_min = 0;
  } else if (!strcmp(ttype, "calendar")) {
    s.trigger_type = TriggerType::Calendar;
    const char *hhmm = trig["time"] | "";
    int hh = 0, mm = 0;
    if (sscanf(hhmm, "%d:%d", &hh, &mm) != 2 || hh < 0 || hh > 23 || mm < 0 || mm > 59) {
      err = "calendar trigger needs time \"HH:MM\"";
      return false;
    }
    s.calendar_min = (uint16_t)(hh * 60 + mm);
    s.days_mask = 0;
    JsonArray days = trig["days"];
    if (days.isNull() || days.size() == 0) { err = "calendar trigger needs days[]"; return false; }
    for (JsonVariant dv : days) {
      int idx = -1;
      if (dv.is<const char *>()) idx = day_name_to_index(dv.as<const char *>());
      else if (dv.is<int>()) { int v = dv.as<int>(); if (v >= 0 && v <= 6) idx = v; }
      if (idx < 0) { err = "bad day in days[]"; return false; }
      s.days_mask |= (uint8_t)(1u << idx);
    }
    s.timer_fire_at = 0;
  } else if (!strcmp(ttype, "solar")) {
    s.trigger_type = TriggerType::Solar;
    const char *ev = trig["event"] | "";
    if (!strcmp(ev, "sunrise")) {
      s.solar_event = (uint8_t) SolarEvent::Sunrise;
    } else if (!strcmp(ev, "sunset")) {
      s.solar_event = (uint8_t) SolarEvent::Sunset;
    } else {
      err = "solar trigger needs event sunrise|sunset";
      return false;
    }
    long off = trig["offset"] | 0L;
    if (off < -720 || off > 720) { err = "solar offset must be within +/-720 min"; return false; }
    s.solar_offset = (int16_t) off;
    s.calendar_min = 0;
    s.timer_fire_at = 0;
    s.days_mask = 0;
    JsonArray days = trig["days"];
    if (days.isNull() || days.size() == 0) { err = "solar trigger needs days[]"; return false; }
    for (JsonVariant dv : days) {
      int idx = -1;
      if (dv.is<const char *>()) idx = day_name_to_index(dv.as<const char *>());
      else if (dv.is<int>()) { int v = dv.as<int>(); if (v >= 0 && v <= 6) idx = v; }
      if (idx < 0) { err = "bad day in days[]"; return false; }
      s.days_mask |= (uint8_t)(1u << idx);
    }
  } else {
    err = "trigger.type must be calendar, solar, or timer";
    return false;
  }

  // Conditions.
  s.condition_count = 0;
  memset(s.conditions, 0, sizeof(s.conditions));
  JsonArray conds = o["conditions"];
  if (!conds.isNull()) {
    for (JsonObject co : conds) {
      if (s.condition_count >= SCHED_MAX_CONDITIONS) { err = "too many conditions (max 4)"; return false; }
      Condition &c = s.conditions[s.condition_count];
      if (!str_to_source(co["source"] | (const char *) nullptr, c.source)) { err = "bad condition.source"; return false; }
      if (!str_to_op(co["op"] | (const char *) nullptr, c.op)) { err = "bad condition.op"; return false; }
      if (!co["value"].is<float>() && !co["value"].is<int>()) { err = "condition.value required"; return false; }
      c.value1 = co["value"].as<float>();
      c.value2 = co["value2"] | NAN;
      if (c.op == CondOp::Between && std::isnan(c.value2)) { err = "between needs value2"; return false; }
      c.zone = (uint8_t)(int) (co["zone"] | 0);
      if ((c.source == CondSource::ZoneTemp || c.source == CondSource::Humidity) &&
          (c.zone < 1 || c.zone > 8)) {
        err = "zone_temp/humidity condition needs zone 1..8";
        return false;
      }
      s.condition_count++;
    }
  }

  // Action.
  Action &a = s.action;
  memset(&a, 0, sizeof(a));
  a.power = TriBool::Unset; a.mode = ActMode::None; a.fan = ActFan::None;
  a.continuous = TriBool::Unset; a.quiet = TriBool::Unset;
  a.setpoint = NAN;
  for (int z = 0; z < 8; z++) { a.zone_on[z] = TriBool::Unset; a.zone_setpoint[z] = NAN; }

  JsonObject act = o["action"];
  if (act.isNull()) { err = "missing action"; return false; }
  if (act["power"].is<bool>()) a.power = act["power"].as<bool>() ? TriBool::True : TriBool::False;
  if (act["mode"].is<const char *>()) {
    a.mode = str_to_mode(act["mode"].as<const char *>());
    if (a.mode == ActMode::None) { err = "bad action.mode"; return false; }
  }
  if (act["fan"].is<const char *>()) {
    a.fan = str_to_fan(act["fan"].as<const char *>());
    if (a.fan == ActFan::None) { err = "bad action.fan"; return false; }
  }
  if (act["continuous"].is<bool>()) a.continuous = act["continuous"].as<bool>() ? TriBool::True : TriBool::False;
  if (act["quiet"].is<bool>()) a.quiet = act["quiet"].as<bool>() ? TriBool::True : TriBool::False;
  if (act["setpoint"].is<float>() || act["setpoint"].is<int>()) a.setpoint = act["setpoint"].as<float>();
  JsonArray zones = act["zones"];
  if (!zones.isNull()) {
    for (JsonObject zo : zones) {
      int n = zo["n"] | 0;
      if (n < 1 || n > 8) { err = "action.zones[].n must be 1..8"; return false; }
      if (zo["on"].is<bool>()) a.zone_on[n - 1] = zo["on"].as<bool>() ? TriBool::True : TriBool::False;
      if (zo["setpoint"].is<float>() || zo["setpoint"].is<int>()) a.zone_setpoint[n - 1] = zo["setpoint"].as<float>();
    }
  }
  return true;
}

// ---- REST surface ------------------------------------------------------------

std::string Scheduler::list_json() {
  JsonDocument doc;
  JsonArray arr = doc["schedules"].to<JsonArray>();
  for (int i = 0; i < blob_.count; i++) {
    JsonObject o = arr.add<JsonObject>();
    schedule_to_json(blob_.records[i], &o);
  }
  doc["count"] = blob_.count;
  doc["max"] = SCHED_MAX;
  std::string out;
  serializeJson(doc, out);
  return out;
}

bool Scheduler::get_json(uint16_t id, std::string &out) {
  Schedule *s = find_(id);
  if (s == nullptr) return false;
  JsonDocument doc;
  JsonObject o = doc.to<JsonObject>();
  schedule_to_json(*s, &o);
  serializeJson(doc, out);
  return true;
}

bool Scheduler::upsert_from_json(const std::string &body, bool is_update, uint16_t id,
                                 uint16_t &out_id, std::string &err) {
  Schedule tmp{};
  if (!json_to_schedule(body, tmp, err)) return false;

  if (is_update) {
    int idx = index_of_(id);
    if (idx < 0) { err = "not found"; return false; }
    tmp.id = id;
    blob_.records[idx] = tmp;
    out_id = id;
  } else {
    if (blob_.count >= SCHED_MAX) { err = "schedule limit reached"; return false; }
    tmp.id = blob_.next_id++;
    if (blob_.next_id == 0) blob_.next_id = 1;  // never hand out 0 (means "none")
    blob_.records[blob_.count++] = tmp;
    out_id = tmp.id;
  }
  save_();
  return true;
}

bool Scheduler::create_timer_from_json(const std::string &body, uint16_t &out_id, std::string &err) {
  // A timer is just a schedule with a Timer trigger; if the caller didn't wrap
  // their body in a trigger object, synthesize one from fire_in_sec/fire_at.
  JsonDocument doc;
  if (deserializeJson(doc, body)) { err = "invalid JSON"; return false; }
  JsonObject o = doc.as<JsonObject>();
  if (o["trigger"].isNull()) {
    JsonObject trig = o["trigger"].to<JsonObject>();
    trig["type"] = "timer";
    if (o["fire_at"].is<uint32_t>()) trig["fire_at"] = o["fire_at"].as<uint32_t>();
    else if (o["fire_in_sec"].is<long>()) trig["fire_in_sec"] = o["fire_in_sec"].as<long>();
    else { err = "timer needs fire_in_sec or fire_at"; return false; }
  }
  std::string normalized;
  serializeJson(doc, normalized);
  return upsert_from_json(normalized, false, 0, out_id, err);
}

bool Scheduler::set_enabled(uint16_t id, bool enabled) {
  Schedule *s = find_(id);
  if (s == nullptr) return false;
  s->enabled = enabled ? 1 : 0;
  if (!enabled && active_id_ == id) { active_id_ = 0; overridden_ = false; }
  save_();
  return true;
}

bool Scheduler::remove(uint16_t id) {
  int idx = index_of_(id);
  if (idx < 0) return false;
  for (int j = idx; j < blob_.count - 1; j++) blob_.records[j] = blob_.records[j + 1];
  blob_.count--;
  if (active_id_ == id) { active_id_ = 0; overridden_ = false; }
  save_();
  return true;
}

std::string Scheduler::status_json() {
  JsonDocument doc;
  auto *rtc = parent_->get_time();
  bool synced = rtc != nullptr && rtc->now().is_valid();
  doc["clock_synced"] = synced;
  doc["responder_mode"] = parent_->state_receiving_data();
  if (active_id_ != 0) doc["active_schedule_id"] = active_id_;
  else doc["active_schedule_id"] = nullptr;
  doc["overridden"] = overridden_;

  uint16_t which = 0;
  uint32_t when = next_event_(which);
  if (when != 0) {
    JsonObject ne = doc["next_event"].to<JsonObject>();
    ne["schedule_id"] = which;
    ne["epoch"] = when;
  } else {
    doc["next_event"] = nullptr;
  }

  JsonObject away = doc["away"].to<JsonObject>();
  away["active"] = (bool) blob_.away_active;
  if (blob_.away_return_at != 0) away["return_at"] = blob_.away_return_at;
  else away["return_at"] = nullptr;

  std::string out;
  serializeJson(doc, out);
  return out;
}

std::string Scheduler::away_json() {
  JsonDocument doc;
  doc["active"] = (bool) blob_.away_active;
  if (blob_.away_return_at != 0) doc["return_at"] = blob_.away_return_at;
  else doc["return_at"] = nullptr;
  std::string out;
  serializeJson(doc, out);
  return out;
}

bool Scheduler::set_away_from_json(const std::string &body, std::string &err) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { err = "invalid JSON"; return false; }
  JsonObject o = doc.as<JsonObject>();
  if (!o["active"].is<bool>()) { err = "active (bool) required"; return false; }
  blob_.away_active = o["active"].as<bool>() ? 1 : 0;
  if (o["return_at"].is<uint32_t>()) blob_.away_return_at = o["return_at"].as<uint32_t>();
  else if (o["return_at"].isNull()) blob_.away_return_at = 0;
  if (!blob_.away_active) { active_id_ = 0; overridden_ = false; }
  save_();
  return true;
}

}  // namespace actron485_api
}  // namespace esphome
