#pragma once

// Bridge-side scheduling engine. Decides WHEN to drive the AC; the actual
// driving reuses Actron485Api::apply_* (the same command path the REST control
// endpoints call). The engine lives in its own translation unit so it can grow
// without bloating actron485_api.cpp, and is owned + ticked by Actron485Api.
//
// Model: Schedule = Trigger + Conditions + Action.
//   - Trigger fires at a moment: a calendar time on selected weekdays, or a
//     one-shot countdown timer (self-deletes after firing).
//   - Conditions (all AND) gate the action at fire time; condition-only state
//     (e.g. outdoor temp) is re-checked on every tick so weather-gated rules
//     don't need a fixed clock time to react.
//   - Action is applied via the parent's apply_* methods.
// Conflict between rules firing on the same tick is resolved by priority
// (higher wins on overlapping fields). A manual change holds until the next
// trigger reasserts; the engine never fights the user, but surfaces an
// "overridden" flag so a controller can show Schedule-active vs Overridden.

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/preferences.h"

namespace esphome {
namespace time {
class RealTimeClock;
}  // namespace time
namespace actron485_api {

class Actron485Api;  // fwd — parent component, owns the command path + inputs

static constexpr int SCHED_MAX = 16;        // NVS blob cap (~2.4 KB total)
static constexpr int SCHED_MAX_CONDITIONS = 4;
static constexpr size_t SCHED_NAME_MAX = 31;

// Calendar fire tolerance: with a ~30 s eval tick (and to ride over a brief
// outage), a calendar rule fires if "now" is within this many minutes past its
// trigger minute and it hasn't already fired for that occurrence.
static constexpr int SCHED_CATCHUP_MINUTES = 5;

enum class TriggerType : uint8_t {
  Calendar = 0,  // time-of-day on a weekday bitmask
  Timer = 1,     // one-shot at an absolute unix time; deleted after firing
  Solar = 2,     // sunrise/sunset (± offset) on a weekday bitmask
};

enum class SolarEvent : uint8_t { Sunrise = 0, Sunset = 1 };

// Input a condition reads. Forecast/weather sources read the parent's
// mutex-guarded cache via typed accessors; zone sources use cond.zone.
enum class CondSource : uint8_t {
  OutdoorTemp = 0,       // weather proxy current temp, °C
  ZoneTemp = 1,          // zone current temp, °C (cond.zone)
  Humidity = 2,          // zone humidity, %RH (cond.zone)
  ForecastTodayMax = 3,  // today.temp_max, °C
  ForecastTodayMin = 4,  // today.temp_min, °C
  ForecastTodayPop = 5,  // today.pop, 0..100
};

enum class CondOp : uint8_t { Lt = 0, Gt = 1, Between = 2 };

// Action operating mode. None = "leave mode unchanged".
enum class ActMode : uint8_t { None = 0, Off = 1, Heat = 2, Cool = 3, Auto = 4, FanOnly = 5 };
// Action fan speed. None = "leave fan unchanged".
enum class ActFan : uint8_t { None = 0, Auto = 1, Low = 2, Medium = 3, High = 4 };
// Tri-state flag for optional booleans in an action. Unset = "leave unchanged".
enum class TriBool : uint8_t { Unset = 0, False = 1, True = 2 };

// One AND-ed gate. value2 is only used by Between.
struct Condition {
  CondSource source;
  CondOp op;
  uint8_t zone;  // 1..8 for ZoneTemp/Humidity; ignored otherwise
  uint8_t _pad;
  float value1;
  float value2;
} __attribute__((packed));

// What a schedule does when it fires. NaN setpoints / None enums / Unset
// tri-bools mean "leave this field as-is".
struct Action {
  TriBool power;
  ActMode mode;
  ActFan fan;
  TriBool continuous;
  TriBool quiet;
  uint8_t _pad[3];
  float setpoint;                 // master setpoint, NaN = unchanged
  TriBool zone_on[8];             // per-zone on/off, Unset = unchanged
  uint8_t _pad2[8];
  float zone_setpoint[8];         // per-zone setpoint, NaN = unchanged
} __attribute__((packed));

// One schedule. Packed so the array serializes straight into NVS.
struct Schedule {
  uint16_t id;
  uint8_t enabled;     // 0/1
  int8_t priority;     // higher wins on same-tick overlap
  char name[SCHED_NAME_MAX + 1];

  TriggerType trigger_type;
  uint8_t days_mask;       // calendar+solar: bit0=Sun .. bit6=Sat
  uint16_t calendar_min;   // calendar: minute of day 0..1439
  uint32_t timer_fire_at;  // timer: absolute unix seconds
  uint8_t solar_event;     // solar: SolarEvent (0=sunrise, 1=sunset)
  uint8_t _tpad;
  int16_t solar_offset;    // solar: signed minutes relative to the event

  uint8_t condition_count;
  uint8_t _pad[3];
  Condition conditions[SCHED_MAX_CONDITIONS];

  Action action;
} __attribute__((packed));

// Persisted blob: the schedule table + away state + id allocator.
struct SchedulesBlob {
  uint32_t magic;
  uint32_t version;
  uint16_t next_id;
  uint8_t count;
  uint8_t away_active;       // 0/1 — schedules suspended while set
  uint32_t away_return_at;   // 0 = none; auto-clears away at/after this unix time
  Schedule records[SCHED_MAX];
} __attribute__((packed));

class Scheduler {
 public:
  explicit Scheduler(Actron485Api *parent) : parent_(parent) {}

  void setup();  // load from NVS
  void loop();   // throttled evaluation tick

  // ---- REST surface (all JSON in/out; called from the HTTP handler) ----
  std::string list_json();                                  // GET /schedules
  bool get_json(uint16_t id, std::string &out);             // GET /schedules/{id}
  // Create/update from a JSON body. On create, id is allocated and returned in
  // out_id. err carries a human reason on failure (400).
  bool upsert_from_json(const std::string &body, bool is_update, uint16_t id,
                        uint16_t &out_id, std::string &err);
  bool set_enabled(uint16_t id, bool enabled);              // POST /schedules/{id}/enable
  bool remove(uint16_t id);                                 // POST /schedules/{id}/delete
  // Convenience for "off in 30 min" style one-shots. Body carries fire_in_sec
  // (or fire_at) + an action; creates a Timer schedule.
  bool create_timer_from_json(const std::string &body, uint16_t &out_id, std::string &err);

  std::string status_json();                                // GET /schedules/status
  std::string away_json();                                  // GET /away
  bool set_away_from_json(const std::string &body, std::string &err);  // POST /away

 protected:
  Actron485Api *parent_;
  SchedulesBlob blob_{};
  ESPPreferenceObject pref_;
  bool loaded_{false};

  uint32_t last_eval_ms_{0};

  // Override tracking. After the engine applies a schedule, it snapshots the
  // fields that schedule set; subsequent ticks compare live state to detect a
  // manual change. active_id_ == 0 means "no schedule currently owns state".
  uint16_t active_id_{0};
  bool overridden_{false};
  // Per-occurrence fire de-dup: last occurrence key fired, indexed by id.
  // RAM-only — a reboot inside the trigger minute re-applying the same action
  // is benign.
  struct FireMark { uint16_t id; uint32_t occurrence; };
  std::vector<FireMark> fired_;

  // Expected state snapshot (only fields the active schedule set are compared;
  // NaN / -1 mean "not set, don't compare").
  struct Expected {
    int power{-1};       // -1 unset, 0/1
    int mode{-1};        // -1 unset, else ActMode value
    float setpoint{NAN};
    int zone_on[8];      // -1 unset
    float zone_setpoint[8];
    Expected() { for (int i = 0; i < 8; i++) { zone_on[i] = -1; zone_setpoint[i] = NAN; } }
  } expected_;

  void save_();
  Schedule *find_(uint16_t id);
  int index_of_(uint16_t id);

  // Evaluation helpers.
  bool conditions_pass_(const Schedule &s);
  bool condition_value_(const Schedule &s, const Condition &c, float &live);
  void apply_action_(const Action &a);
  void snapshot_expected_(const Action &a);
  bool detect_override_();
  uint32_t occurrence_key_(uint32_t local_unix_day, uint16_t minute) const;
  bool fired_already_(uint16_t id, uint32_t occurrence);
  void mark_fired_(uint16_t id, uint32_t occurrence);
  // Soonest upcoming trigger across enabled schedules, 0 if none. Fills which.
  uint32_t next_event_(uint16_t &which_id);

  // JSON (de)serialization of a single schedule.
  void schedule_to_json(const Schedule &s, void *json_obj);  // void* = JsonObject
  bool json_to_schedule(const std::string &body, Schedule &s, std::string &err);
};

}  // namespace actron485_api
}  // namespace esphome
