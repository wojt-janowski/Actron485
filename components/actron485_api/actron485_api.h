#pragma once

#include <string>
#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/components/actron485/actron485_climate.h"
#include "Actron485.h"

namespace esphome {
namespace actron485_api {

class Actron485ApiHandler;  // fwd

class Actron485Api : public Component {
 public:
  void set_climate(actron485::Actron485Climate *climate) { climate_ = climate; }
  void set_auth_token(const std::string &token) { auth_token_ = token; }
  void set_sensor_stale_timeout_ms(uint32_t ms) { sensor_stale_timeout_ms_ = ms; }
  void set_demo_mode(bool on) { demo_mode_ = on; }
  bool demo_mode() const { return demo_mode_; }

  // ---- Weather proxy ----
  // Poll interval is the only compile-time tunable (actron.yaml). The API
  // key and the home location are entered at runtime via the dashboard and
  // persisted to NVS, mirroring the auth api_key pattern.
  void set_weather_update_interval_ms(uint32_t ms) { weather_update_interval_ms_ = ms; }

  // Runtime setters (dashboard + PATCH /api/v1/settings). Both persist to
  // NVS. Empty key disables weather; lat/lon outside valid ranges clears
  // the location. Thread-safe — guarded against the weather task.
  void set_weather_api_key_runtime(const std::string &key);
  void set_weather_location_runtime(float lat, float lon);

  // Read-backs for the dashboard widgets / GET /api/v1/settings.
  bool weather_api_key_set();
  bool weather_location_set();
  float weather_latitude();
  float weather_longitude();

  // Site timezone as a POSIX TZ string (e.g. "AEST-10AEDT,M10.1.0,M4.1.0/3").
  // Bridge-authoritative: served in /api/v1/info so every wall panel inherits
  // it. The DST rule is embedded in the string, so it auto-adjusts. Persisted
  // in its own NVS slot so the weather/api_key settings blob is untouched.
  std::string timezone();
  void set_timezone_runtime(const std::string &tz);

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Called by the temperature handler on each successful POST.
  void note_zone_temperature_update(uint8_t zone);

  // Returns the display name for a zone (1..8): override (if set) > ESPHome
  // entity name > "Zone N".
  std::string get_zone_display_name(int zone);

  // Persists an override name for a zone. Empty string clears the override
  // (the ESPHome entity name is used again). Returns false if zone is
  // out of range or the name is too long.
  bool set_zone_name_override(uint8_t zone, const std::string &name);

  // ---- Runtime-mutable settings (PATCH /api/v1/settings) ----
  // Each of these persists to NVS so it survives reboots. They override
  // whatever was set at compile-time in actron.yaml, on the principle
  // that the most recent user instruction wins (yaml is the factory
  // default; runtime overrides via the API stick). Pass empty string
  // to set_api_key_runtime("") to clear the key (auth disabled).
  void set_api_key_runtime(const std::string &key);
  void set_act_as_slave_3_runtime(bool on);
  void set_logging_mode_runtime(int mode);
  // Snapshot of currently-effective settings for GET /api/v1/settings.
  // api_key is returned masked ("***" if set, "" if not).
  std::string build_settings_json();

  Actron485::Controller *controller() { return climate_->get_controller(); }
  actron485::Actron485Climate *climate() { return climate_; }
  const std::string &auth_token() const { return auth_token_; }

  // Serialized state snapshot used by both GET /state and (future) streaming.
  std::string build_state_json();

  // Serialized last-known-good weather snapshot for GET /api/v1/weather.
  // Safe to call from the HTTP task — reads cached values under the mutex.
  std::string build_weather_json();

  // Human-readable one-liner for the ESPHome dashboard status sensor, e.g.
  // "18.7°C · Partly cloudy", "Waiting for data…", or "Not configured".
  std::string weather_status_summary();

  // Write-path wrappers. In normal mode these delegate to the Actron485
  // controller. In demo mode they mutate only the simulator's state and
  // never touch the RS485 bus.
  void apply_system_on(bool on);
  void apply_operating_mode(Actron485::OperatingMode mode);
  void apply_fan_speed(Actron485::FanMode mode);
  void apply_continuous_fan(bool on);
  void apply_quiet_mode(bool on);
  void apply_master_setpoint(double temperature);
  void apply_zone_on(uint8_t zone, bool on);
  void apply_zone_setpoint(uint8_t zone, double temperature);
  void apply_zone_control(uint8_t zone, bool enabled);
  void apply_zone_current_temperature(uint8_t zone, double temperature);

  // State accessor that respects demo mode.
  bool state_receiving_data();

 protected:
  actron485::Actron485Climate *climate_{nullptr};
  std::string auth_token_;  // empty = no auth
  Actron485ApiHandler *handler_{nullptr};

  // Stale-sensor safety. last_temp_update_ms_[i] == 0 means we've never
  // received a /temperature POST for that zone, so the watchdog ignores it.
  uint32_t sensor_stale_timeout_ms_{600000};
  unsigned long last_temp_update_ms_[8]{};
  unsigned long last_stale_check_ms_{0};

  // Per-zone humidity from external sensors (POST .../humidity). The AC
  // bus doesn't carry per-zone humidity; we just relay the most recent
  // POSTed value through /api/v1/state for clients to consume. NaN
  // means "no reading received yet" — clients should render that as
  // "—" or hide the field.
  float zone_humidity_[8]{NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
  unsigned long last_humidity_update_ms_[8]{};
 public:
  void note_zone_humidity_update(uint8_t zone, float humidity);
  float zone_humidity(uint8_t zone) const {
    return (zone >= 1 && zone <= 8) ? zone_humidity_[zone - 1] : NAN;
  }
 protected:

  // ---- Demo-mode simulation state ----
  // Only used when demo_mode_ is true. On a real device these are all
  // ignored and the Actron485 controller owns the state.
  bool demo_mode_{false};
  bool demo_system_on_{false};
  Actron485::OperatingMode demo_op_mode_{Actron485::OperatingMode::Off};
  Actron485::FanMode demo_fan_{Actron485::FanMode::Esp};
  bool demo_continuous_fan_{false};
  float demo_setpoint_{22.0f};
  float demo_current_{21.4f};
  bool demo_zone_on_[8]{};
  bool demo_zone_control_[8]{};
  float demo_zone_setpoint_[8]{22.0f, 22.0f, 22.0f, 22.0f, 22.0f, 22.0f, 22.0f, 22.0f};
  float demo_zone_current_[8]{21.4f, 21.4f, 21.4f, 21.4f, 21.4f, 21.4f, 21.4f, 21.4f};
  unsigned long demo_last_tick_ms_{0};
  unsigned long demo_last_publish_ms_{0};
  // "Last value we wrote into the Climate/fan entities from demo_tick".
  // Used at the next tick to detect web-UI-driven changes (where the
  // current climate_ value differs from what we last pushed) vs
  // API-driven changes (where demo_* diverges but climate_ is unchanged).
  float demo_last_pub_setpoint_{NAN};
  int demo_last_pub_mode_{-1};
  int demo_last_pub_fan_{-1};
  bool demo_last_pub_zone_on_[8]{};
  float demo_last_pub_zone_setpoint_[8]{NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
  void demo_tick_();

  // Zone name overrides persisted to flash via ESPHome preferences.
  // Empty string means "no override; use the ESPHome entity name".
  static constexpr size_t ZONE_NAME_MAX = 31;
  struct ZoneNamesBlob {
    char names[8][ZONE_NAME_MAX + 1];
  };
  std::string zone_name_overrides_[8];
  ESPPreferenceObject zone_names_pref_;
  void load_zone_names_();
  void save_zone_names_();

  // Runtime-mutable settings persisted alongside zone names. yaml sets
  // factory defaults at compile time; any NVS-stored value takes
  // precedence at boot. PATCH /api/v1/settings writes through both
  // the live state and the NVS slot.
  //
  // act_as_slave_3_runtime_ / logging_mode_runtime_ are tri-state: -1
  // means "no override, use yaml". 0/1/2/... are the active values.
  static constexpr uint32_t kSettingsMagic   = 0xAC773501;
  // v2 adds the weather api key + lat/lon. The NVS slot key is versioned
  // (`..._settings_v2`), so a v1 device upgrading simply falls back to yaml
  // defaults for the non-weather settings (act_as_slave_3 also defaults via
  // yaml; api_key was unset) — acceptable, and avoids unaligned-float
  // migration of the old blob.
  static constexpr uint32_t kSettingsVersion  = 2;
  static constexpr size_t   API_KEY_MAX      = 63;
  static constexpr size_t   WEATHER_KEY_MAX  = 63;
  struct SettingsBlob {
    uint32_t magic;
    uint32_t version;
    uint8_t  act_as_slave_3;       // 0=off, 1=on
    uint8_t  logging_mode;         // 0..5 — same enum as yaml `logging_mode`
    uint8_t  has_api_key;          // 0=none, 1=use api_key field
    uint8_t  has_weather_key;      // 0=none, 1=use weather_api_key field
    uint8_t  weather_location_set; // 0=unset, 1=lat/lon valid
    uint8_t  _pad[3];              // keep api_key (and later floats) aligned
    char     api_key[API_KEY_MAX + 1];
    char     weather_api_key[WEATHER_KEY_MAX + 1];
    float    weather_lat;
    float    weather_lon;
  } __attribute__((packed));
  ESPPreferenceObject settings_pref_;
  bool settings_loaded_{false};
  // Cached for fast PATCH echoes; sourced from NVS at boot (or yaml
  // fallback if NVS is empty).
  bool settings_act_as_slave_3_{false};
  int  settings_logging_mode_{1};  // 1 = STATUS, ESPHome default
  void load_settings_();
  void save_settings_();

  // Site timezone (POSIX TZ string). Stored in its own NVS slot so changing it
  // never disturbs the versioned SettingsBlob (api_key / weather). Defaults to
  // Sydney with DST when nothing is persisted.
  static constexpr size_t TIMEZONE_MAX = 47;
  struct TimezoneBlob {
    char tz[TIMEZONE_MAX + 1];
  };
  ESPPreferenceObject timezone_pref_;
  std::string timezone_{"AEST-10AEDT,M10.1.0,M4.1.0/3"};
  void load_timezone_();
  void save_timezone_();

  // ---- Weather proxy state ----
  // The fetch runs on a dedicated FreeRTOS task, never on loop(): a
  // blocking HTTPS GET would stall the main loop and, with the slave-3
  // responder active, risk dropping AMIB Modbus polls. loop()/the HTTP
  // handler only ever read the cached fields below, under weather_mutex_.
  std::string weather_api_key_;
  float weather_lat_{0.0f};
  float weather_lon_{0.0f};
  bool  weather_location_set_{false};
  uint32_t weather_update_interval_ms_{900000};  // 15 min default

  SemaphoreHandle_t weather_mutex_{nullptr};
  TaskHandle_t weather_task_handle_{nullptr};
  bool weather_available_{false};
  float weather_temp_{NAN};
  float weather_humidity_{NAN};
  std::string weather_condition_;
  std::string weather_icon_;
  unsigned long weather_updated_ms_{0};
  // Last fetch error, surfaced on the dashboard / endpoint when there is no
  // usable reading. Empty once a fetch has succeeded. Set under the mutex.
  std::string weather_error_;

  static void weather_task_trampoline_(void *arg);
  void weather_task_();
  bool fetch_weather_();  // true on a successful fetch + parse
  void set_weather_error_(const std::string &msg);
  bool parse_weather_response_(const std::string &body);
  // Maps OpenWeather's icon code (e.g. "10d") / condition id to a small
  // stable glyph name the wall can render. Provider-specific logic lives
  // here so swapping to a keyless provider later stays localized.
  static const char *map_weather_icon_(const char *owm_icon, int owm_id);
};

class Actron485ApiHandler : public AsyncWebHandler {
 public:
  explicit Actron485ApiHandler(Actron485Api *parent) : parent_(parent) {}

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  bool isRequestHandlerTrivial() const override { return false; }

 protected:
  Actron485Api *parent_;

  bool authorized_(AsyncWebServerRequest *request);
  std::string read_body_(AsyncWebServerRequest *request);

  void send_json_(AsyncWebServerRequest *request, int code, const std::string &body);
  void send_error_(AsyncWebServerRequest *request, int code, const char *message);
  void add_cors_headers_(AsyncWebServerResponse *response);

  void handle_info_(AsyncWebServerRequest *request);
  void handle_state_(AsyncWebServerRequest *request);
  void handle_diagnostics_(AsyncWebServerRequest *request);
  void handle_weather_(AsyncWebServerRequest *request);
  void handle_bus_(AsyncWebServerRequest *request);
  void handle_demo_(AsyncWebServerRequest *request, const std::string &body);
  void handle_power_(AsyncWebServerRequest *request, const std::string &body);
  void handle_mode_(AsyncWebServerRequest *request, const std::string &body);
  void handle_fan_(AsyncWebServerRequest *request, const std::string &body);
  void handle_quiet_(AsyncWebServerRequest *request, const std::string &body);
  void handle_setpoint_(AsyncWebServerRequest *request, const std::string &body);
  void handle_zone_(AsyncWebServerRequest *request, int zone, const std::string &body);
  void handle_zone_control_(AsyncWebServerRequest *request, int zone, const std::string &body);
  void handle_zone_temperature_(AsyncWebServerRequest *request, int zone, const std::string &body);
  void handle_zone_humidity_(AsyncWebServerRequest *request, int zone, const std::string &body);
  void handle_zone_name_(AsyncWebServerRequest *request, int zone, const std::string &body);
  void handle_settings_get_(AsyncWebServerRequest *request);
  void handle_settings_patch_(AsyncWebServerRequest *request, const std::string &body);
};

}  // namespace actron485_api
}  // namespace esphome
