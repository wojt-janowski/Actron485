#pragma once

#include <string>

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

  Actron485::Controller *controller() { return climate_->get_controller(); }
  actron485::Actron485Climate *climate() { return climate_; }
  const std::string &auth_token() const { return auth_token_; }

  // Serialized state snapshot used by both GET /state and (future) streaming.
  std::string build_state_json();

  // Write-path wrappers. In normal mode these delegate to the Actron485
  // controller. In demo mode they mutate only the simulator's state and
  // never touch the RS485 bus.
  void apply_system_on(bool on);
  void apply_operating_mode(Actron485::OperatingMode mode);
  void apply_fan_speed(Actron485::FanMode mode);
  void apply_continuous_fan(bool on);
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
  void handle_power_(AsyncWebServerRequest *request, const std::string &body);
  void handle_mode_(AsyncWebServerRequest *request, const std::string &body);
  void handle_fan_(AsyncWebServerRequest *request, const std::string &body);
  void handle_setpoint_(AsyncWebServerRequest *request, const std::string &body);
  void handle_zone_(AsyncWebServerRequest *request, int zone, const std::string &body);
  void handle_zone_control_(AsyncWebServerRequest *request, int zone, const std::string &body);
  void handle_zone_temperature_(AsyncWebServerRequest *request, int zone, const std::string &body);
  void handle_zone_name_(AsyncWebServerRequest *request, int zone, const std::string &body);
};

}  // namespace actron485_api
}  // namespace esphome
