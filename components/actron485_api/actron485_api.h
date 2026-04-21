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

 protected:
  actron485::Actron485Climate *climate_{nullptr};
  std::string auth_token_;  // empty = no auth
  Actron485ApiHandler *handler_{nullptr};

  // Stale-sensor safety. last_temp_update_ms_[i] == 0 means we've never
  // received a /temperature POST for that zone, so the watchdog ignores it.
  uint32_t sensor_stale_timeout_ms_{600000};
  unsigned long last_temp_update_ms_[8]{};
  unsigned long last_stale_check_ms_{0};

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
