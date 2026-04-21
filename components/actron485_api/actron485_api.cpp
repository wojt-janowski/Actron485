#include "actron485_api.h"

#include <cstdlib>
#include <cstring>

#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include <esp_http_server.h>
#include <ArduinoJson.h>

namespace esphome {
namespace actron485_api {

static const char *const TAG = "actron485_api";
static const size_t MAX_BODY_BYTES = 4 * 1024;

// ------------ String conversions ------------

static const char *operating_mode_to_string(Actron485::OperatingMode mode) {
  switch (mode) {
    case Actron485::OperatingMode::Off: return "off";
    case Actron485::OperatingMode::OffAuto: return "off_auto";
    case Actron485::OperatingMode::OffCool: return "off_cool";
    case Actron485::OperatingMode::OffHeat: return "off_heat";
    case Actron485::OperatingMode::FanOnly: return "fan_only";
    case Actron485::OperatingMode::Auto: return "auto";
    case Actron485::OperatingMode::Cool: return "cool";
    case Actron485::OperatingMode::Heat: return "heat";
  }
  return "unknown";
}

static bool string_to_operating_mode(const char *s, Actron485::OperatingMode &out) {
  if (!s) return false;
  if (!strcmp(s, "off")) { out = Actron485::OperatingMode::Off; return true; }
  if (!strcmp(s, "fan_only")) { out = Actron485::OperatingMode::FanOnly; return true; }
  if (!strcmp(s, "auto")) { out = Actron485::OperatingMode::Auto; return true; }
  if (!strcmp(s, "cool")) { out = Actron485::OperatingMode::Cool; return true; }
  if (!strcmp(s, "heat")) { out = Actron485::OperatingMode::Heat; return true; }
  return false;
}

static const char *fan_mode_to_string(Actron485::FanMode mode) {
  switch (mode) {
    case Actron485::FanMode::Off: return "off";
    case Actron485::FanMode::Low: case Actron485::FanMode::LowContinuous: return "low";
    case Actron485::FanMode::Medium: case Actron485::FanMode::MediumContinuous: return "medium";
    case Actron485::FanMode::High: case Actron485::FanMode::HighContinuous: return "high";
    case Actron485::FanMode::Esp: case Actron485::FanMode::EspContinuous: return "auto";
  }
  return "unknown";
}

static bool string_to_fan_mode(const char *s, Actron485::FanMode &out) {
  if (!s) return false;
  if (!strcmp(s, "low")) { out = Actron485::FanMode::Low; return true; }
  if (!strcmp(s, "medium")) { out = Actron485::FanMode::Medium; return true; }
  if (!strcmp(s, "high")) { out = Actron485::FanMode::High; return true; }
  if (!strcmp(s, "auto")) { out = Actron485::FanMode::Esp; return true; }
  return false;
}

static const char *compressor_to_string(Actron485::CompressorMode m) {
  switch (m) {
    case Actron485::CompressorMode::Idle: return "idle";
    case Actron485::CompressorMode::Cooling: return "cooling";
    case Actron485::CompressorMode::Heating: return "heating";
    default: return "unknown";
  }
}

// ================= Actron485Api =================

float Actron485Api::get_setup_priority() const {
  return setup_priority::LATE;
}

void Actron485Api::setup() {
  if (climate_ == nullptr) {
    ESP_LOGE(TAG, "No climate reference configured; API will not start");
    this->mark_failed();
    return;
  }
  auto *base = web_server_base::global_web_server_base;
  if (base == nullptr) {
    ESP_LOGE(TAG, "web_server_base not available; add web_server: to your YAML");
    this->mark_failed();
    return;
  }
  handler_ = new Actron485ApiHandler(this);
  base->add_handler(handler_);
  this->load_zone_names_();
  ESP_LOGCONFIG(TAG, "Actron485 API mounted at /api/v1/* on port %u", base->get_port());
}

void Actron485Api::load_zone_names_() {
  // Stable hash for the preference slot. Keep this constant across
  // firmware builds or you'll lose saved names.
  uint32_t hash = fnv1_hash(std::string("actron485_api_zone_names_v1"));
  zone_names_pref_ = global_preferences->make_preference<ZoneNamesBlob>(hash);
  ZoneNamesBlob blob{};
  if (zone_names_pref_.load(&blob)) {
    for (int i = 0; i < 8; i++) {
      // Defensive: ensure null-termination even if flash was corrupted.
      blob.names[i][ZONE_NAME_MAX] = '\0';
      zone_name_overrides_[i] = std::string(blob.names[i]);
    }
  }
}

void Actron485Api::save_zone_names_() {
  ZoneNamesBlob blob{};
  for (int i = 0; i < 8; i++) {
    const auto &s = zone_name_overrides_[i];
    size_t n = std::min(s.size(), ZONE_NAME_MAX);
    memcpy(blob.names[i], s.data(), n);
    blob.names[i][n] = '\0';
  }
  zone_names_pref_.save(&blob);
  global_preferences->sync();
}

std::string Actron485Api::get_zone_display_name(int zone) {
  if (zone < 1 || zone > 8) return std::string();
  const auto &override_name = zone_name_overrides_[zone - 1];
  if (!override_name.empty()) return override_name;
  if (auto *zc = climate_->get_zone_climate(zone)) {
    auto s = std::string(zc->get_name());
    if (!s.empty()) return s;
  }
  if (auto *zf = climate_->get_zone_fan(zone)) {
    auto s = std::string(zf->get_name());
    if (!s.empty()) return s;
  }
  return "Zone " + std::to_string(zone);
}

bool Actron485Api::set_zone_name_override(uint8_t zone, const std::string &name) {
  if (zone < 1 || zone > 8) return false;
  if (name.size() > ZONE_NAME_MAX) return false;
  zone_name_overrides_[zone - 1] = name;
  this->save_zone_names_();
  return true;
}

void Actron485Api::dump_config() {
  ESP_LOGCONFIG(TAG, "Actron485 API:");
  ESP_LOGCONFIG(TAG, "  Auth: %s", auth_token_.empty() ? "disabled" : "token");
  if (sensor_stale_timeout_ms_ == 0) {
    ESP_LOGCONFIG(TAG, "  Sensor stale timeout: disabled");
  } else {
    ESP_LOGCONFIG(TAG, "  Sensor stale timeout: %u ms", sensor_stale_timeout_ms_);
  }
  if (demo_mode_) {
    ESP_LOGW(TAG, "  !!! DEMO MODE ENABLED — NOT connected to any AC !!!");
    ESP_LOGW(TAG, "  !!! API returns simulated state; RS485 writes suppressed !!!");
  }
}

// -------- Write wrappers: controller in normal mode, local state in demo --------

void Actron485Api::apply_system_on(bool on) {
  if (demo_mode_) { demo_system_on_ = on; return; }
  controller()->setSystemOn(on);
}
void Actron485Api::apply_operating_mode(Actron485::OperatingMode mode) {
  if (demo_mode_) { demo_op_mode_ = mode; return; }
  controller()->setOperatingMode(mode);
}
void Actron485Api::apply_fan_speed(Actron485::FanMode mode) {
  if (demo_mode_) { demo_fan_ = mode; return; }
  controller()->setFanSpeed(mode);
}
void Actron485Api::apply_continuous_fan(bool on) {
  if (demo_mode_) { demo_continuous_fan_ = on; return; }
  controller()->setContinuousFanMode(on);
}
void Actron485Api::apply_master_setpoint(double temperature) {
  if (demo_mode_) { demo_setpoint_ = (float) temperature; return; }
  controller()->setMasterSetpoint(temperature);
}
void Actron485Api::apply_zone_on(uint8_t zone, bool on) {
  if (zone < 1 || zone > 8) return;
  if (demo_mode_) { demo_zone_on_[zone - 1] = on; return; }
  controller()->setZoneOn(zone, on);
}
void Actron485Api::apply_zone_setpoint(uint8_t zone, double temperature) {
  if (zone < 1 || zone > 8) return;
  if (demo_mode_) { demo_zone_setpoint_[zone - 1] = (float) temperature; return; }
  controller()->setZoneSetpointTemperatureCustom(zone, temperature, false);
}
void Actron485Api::apply_zone_control(uint8_t zone, bool enabled) {
  if (zone < 1 || zone > 8) return;
  if (demo_mode_) { demo_zone_control_[zone - 1] = enabled; return; }
  controller()->setControlZone(zone, enabled);
}
void Actron485Api::apply_zone_current_temperature(uint8_t zone, double temperature) {
  if (zone < 1 || zone > 8) return;
  if (demo_mode_) { demo_zone_current_[zone - 1] = (float) temperature; return; }
  controller()->setZoneCurrentTemperature(zone, temperature);
}

bool Actron485Api::state_receiving_data() {
  if (demo_mode_) return true;
  return controller()->receivingData();
}

// Drift zone currents toward their setpoints (or toward master setpoint if the
// zone has no setpoint set / isn't controlled), so the mobile app sees live-ish
// movement in demo mode. Master current is the mean of zone currents.
void Actron485Api::demo_tick_() {
  unsigned long now = millis();
  if (demo_last_tick_ms_ == 0) { demo_last_tick_ms_ = now; return; }
  float dt = (now - demo_last_tick_ms_) / 1000.0f;
  demo_last_tick_ms_ = now;
  if (dt <= 0) return;

  const float rate = 0.05f;  // °C per second toward target
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    float target = demo_zone_setpoint_[i];
    if (!demo_system_on_) target = 20.0f + (i * 0.1f);  // ambient drift when off
    float delta = target - demo_zone_current_[i];
    float step = std::max(-rate * dt, std::min(rate * dt, delta));
    demo_zone_current_[i] += step;
    sum += demo_zone_current_[i];
  }
  demo_current_ = sum / 8.0f;
}

void Actron485Api::note_zone_temperature_update(uint8_t zone) {
  if (zone < 1 || zone > 8) return;
  last_temp_update_ms_[zone - 1] = millis();
  if (last_temp_update_ms_[zone - 1] == 0) {
    // millis() can be 0 very early; nudge to 1 so the "never set" sentinel
    // (zero) still works.
    last_temp_update_ms_[zone - 1] = 1;
  }
}

void Actron485Api::loop() {
  if (demo_mode_) {
    this->demo_tick_();
    return;  // nothing else to do — the real bus isn't in play
  }
  if (sensor_stale_timeout_ms_ == 0) return;
  unsigned long now = millis();
  // Check at most once per second — cheap but avoids per-tick overhead.
  if (now - last_stale_check_ms_ < 1000) return;
  last_stale_check_ms_ = now;

  for (int i = 0; i < 8; i++) {
    if (last_temp_update_ms_[i] == 0) continue;  // never fed
    if (now - last_temp_update_ms_[i] < sensor_stale_timeout_ms_) continue;
    uint8_t zone = (uint8_t) (i + 1);
    if (controller()->getControlZone(zone)) {
      ESP_LOGW(TAG, "Zone %u sensor stale (no POST for %lu ms); releasing wall-controller role",
               zone, now - last_temp_update_ms_[i]);
      controller()->setControlZone(zone, false);
    }
    // Zero it so we don't keep firing. A fresh POST will re-arm.
    last_temp_update_ms_[i] = 0;
  }
}

std::string Actron485Api::build_state_json() {
  JsonDocument doc;
  auto root = doc.to<JsonObject>();

  // Monotonic ms since boot; the mobile app can compute freshness by diffing
  // two snapshots' updated_at_ms or comparing to its own request clock.
  root["updated_at_ms"] = (unsigned long) millis();

  if (demo_mode_) {
    root["status_received_ms"] = (unsigned long) millis();
    root["system_on"] = demo_system_on_;
    root["mode"] = operating_mode_to_string(demo_op_mode_);
    root["fan"] = fan_mode_to_string(demo_fan_);
    root["fan_running"] = fan_mode_to_string(demo_fan_);
    root["continuous_fan"] = demo_continuous_fan_;
    // Very simple "compressor" rule — if the system is on and in a thermal
    // mode, report the appropriate direction. Good enough to exercise
    // status/action indicators in the app.
    const char *compressor = "idle";
    if (demo_system_on_) {
      if (demo_op_mode_ == Actron485::OperatingMode::Cool) compressor = "cooling";
      else if (demo_op_mode_ == Actron485::OperatingMode::Heat) compressor = "heating";
    }
    root["compressor"] = compressor;
    root["setpoint"] = demo_setpoint_;
    root["current_temperature"] = demo_current_;
    root["has_ultima"] = climate_->has_ultima();
    root["demo"] = true;

    JsonArray zones = root["zones"].to<JsonArray>();
    for (int i = 1; i <= 8; i++) {
      JsonObject z = zones.add<JsonObject>();
      z["number"] = i;
      z["name"] = this->get_zone_display_name(i);
      z["on"] = demo_zone_on_[i - 1];
      z["damper"] = demo_zone_on_[i - 1] ? 1.0 : 0.0;
      z["control"] = demo_zone_control_[i - 1];
      if (climate_->has_ultima()) {
        z["setpoint"] = demo_zone_setpoint_[i - 1];
        z["current_temperature"] = demo_zone_current_[i - 1];
      }
    }
  } else {
    auto *c = controller();
    root["status_received_ms"] = c->statusLastReceivedTime;
    root["system_on"] = c->getSystemOn();
    root["mode"] = operating_mode_to_string(c->getOperatingMode());
    root["fan"] = fan_mode_to_string(c->getFanSpeed());
    root["fan_running"] = fan_mode_to_string(c->getRunningFanSpeed());
    root["continuous_fan"] = c->getContinuousFanMode();
    root["compressor"] = compressor_to_string(c->getCompressorMode());
    root["setpoint"] = c->getMasterSetpoint();
    root["current_temperature"] = c->getMasterCurrentTemperature();
    root["has_ultima"] = climate_->has_ultima();

    JsonArray zones = root["zones"].to<JsonArray>();
    for (int i = 1; i <= 8; i++) {
      JsonObject z = zones.add<JsonObject>();
      z["number"] = i;
      z["name"] = this->get_zone_display_name(i);
      z["on"] = c->getZoneOn(i);
      z["damper"] = c->getZoneDamperPosition(i);
      z["control"] = c->getControlZone(i);
      if (climate_->has_ultima()) {
        z["setpoint"] = c->getZoneSetpointTemperature(i);
        z["current_temperature"] = c->getZoneCurrentTemperature(i);
      }
    }
  }

  std::string out;
  serializeJson(doc, out);
  return out;
}

// ================= Handler =================

bool Actron485ApiHandler::canHandle(AsyncWebServerRequest *request) const {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  auto url = request->url_to(url_buf);
  // StringRef has no starts_with(); compare via std::string.
  std::string s(url);
  return s.rfind("/api/v1/", 0) == 0 || s == "/api/v1";
}

bool Actron485ApiHandler::authorized_(AsyncWebServerRequest *request) {
  const std::string &token = parent_->auth_token();
  if (token.empty()) return true;
  auto header = request->get_header("Authorization");
  if (!header.has_value()) return false;
  const std::string prefix = "Bearer ";
  const std::string &value = *header;
  if (value.rfind(prefix, 0) != 0) return false;
  return value.substr(prefix.size()) == token;
}

std::string Actron485ApiHandler::read_body_(AsyncWebServerRequest *request) {
  httpd_req_t *r = *request;
  size_t content_len = r->content_len;
  if (content_len == 0 || content_len > MAX_BODY_BYTES) return "";

  std::string body(content_len, '\0');
  size_t received = 0;
  while (received < content_len) {
    int n = httpd_req_recv(r, &body[received], content_len - received);
    if (n <= 0) return "";
    received += (size_t) n;
  }
  body.resize(received);
  return body;
}

void Actron485ApiHandler::add_cors_headers_(AsyncWebServerResponse *response) {
  // Access-Control-Allow-Origin: * is already added by web_server_base's
  // DefaultHeaders. We add the method/header list for preflight support.
  response->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  response->addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
}

// ESPHome's web_server_idf::AsyncWebServerRequest::init_response_ only
// recognises 200/404/409 and falls through to 500 for everything else
// (see esphome/components/web_server_idf/web_server_idf.cpp:272). We
// override the status directly on the underlying IDF handle so codes like
// 202 Accepted, 400 Bad Request, 401 Unauthorized, 204 No Content work.
static void override_status(AsyncWebServerRequest *request, int code) {
  httpd_req_t *r = *request;
  switch (code) {
    case 200: httpd_resp_set_status(r, "200 OK"); break;
    case 202: httpd_resp_set_status(r, "202 Accepted"); break;
    case 204: httpd_resp_set_status(r, "204 No Content"); break;
    case 400: httpd_resp_set_status(r, "400 Bad Request"); break;
    case 401: httpd_resp_set_status(r, "401 Unauthorized"); break;
    case 404: httpd_resp_set_status(r, "404 Not Found"); break;
    case 409: httpd_resp_set_status(r, "409 Conflict"); break;
    default:  httpd_resp_set_status(r, "500 Internal Server Error"); break;
  }
}

void Actron485ApiHandler::send_json_(AsyncWebServerRequest *request, int code, const std::string &body) {
  auto *response = request->beginResponse(code, "application/json", body);
  add_cors_headers_(response);
  override_status(request, code);
  request->send(response);
}

void Actron485ApiHandler::send_error_(AsyncWebServerRequest *request, int code, const char *message) {
  std::string body = "{\"error\":\"";
  body += message;
  body += "\"}";
  send_json_(request, code, body);
}

void Actron485ApiHandler::handleRequest(AsyncWebServerRequest *request) {
  if (request->method() == HTTP_OPTIONS) {
    auto *response = request->beginResponse(204, nullptr);
    add_cors_headers_(response);
    override_status(request, 204);
    request->send(response);
    return;
  }

  if (!authorized_(request)) {
    send_error_(request, 401, "unauthorized");
    return;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  auto url_ref = request->url_to(url_buf);
  std::string url(url_ref);
  http_method method = request->method();

  if (method == HTTP_GET && url == "/api/v1/info") {
    handle_info_(request);
    return;
  }
  if (method == HTTP_GET && url == "/api/v1/state") {
    handle_state_(request);
    return;
  }
  if (method == HTTP_GET && url == "/api/v1/diagnostics") {
    handle_diagnostics_(request);
    return;
  }

  if (method == HTTP_POST) {
    std::string body = read_body_(request);
    if (url == "/api/v1/power") { handle_power_(request, body); return; }
    if (url == "/api/v1/mode") { handle_mode_(request, body); return; }
    if (url == "/api/v1/fan") { handle_fan_(request, body); return; }
    if (url == "/api/v1/setpoint") { handle_setpoint_(request, body); return; }

    const std::string zones_prefix = "/api/v1/zones/";
    if (url.rfind(zones_prefix, 0) == 0) {
      // Accepts:
      //   /api/v1/zones/{n}              — on/off + setpoint (Ultima)
      //   /api/v1/zones/{n}/control      — claim/release wall-controller role
      //   /api/v1/zones/{n}/temperature  — inject external sensor reading
      const char *tail = url.c_str() + zones_prefix.size();
      int zone = atoi(tail);
      const char *slash = strchr(tail, '/');
      if (slash == nullptr) {
        handle_zone_(request, zone, body);
        return;
      }
      std::string subpath(slash + 1);
      if (subpath == "control") {
        handle_zone_control_(request, zone, body);
        return;
      }
      if (subpath == "temperature") {
        handle_zone_temperature_(request, zone, body);
        return;
      }
      if (subpath == "name") {
        handle_zone_name_(request, zone, body);
        return;
      }
      send_error_(request, 404, "not_found");
      return;
    }
  }

  send_error_(request, 404, "not_found");
}

void Actron485ApiHandler::handle_info_(AsyncWebServerRequest *request) {
  JsonDocument doc;
  auto root = doc.to<JsonObject>();
  root["device_name"] = App.get_name();
  root["api_version"] = "v1";
  root["esphome_version"] = ESPHOME_VERSION;
  root["build_time"] = App.get_compilation_time();
  root["has_ultima"] = parent_->climate()->has_ultima();
  root["zone_count"] = 8;
  root["uptime_ms"] = (unsigned long) millis();
  root["demo"] = parent_->demo_mode();
  // Static zone metadata — safe to return even when the RS485 bus is silent.
  JsonArray zones = root["zones"].to<JsonArray>();
  for (int i = 1; i <= 8; i++) {
    JsonObject z = zones.add<JsonObject>();
    z["number"] = i;
    z["name"] = parent_->get_zone_display_name(i);
  }

  std::string out;
  serializeJson(doc, out);
  send_json_(request, 200, out);
}

void Actron485ApiHandler::handle_state_(AsyncWebServerRequest *request) {
  if (!parent_->state_receiving_data()) {
    send_error_(request, 409, "rs485_not_receiving");
    return;
  }
  send_json_(request, 200, parent_->build_state_json());
}

void Actron485ApiHandler::handle_diagnostics_(AsyncWebServerRequest *request) {
  JsonDocument doc;
  auto root = doc.to<JsonObject>();
  root["demo"] = parent_->demo_mode();
  if (parent_->demo_mode()) {
    root["receiving_data"] = true;
    root["data_last_received_ms"] = (unsigned long) millis();
    root["data_last_sent_ms"] = (unsigned long) millis();
    root["status_last_received_ms"] = (unsigned long) millis();
    root["pending_commands"] = 0;
    root["pending_main_commands"] = 0;
  } else {
    auto *c = parent_->controller();
    root["receiving_data"] = c->receivingData();
    root["data_last_received_ms"] = c->dataLastReceivedTime;
    root["data_last_sent_ms"] = c->dataLastSentTime;
    root["status_last_received_ms"] = c->statusLastReceivedTime;
    root["pending_commands"] = c->totalPendingCommands();
    root["pending_main_commands"] = c->totalPendingMainCommands();
  }
  root["uptime_ms"] = (unsigned long) millis();

  std::string out;
  serializeJson(doc, out);
  send_json_(request, 200, out);
}

void Actron485ApiHandler::handle_power_(AsyncWebServerRequest *request, const std::string &body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (!doc["on"].is<bool>()) { send_error_(request, 400, "missing_on"); return; }
  parent_->apply_system_on(doc["on"].as<bool>());
  send_json_(request, 202, "{\"status\":\"queued\"}");
}

void Actron485ApiHandler::handle_mode_(AsyncWebServerRequest *request, const std::string &body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  const char *mode = doc["mode"] | (const char *) nullptr;
  Actron485::OperatingMode op;
  if (!string_to_operating_mode(mode, op)) { send_error_(request, 400, "invalid_mode"); return; }
  parent_->apply_operating_mode(op);
  send_json_(request, 202, "{\"status\":\"queued\"}");
}

void Actron485ApiHandler::handle_fan_(AsyncWebServerRequest *request, const std::string &body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  const char *speed = doc["speed"] | (const char *) nullptr;
  if (speed != nullptr) {
    Actron485::FanMode fan;
    if (!string_to_fan_mode(speed, fan)) { send_error_(request, 400, "invalid_speed"); return; }
    parent_->apply_fan_speed(fan);
  }
  if (doc["continuous"].is<bool>()) {
    parent_->apply_continuous_fan(doc["continuous"].as<bool>());
  }
  send_json_(request, 202, "{\"status\":\"queued\"}");
}

void Actron485ApiHandler::handle_setpoint_(AsyncWebServerRequest *request, const std::string &body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (!doc["temperature"].is<float>() && !doc["temperature"].is<double>() && !doc["temperature"].is<int>()) {
    send_error_(request, 400, "missing_temperature"); return;
  }
  double t = doc["temperature"].as<double>();
  if (t < 16.0 || t > 30.0) { send_error_(request, 400, "temperature_out_of_range"); return; }
  parent_->apply_master_setpoint(t);
  send_json_(request, 202, "{\"status\":\"queued\"}");
}

void Actron485ApiHandler::handle_zone_(AsyncWebServerRequest *request, int zone, const std::string &body) {
  if (zone < 1 || zone > 8) { send_error_(request, 400, "invalid_zone"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }

  if (doc["on"].is<bool>()) {
    parent_->apply_zone_on((uint8_t) zone, doc["on"].as<bool>());
  }
  if (doc["setpoint"].is<float>() || doc["setpoint"].is<double>() || doc["setpoint"].is<int>()) {
    if (!parent_->climate()->has_ultima()) {
      send_error_(request, 400, "setpoint_requires_ultima"); return;
    }
    double t = doc["setpoint"].as<double>();
    if (t < 16.0 || t > 30.0) { send_error_(request, 400, "setpoint_out_of_range"); return; }
    parent_->apply_zone_setpoint((uint8_t) zone, t);
  }
  send_json_(request, 202, "{\"status\":\"queued\"}");
}

// POST /api/v1/zones/{n}/control  {"enabled": true}
// Claims (true) or releases (false) the wall-controller role for the given
// zone on the RS485 bus. Required before injected temperature readings will
// be respected by the Actron master. Call once at startup per zone you plan
// to feed external sensor data for.
void Actron485ApiHandler::handle_zone_control_(AsyncWebServerRequest *request, int zone, const std::string &body) {
  if (zone < 1 || zone > 8) { send_error_(request, 400, "invalid_zone"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (!doc["enabled"].is<bool>()) { send_error_(request, 400, "missing_enabled"); return; }
  parent_->apply_zone_control((uint8_t) zone, doc["enabled"].as<bool>());
  send_json_(request, 202, "{\"status\":\"queued\"}");
}

// POST /api/v1/zones/{n}/temperature  {"current": 22.4}
// Injects a sensor reading for the zone on behalf of a remote ESP32 sensor.
// Only effective if /control was previously set enabled=true for this zone,
// otherwise the Actron master ignores our reading.
void Actron485ApiHandler::handle_zone_temperature_(AsyncWebServerRequest *request, int zone, const std::string &body) {
  if (zone < 1 || zone > 8) { send_error_(request, 400, "invalid_zone"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (!doc["current"].is<float>() && !doc["current"].is<double>() && !doc["current"].is<int>()) {
    send_error_(request, 400, "missing_current"); return;
  }
  double t = doc["current"].as<double>();
  // Wide bounds — not the AC's setpoint range; just sanity-checking the
  // reading is plausibly room temperature in Celsius.
  if (t < 0.0 || t > 60.0) { send_error_(request, 400, "temperature_out_of_range"); return; }
  bool controlled = parent_->demo_mode()
                       ? false  // demo: just accept the reading
                       : parent_->controller()->getControlZone((uint8_t) zone);
  if (!parent_->demo_mode() && !controlled) {
    ESP_LOGW(TAG, "Zone %d temperature set while not in control; call /control first", zone);
  }
  parent_->apply_zone_current_temperature((uint8_t) zone, t);
  parent_->note_zone_temperature_update((uint8_t) zone);
  send_json_(request, 202, "{\"status\":\"queued\"}");
}

// POST /api/v1/zones/{n}/name  {"name": "Living Room"}
// Persists a display-name override for the zone to flash (ESPHome
// preferences). The override is returned in /info and /state in place of
// the yaml-configured name. Pass an empty string to clear the override.
// Applies immediately (returns 200, not 202 — it's local state, not an
// RS485 command).
void Actron485ApiHandler::handle_zone_name_(AsyncWebServerRequest *request, int zone, const std::string &body) {
  if (zone < 1 || zone > 8) { send_error_(request, 400, "invalid_zone"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (!doc["name"].is<const char *>()) { send_error_(request, 400, "missing_name"); return; }
  std::string name = doc["name"].as<const char *>();
  if (!parent_->set_zone_name_override((uint8_t) zone, name)) {
    send_error_(request, 400, "name_too_long");
    return;
  }
  JsonDocument resp;
  resp["number"] = zone;
  resp["name"] = parent_->get_zone_display_name(zone);
  std::string body_out;
  serializeJson(resp, body_out);
  send_json_(request, 200, body_out);
}

}  // namespace actron485_api
}  // namespace esphome
