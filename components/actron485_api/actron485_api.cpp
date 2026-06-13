#include "actron485_api.h"

#include <cstdlib>
#include <cstring>

#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/actron485/utilities.h"
#include "Utilities.h"  // Actron485::printOut (the global log sink) for true-mute
#include "esphome/components/actron485/zone_fan.h"
#include "esphome/components/actron485/zone_climate.h"
#include "esphome/components/network/util.h"

#include <esp_http_server.h>
#include <esp_http_client.h>
#include <esp_crt_bundle.h>
#include <esp_system.h>
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
  this->load_settings_();
  this->load_timezone_();
  this->scheduler_.setup();
  ESP_LOGCONFIG(TAG, "Actron485 API mounted at /api/v1/* on port %u", base->get_port());

  // Weather proxy. The fetch is blocking (DNS + TLS + transfer), so it runs
  // on its own FreeRTOS task pinned to APP_CPU — keeping it off the ESPHome
  // main loop, which the actron485 climate component shares to service
  // time-critical slave-3 Modbus polls. The task only ever writes the cached
  // weather_* fields (under weather_mutex_); the HTTP handler only reads them.
  //
  // The task always starts: the api key + location are entered at runtime via
  // the dashboard (and may already be loaded from NVS above), so the task
  // idles until both are present rather than requiring a reboot to activate.
  weather_mutex_ = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(&Actron485Api::weather_task_trampoline_, "weather",
                          8192, this, 1, &weather_task_handle_, APP_CPU_NUM);
  ESP_LOGCONFIG(TAG, "Weather proxy task started (interval %u ms; configured=%s)",
                (unsigned) weather_update_interval_ms_,
                (!weather_api_key_.empty() && weather_location_set_) ? "yes" : "no (set key+location on dashboard)");
}

void Actron485Api::weather_task_trampoline_(void *arg) {
  static_cast<Actron485Api *>(arg)->weather_task_();
}

void Actron485Api::weather_task_() {
  for (;;) {
    bool configured = this->weather_api_key_set() && this->weather_location_set();
    if (configured && network::is_connected()) {
      // Snapshot config once for both fetches — the dashboard/API can mutate
      // the key + location from another task at any time.
      std::string key;
      float lat, lon;
      if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
      key = weather_api_key_;
      lat = weather_lat_;
      lon = weather_lon_;
      if (weather_mutex_) xSemaphoreGive(weather_mutex_);

      bool ok = this->fetch_weather_();
      // Forecast is best-effort and on its own provider path; a failure here
      // keeps the last-known-good forecast and never disturbs the current
      // reading that drives the retry cadence below.
      this->fetch_forecast_(key, lat, lon);
      // Full interval on success; quick 60s retry on failure so a transient
      // error, or a brand-new OpenWeather key that isn't active yet (returns
      // HTTP 401 for up to ~2h), recovers without a full 15-min wait.
      vTaskDelay(pdMS_TO_TICKS(ok ? weather_update_interval_ms_ : 60000));
    } else {
      // Not configured yet, or network not up — re-check soon so the first
      // reading lands shortly after the key/location is entered (or WiFi
      // associates) rather than a full interval later.
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

void Actron485Api::set_weather_error_(const std::string &msg) {
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  weather_error_ = msg;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
}

bool Actron485Api::fetch_weather_() {
  // Snapshot the runtime config under the mutex — the dashboard/API can
  // mutate the key + location from another task at any time.
  std::string key;
  float lat, lon;
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  key = weather_api_key_;
  lat = weather_lat_;
  lon = weather_lon_;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  if (key.empty()) return false;

  char url[256];
  snprintf(url, sizeof(url),
           "https://api.openweathermap.org/data/2.5/weather"
           "?lat=%.4f&lon=%.4f&units=metric&appid=%s",
           lat, lon, key.c_str());

  // ESP-IDF HTTP client. TLS is validated against the firmware's bundled CA
  // set (esp_crt_bundle) — no per-host cert to maintain. esp_http_client is
  // always available regardless of the arduino/esp-idf framework split.
  esp_http_client_config_t cfg = {};
  cfg.url = url;
  cfg.timeout_ms = 8000;
  cfg.crt_bundle_attach = esp_crt_bundle_attach;
  cfg.disable_auto_redirect = false;
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (client == nullptr) {
    ESP_LOGW(TAG, "weather: client init failed");
    this->set_weather_error_("internal error");
    return false;
  }

  esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "weather: connect failed: %s (keeping last-known-good)",
             esp_err_to_name(err));
    this->set_weather_error_(std::string("network/TLS error (") + esp_err_to_name(err) + ")");
    esp_http_client_cleanup(client);
    return false;
  }
  esp_http_client_fetch_headers(client);
  int status = esp_http_client_get_status_code(client);
  ESP_LOGI(TAG, "weather: HTTP %d", status);
  if (status != 200) {
    // 401 here almost always means a brand-new key that isn't active yet —
    // OpenWeather takes up to ~2h to activate. The task will retry in 60s.
    ESP_LOGW(TAG, "weather: HTTP %d (keeping last-known-good)", status);
    if (status == 401) {
      this->set_weather_error_("HTTP 401 - check API key (new keys take up to ~2h to activate)");
    } else {
      char m[40];
      snprintf(m, sizeof(m), "HTTP %d from weather API", status);
      this->set_weather_error_(m);
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return false;
  }

  std::string body;
  char buf[512];
  int r;
  while ((r = esp_http_client_read(client, buf, sizeof(buf))) > 0) {
    body.append(buf, (size_t) r);
    if (body.size() > 8192) break;  // safety cap — the payload is ~500 bytes
  }
  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  if (this->parse_weather_response_(body)) {
    ESP_LOGI(TAG, "weather: updated temp=%.1f cond=%s icon=%s",
             weather_temp_, weather_condition_.c_str(), weather_icon_.c_str());
    return true;
  }
  ESP_LOGW(TAG, "weather: response parse failed (keeping last-known-good)");
  this->set_weather_error_("unexpected response from weather API");
  return false;
}

bool Actron485Api::parse_weather_response_(const std::string &body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) return false;
  if (!doc["main"]["temp"].is<float>()) return false;

  float temp = doc["main"]["temp"].as<float>();
  float humidity = doc["main"]["humidity"].is<float>()
                       ? doc["main"]["humidity"].as<float>()
                       : NAN;
  const char *condition = "";
  const char *owm_icon = "";
  int owm_id = 0;
  JsonArray w = doc["weather"].as<JsonArray>();
  if (!w.isNull() && w.size() > 0) {
    JsonObject w0 = w[0];
    condition = w0["main"] | "";
    owm_icon = w0["icon"] | "";
    owm_id = w0["id"] | 0;
  }
  const char *glyph = map_weather_icon_(owm_icon, owm_id);

  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  weather_temp_ = temp;
  weather_humidity_ = humidity;
  weather_condition_ = condition;
  weather_icon_ = glyph;
  weather_updated_ms_ = millis();
  weather_available_ = true;
  weather_error_.clear();
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  return true;
}

const char *Actron485Api::map_weather_icon_(const char *owm_icon, int owm_id) {
  (void) owm_id;
  if (owm_icon == nullptr || strlen(owm_icon) < 2) return "cloudy";
  bool night = (strlen(owm_icon) >= 3 && owm_icon[2] == 'n');
  if (!strncmp(owm_icon, "01", 2)) return night ? "clear-night" : "clear-day";
  if (!strncmp(owm_icon, "02", 2)) return night ? "partly-cloudy-night" : "partly-cloudy-day";
  if (!strncmp(owm_icon, "03", 2)) return "cloudy";
  if (!strncmp(owm_icon, "04", 2)) return "cloudy";
  if (!strncmp(owm_icon, "09", 2)) return "rain";
  if (!strncmp(owm_icon, "10", 2)) return "rain";
  if (!strncmp(owm_icon, "11", 2)) return "storm";
  if (!strncmp(owm_icon, "13", 2)) return "snow";
  if (!strncmp(owm_icon, "50", 2)) return "fog";
  return "cloudy";
}

std::string Actron485Api::build_weather_json() {
  JsonDocument doc;
  auto root = doc.to<JsonObject>();

  bool configured, available;
  float temp, humidity;
  std::string condition, icon, error;
  unsigned long updated;
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  configured = !weather_api_key_.empty() && weather_location_set_;
  available = weather_available_;
  temp = weather_temp_;
  humidity = weather_humidity_;
  condition = weather_condition_;
  icon = weather_icon_;
  updated = weather_updated_ms_;
  error = weather_error_;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);

  if (!configured) {
    root["available"] = false;
    root["reason"] = "not_configured";
    std::string out;
    serializeJson(doc, out);
    return out;
  }

  if (!available) {
    root["available"] = false;
    root["reason"] = "no_data";  // configured, but no successful fetch yet
    if (!error.empty()) root["error"] = error;
    std::string out;
    serializeJson(doc, out);
    return out;
  }

  root["available"] = true;
  root["temp"] = temp;
  if (std::isfinite(humidity)) {
    root["humidity"] = humidity;
  } else {
    root["humidity"] = nullptr;
  }
  root["condition"] = condition;
  root["icon"] = icon;
  root["source"] = "openweather";
  root["updated_at_ms"] = updated;
  root["age_ms"] = (unsigned long) (millis() - updated);

  std::string out;
  serializeJson(doc, out);
  return out;
}

std::string Actron485Api::weather_status_summary() {
  bool configured, available;
  float temp;
  std::string condition, error;
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  configured = !weather_api_key_.empty() && weather_location_set_;
  available = weather_available_;
  temp = weather_temp_;
  condition = weather_condition_;
  error = weather_error_;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);

  if (!configured) return "Not configured";
  // Last-known-good wins even if a later poll failed; only surface the error
  // when we have nothing to show. Before the first attempt error is empty.
  if (!available) {
    if (!error.empty()) return "Error: " + error;
    return "Waiting for data\xE2\x80\xA6";  // …
  }

  char out[64];
  // "18.7°C · Partly cloudy" (° = U+00B0, · = U+00B7, both UTF-8 literals)
  snprintf(out, sizeof(out), "%.1f\xC2\xB0""C \xC2\xB7 %s", temp,
           condition.empty() ? "—" : condition.c_str());
  return std::string(out);
}

void Actron485Api::set_forecast_error_(const std::string &msg) {
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  forecast_error_ = msg;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
}

// Blocking HTTPS GET shared by the forecast fetchers. Returns true when the
// transport succeeded (any HTTP status — the caller inspects `status`); false
// only on connect/TLS failure (reason in *err_name). The body is always read
// so callers can inspect error payloads (e.g. the One Call subscription 401).
bool Actron485Api::http_get_(const char *url, std::string &body, int &status,
                             size_t cap, std::string *err_name) {
  body.clear();
  status = 0;
  esp_http_client_config_t cfg = {};
  cfg.url = url;
  cfg.timeout_ms = 8000;
  cfg.crt_bundle_attach = esp_crt_bundle_attach;
  cfg.disable_auto_redirect = false;
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (client == nullptr) {
    if (err_name) *err_name = "client init failed";
    return false;
  }
  esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    if (err_name) *err_name = esp_err_to_name(err);
    esp_http_client_cleanup(client);
    return false;
  }
  esp_http_client_fetch_headers(client);
  status = esp_http_client_get_status_code(client);
  char buf[512];
  int r;
  while ((r = esp_http_client_read(client, buf, sizeof(buf))) > 0) {
    body.append(buf, (size_t) r);
    if (body.size() > cap) break;  // safety cap — forecast payloads are ~20-35 KB
  }
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  return true;
}

// Builds the One Call 3.0 URL into `out`. exclude=minutely,alerts trims the
// payload to current + hourly(48) + daily(8) — the parts the wall renders.
static void build_onecall_url(char *out, size_t n, float lat, float lon,
                              const std::string &key) {
  snprintf(out, n,
           "https://api.openweathermap.org/data/3.0/onecall"
           "?lat=%.4f&lon=%.4f&units=metric&exclude=minutely,alerts&appid=%s",
           lat, lon, key.c_str());
}

bool Actron485Api::fetch_forecast_(const std::string &key, float lat, float lon) {
  if (key.empty()) return false;
  static const size_t kForecastCap = 49152;  // 48 KB — fits onecall + free

  WeatherSource src;
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  src = weather_source_;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);

  // Provider detection (only while UNKNOWN). Probe One Call 3.0 once: a 200
  // latches ONECALL and we parse the probe body directly (no second call); a
  // 401/403 whose body names the subscription latches FREE; any other failure
  // (invalid/inactive key, 429, 5xx) leaves the source UNKNOWN so it re-probes
  // next cycle — important so a brand-new key promotes to ONECALL once active.
  if (src == WeatherSource::UNKNOWN) {
    char url[256];
    build_onecall_url(url, sizeof(url), lat, lon, key);
    std::string body, errn;
    int status = 0;
    if (!http_get_(url, body, status, kForecastCap, &errn)) {
      ESP_LOGW(TAG, "forecast: onecall probe connect failed: %s", errn.c_str());
      this->set_forecast_error_(std::string("network/TLS error (") + errn + ")");
      return false;
    }
    ESP_LOGI(TAG, "forecast: onecall probe HTTP %d", status);
    if (status == 200) {
      if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
      weather_source_ = WeatherSource::ONECALL;
      if (weather_mutex_) xSemaphoreGive(weather_mutex_);
      ESP_LOGI(TAG, "forecast: provider = One Call 3.0");
      if (this->parse_onecall_response_(body)) return true;
      this->set_forecast_error_("unexpected One Call response");
      return false;
    }
    bool not_subscribed = (status == 401 || status == 403) &&
                          (body.find("subscri") != std::string::npos ||
                           body.find("One Call") != std::string::npos ||
                           body.find("one call") != std::string::npos);
    if (not_subscribed) {
      if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
      weather_source_ = WeatherSource::FREE;
      if (weather_mutex_) xSemaphoreGive(weather_mutex_);
      ESP_LOGI(TAG, "forecast: key not subscribed to One Call - using free 5-day/3-hour forecast");
    } else {
      ESP_LOGW(TAG, "forecast: onecall probe HTTP %d (not a subscription error; "
                    "will re-probe next cycle, trying free forecast for now)", status);
    }
    src = WeatherSource::FREE;  // try the free forecast this cycle either way
  }

  if (src == WeatherSource::ONECALL) {
    char url[256];
    build_onecall_url(url, sizeof(url), lat, lon, key);
    std::string body, errn;
    int status = 0;
    if (!http_get_(url, body, status, kForecastCap, &errn)) {
      this->set_forecast_error_(std::string("network/TLS error (") + errn + ")");
      return false;
    }
    if (status != 200) {
      ESP_LOGW(TAG, "forecast: onecall HTTP %d (keeping last-known-good)", status);
      char m[40];
      snprintf(m, sizeof(m), "HTTP %d from One Call", status);
      this->set_forecast_error_(m);
      return false;
    }
    if (this->parse_onecall_response_(body)) return true;
    this->set_forecast_error_("unexpected One Call response");
    return false;
  }

  // FREE tier: 5-day / 3-hour forecast.
  char url[256];
  snprintf(url, sizeof(url),
           "https://api.openweathermap.org/data/2.5/forecast"
           "?lat=%.4f&lon=%.4f&units=metric&appid=%s",
           lat, lon, key.c_str());
  std::string body, errn;
  int status = 0;
  if (!http_get_(url, body, status, kForecastCap, &errn)) {
    this->set_forecast_error_(std::string("network/TLS error (") + errn + ")");
    return false;
  }
  ESP_LOGI(TAG, "forecast: free HTTP %d", status);
  if (status != 200) {
    if (status == 401) {
      this->set_forecast_error_("HTTP 401 - check API key (new keys take up to ~2h to activate)");
    } else {
      char m[44];
      snprintf(m, sizeof(m), "HTTP %d from forecast API", status);
      this->set_forecast_error_(m);
    }
    return false;
  }
  if (this->parse_free_forecast_response_(body)) return true;
  this->set_forecast_error_("unexpected forecast response");
  return false;
}

bool Actron485Api::parse_onecall_response_(const std::string &body) {
  // Filter: only deserialize the fields we keep, so the ~30 KB payload never
  // becomes a full in-RAM DOM. [0] in a filter applies to every array element.
  JsonDocument filter;
  filter["hourly"][0]["dt"] = true;
  filter["hourly"][0]["temp"] = true;
  filter["hourly"][0]["pop"] = true;
  filter["hourly"][0]["weather"][0]["id"] = true;
  filter["hourly"][0]["weather"][0]["icon"] = true;
  filter["daily"][0]["dt"] = true;
  filter["daily"][0]["temp"]["min"] = true;
  filter["daily"][0]["temp"]["max"] = true;
  filter["daily"][0]["pop"] = true;
  filter["daily"][0]["weather"][0]["id"] = true;
  filter["daily"][0]["weather"][0]["icon"] = true;

  JsonDocument doc;
  DeserializationError err =
      deserializeJson(doc, body, DeserializationOption::Filter(filter));
  if (err) {
    ESP_LOGW(TAG, "forecast: onecall parse error: %s", err.c_str());
    return false;
  }

  ForecastHour hours[FORECAST_HOURS_MAX];
  int nh = 0;
  for (JsonObject h : doc["hourly"].as<JsonArray>()) {
    if (nh >= FORECAST_HOURS_MAX) break;
    hours[nh].dt = h["dt"] | 0;
    hours[nh].temp = h["temp"] | NAN;
    hours[nh].pop = (uint8_t) ((h["pop"] | 0.0f) * 100.0f + 0.5f);
    const char *icon = "";
    int id = 0;
    JsonArray w = h["weather"].as<JsonArray>();
    if (!w.isNull() && w.size() > 0) { icon = w[0]["icon"] | ""; id = w[0]["id"] | 0; }
    strncpy(hours[nh].icon, map_weather_icon_(icon, id), sizeof(hours[nh].icon) - 1);
    hours[nh].icon[sizeof(hours[nh].icon) - 1] = '\0';
    nh++;
  }

  ForecastDay days[FORECAST_DAYS_MAX];
  int nd = 0;
  for (JsonObject d : doc["daily"].as<JsonArray>()) {
    if (nd >= FORECAST_DAYS_MAX) break;
    days[nd].dt = d["dt"] | 0;
    days[nd].temp_min = d["temp"]["min"] | NAN;
    days[nd].temp_max = d["temp"]["max"] | NAN;
    days[nd].pop = (uint8_t) ((d["pop"] | 0.0f) * 100.0f + 0.5f);
    const char *icon = "";
    int id = 0;
    JsonArray w = d["weather"].as<JsonArray>();
    if (!w.isNull() && w.size() > 0) { icon = w[0]["icon"] | ""; id = w[0]["id"] | 0; }
    strncpy(days[nd].icon, map_weather_icon_(icon, id), sizeof(days[nd].icon) - 1);
    days[nd].icon[sizeof(days[nd].icon) - 1] = '\0';
    nd++;
  }

  if (nh == 0 && nd == 0) return false;

  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  if (nh) memcpy(forecast_hours_, hours, sizeof(ForecastHour) * nh);
  if (nd) memcpy(forecast_days_, days, sizeof(ForecastDay) * nd);
  forecast_hours_count_ = nh;
  forecast_days_count_ = nd;
  forecast_available_ = true;
  forecast_updated_ms_ = millis();
  forecast_error_.clear();
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  ESP_LOGI(TAG, "forecast: onecall ok (%d hours, %d days; body %u B, free heap %u)",
           nh, nd, (unsigned) body.size(), (unsigned) esp_get_free_heap_size());
  return true;
}

bool Actron485Api::parse_free_forecast_response_(const std::string &body) {
  JsonDocument filter;
  filter["list"][0]["dt"] = true;
  filter["list"][0]["main"]["temp"] = true;
  filter["list"][0]["main"]["temp_min"] = true;
  filter["list"][0]["main"]["temp_max"] = true;
  filter["list"][0]["pop"] = true;
  filter["list"][0]["weather"][0]["id"] = true;
  filter["list"][0]["weather"][0]["icon"] = true;
  filter["city"]["timezone"] = true;

  JsonDocument doc;
  DeserializationError err =
      deserializeJson(doc, body, DeserializationOption::Filter(filter));
  if (err) {
    ESP_LOGW(TAG, "forecast: free parse error: %s", err.c_str());
    return false;
  }
  JsonArray list = doc["list"].as<JsonArray>();
  if (list.isNull() || list.size() == 0) return false;

  // Local-day offset (seconds) for grouping the 3-hour blocks into calendar
  // days at the install location.
  long tz = (long) (doc["city"]["timezone"] | 0);

  // Hourly strip: the first N 3-hour blocks, as-is.
  ForecastHour hours[FORECAST_HOURS_MAX];
  int nh = 0;
  for (JsonObject e : list) {
    if (nh >= FORECAST_HOURS_MAX) break;
    hours[nh].dt = e["dt"] | 0;
    hours[nh].temp = e["main"]["temp"] | NAN;
    hours[nh].pop = (uint8_t) ((e["pop"] | 0.0f) * 100.0f + 0.5f);
    const char *icon = "";
    int id = 0;
    JsonArray w = e["weather"].as<JsonArray>();
    if (!w.isNull() && w.size() > 0) { icon = w[0]["icon"] | ""; id = w[0]["id"] | 0; }
    strncpy(hours[nh].icon, map_weather_icon_(icon, id), sizeof(hours[nh].icon) - 1);
    hours[nh].icon[sizeof(hours[nh].icon) - 1] = '\0';
    nh++;
  }

  // Daily outlook: fold the 3-hour blocks into per-day min/max + max pop, with
  // the icon from the block nearest local noon (most representative of the day).
  struct DayAcc {
    long day;
    float tmin, tmax;
    uint8_t pop;
    int32_t dt;
    int best_noon;
    char icon[20];
  };
  DayAcc acc[FORECAST_DAYS_MAX];
  int na = 0;
  for (JsonObject e : list) {
    int32_t dt = e["dt"] | 0;
    long local = (long) dt + tz;
    long localDay = local / 86400;
    int localHour = (int) ((local % 86400) / 3600);
    float t = e["main"]["temp"] | NAN;
    float tmin = e["main"]["temp_min"] | t;
    float tmax = e["main"]["temp_max"] | t;
    uint8_t pop = (uint8_t) ((e["pop"] | 0.0f) * 100.0f + 0.5f);
    const char *icon = "";
    int id = 0;
    JsonArray w = e["weather"].as<JsonArray>();
    if (!w.isNull() && w.size() > 0) { icon = w[0]["icon"] | ""; id = w[0]["id"] | 0; }

    int idx = -1;
    for (int i = 0; i < na; i++) {
      if (acc[i].day == localDay) { idx = i; break; }
    }
    if (idx < 0) {
      if (na >= FORECAST_DAYS_MAX) break;
      idx = na++;
      acc[idx].day = localDay;
      acc[idx].tmin = tmin;
      acc[idx].tmax = tmax;
      acc[idx].pop = pop;
      acc[idx].dt = dt;
      acc[idx].best_noon = 99;
      acc[idx].icon[0] = '\0';
    } else {
      if (std::isfinite(tmin) && (!std::isfinite(acc[idx].tmin) || tmin < acc[idx].tmin)) acc[idx].tmin = tmin;
      if (std::isfinite(tmax) && (!std::isfinite(acc[idx].tmax) || tmax > acc[idx].tmax)) acc[idx].tmax = tmax;
      if (pop > acc[idx].pop) acc[idx].pop = pop;
    }
    int noon = abs(localHour - 12);
    if (noon < acc[idx].best_noon) {
      acc[idx].best_noon = noon;
      strncpy(acc[idx].icon, map_weather_icon_(icon, id), sizeof(acc[idx].icon) - 1);
      acc[idx].icon[sizeof(acc[idx].icon) - 1] = '\0';
    }
  }

  ForecastDay days[FORECAST_DAYS_MAX];
  for (int i = 0; i < na; i++) {
    days[i].dt = acc[i].dt;
    days[i].temp_min = acc[i].tmin;
    days[i].temp_max = acc[i].tmax;
    days[i].pop = acc[i].pop;
    strncpy(days[i].icon, acc[i].icon, sizeof(days[i].icon) - 1);
    days[i].icon[sizeof(days[i].icon) - 1] = '\0';
  }

  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  if (nh) memcpy(forecast_hours_, hours, sizeof(ForecastHour) * nh);
  if (na) memcpy(forecast_days_, days, sizeof(ForecastDay) * na);
  forecast_hours_count_ = nh;
  forecast_days_count_ = na;
  forecast_available_ = true;
  forecast_updated_ms_ = millis();
  forecast_error_.clear();
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  ESP_LOGI(TAG, "forecast: free ok (%d blocks, %d days; body %u B, free heap %u)",
           nh, na, (unsigned) body.size(), (unsigned) esp_get_free_heap_size());
  return true;
}

std::string Actron485Api::build_forecast_json_(bool include_hourly, bool include_daily) {
  JsonDocument doc;
  auto root = doc.to<JsonObject>();

  bool configured, available;
  WeatherSource src;
  unsigned long updated;
  std::string error;
  ForecastHour hours[FORECAST_HOURS_MAX];
  ForecastDay days[FORECAST_DAYS_MAX];
  int nh, nd;
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  configured = !weather_api_key_.empty() && weather_location_set_;
  available = forecast_available_;
  src = weather_source_;
  updated = forecast_updated_ms_;
  error = forecast_error_;
  nh = forecast_hours_count_;
  nd = forecast_days_count_;
  if (nh) memcpy(hours, forecast_hours_, sizeof(ForecastHour) * nh);
  if (nd) memcpy(days, forecast_days_, sizeof(ForecastDay) * nd);
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);

  if (!configured) {
    root["available"] = false;
    root["reason"] = "not_configured";
    std::string out;
    serializeJson(doc, out);
    return out;
  }
  if (!available) {
    root["available"] = false;
    root["reason"] = "no_data";
    if (!error.empty()) root["error"] = error;
    std::string out;
    serializeJson(doc, out);
    return out;
  }

  root["available"] = true;
  root["source"] = (src == WeatherSource::ONECALL) ? "onecall" : "openweather_free";
  root["updated_at_ms"] = updated;
  root["age_ms"] = (unsigned long) (millis() - updated);

  if (include_daily && nd > 0) {
    JsonObject today = root["today"].to<JsonObject>();
    today["dt"] = days[0].dt;
    if (std::isfinite(days[0].temp_min)) today["temp_min"] = days[0].temp_min; else today["temp_min"] = nullptr;
    if (std::isfinite(days[0].temp_max)) today["temp_max"] = days[0].temp_max; else today["temp_max"] = nullptr;
    today["pop"] = days[0].pop;
    today["icon"] = days[0].icon;
  }

  if (include_hourly) {
    JsonArray ha = root["hourly"].to<JsonArray>();
    for (int i = 0; i < nh; i++) {
      JsonObject o = ha.add<JsonObject>();
      o["dt"] = hours[i].dt;
      if (std::isfinite(hours[i].temp)) o["temp"] = hours[i].temp; else o["temp"] = nullptr;
      o["pop"] = hours[i].pop;
      o["icon"] = hours[i].icon;
    }
  }

  if (include_daily) {
    JsonArray da = root["daily"].to<JsonArray>();
    for (int i = 0; i < nd; i++) {
      JsonObject o = da.add<JsonObject>();
      o["dt"] = days[i].dt;
      if (std::isfinite(days[i].temp_min)) o["temp_min"] = days[i].temp_min; else o["temp_min"] = nullptr;
      if (std::isfinite(days[i].temp_max)) o["temp_max"] = days[i].temp_max; else o["temp_max"] = nullptr;
      o["pop"] = days[i].pop;
      o["icon"] = days[i].icon;
    }
  }

  std::string out;
  serializeJson(doc, out);
  return out;
}

std::string Actron485Api::build_forecast_json() { return build_forecast_json_(true, true); }
std::string Actron485Api::build_forecast_hourly_json() { return build_forecast_json_(true, false); }
std::string Actron485Api::build_forecast_daily_json() { return build_forecast_json_(false, true); }

std::string Actron485Api::forecast_status_summary() {
  bool configured, available;
  WeatherSource src;
  float tmin, tmax;
  int nd;
  std::string error;
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  configured = !weather_api_key_.empty() && weather_location_set_;
  available = forecast_available_;
  src = weather_source_;
  nd = forecast_days_count_;
  tmin = nd > 0 ? forecast_days_[0].temp_min : NAN;
  tmax = nd > 0 ? forecast_days_[0].temp_max : NAN;
  error = forecast_error_;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);

  if (!configured) return "Not configured";
  if (!available) {
    if (!error.empty()) return "Error: " + error;
    return "Waiting for data\xE2\x80\xA6";  // …
  }

  const char *srcname = (src == WeatherSource::ONECALL) ? "onecall" : "free";
  char out[64];
  if (std::isfinite(tmin) && std::isfinite(tmax)) {
    // "Today 11.2–19.8°C · onecall" (– = U+2013, ° = U+00B0, · = U+00B7)
    snprintf(out, sizeof(out), "Today %.1f\xE2\x80\x93%.1f\xC2\xB0""C \xC2\xB7 %s",
             tmin, tmax, srcname);
  } else {
    snprintf(out, sizeof(out), "%d-day outlook \xC2\xB7 %s", nd, srcname);
  }
  return std::string(out);
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

void Actron485Api::load_settings_() {
  uint32_t hash = fnv1_hash(std::string("actron485_api_settings_v2"));
  settings_pref_ = global_preferences->make_preference<SettingsBlob>(hash);
  settings_loaded_ = true;

  // Seed the settings cache from the current live state so GET /settings
  // reflects the actual yaml-time config before any NVS override is
  // applied. Without this, the cache shows the in-class defaults
  // (false / 1) until the user issues a PATCH — which is misleading.
  if (climate_ && climate_->get_controller()) {
    auto *c = climate_->get_controller();
    settings_act_as_slave_3_ = c->getSlaveResponderEnabled() && c->getSlaveResponderId() == 3;
    settings_logging_mode_ = (int) c->printOutMode;
  }

  SettingsBlob blob{};
  if (!settings_pref_.load(&blob)) {
    ESP_LOGCONFIG(TAG, "  No saved runtime settings — using yaml defaults "
                       "(act_as_slave_3=%s log_mode=%d)",
                  settings_act_as_slave_3_ ? "on" : "off", settings_logging_mode_);
    return;
  }
  if (blob.magic != kSettingsMagic || blob.version != kSettingsVersion) {
    ESP_LOGW(TAG, "  Saved settings have wrong magic/version (0x%08X v%u); discarding",
             blob.magic, blob.version);
    return;
  }
  // Apply persisted overrides on top of yaml-time defaults.
  if (blob.has_api_key) {
    blob.api_key[API_KEY_MAX] = '\0';  // defensive
    auth_token_ = std::string(blob.api_key);
    ESP_LOGCONFIG(TAG, "  Loaded API key from NVS (override yaml)");
  }
  if (blob.has_weather_key) {
    blob.weather_api_key[WEATHER_KEY_MAX] = '\0';  // defensive
    weather_api_key_ = std::string(blob.weather_api_key);
  }
  if (blob.weather_location_set) {
    weather_lat_ = blob.weather_lat;
    weather_lon_ = blob.weather_lon;
    weather_location_set_ = true;
  }
  ESP_LOGCONFIG(TAG, "  Weather: key=%s location=%s",
                blob.has_weather_key ? "set" : "unset",
                blob.weather_location_set ? "set" : "unset");
  settings_act_as_slave_3_ = blob.act_as_slave_3 != 0;
  settings_logging_mode_   = blob.logging_mode;
  // Push slave-responder into the live controller now. The logging mode is
  // applied later from loop() (apply_logging_mode_) — it touches the log sink,
  // which the climate component only attaches at its own setup(), so applying
  // it here could race the sink (re)attachment.
  if (climate_) {
    auto *c = climate_->get_controller();
    if (c) {
      c->setSlaveResponderMode(3, settings_act_as_slave_3_);
    }
  }
  ESP_LOGCONFIG(TAG, "  Loaded settings: act_as_slave_3=%s log_mode=%d api_key=%s",
                settings_act_as_slave_3_ ? "on" : "off", settings_logging_mode_,
                blob.has_api_key ? "set" : "unset");
}

void Actron485Api::save_settings_() {
  if (!settings_loaded_) return;
  SettingsBlob blob{};
  blob.magic = kSettingsMagic;
  blob.version = kSettingsVersion;
  blob.act_as_slave_3 = settings_act_as_slave_3_ ? 1 : 0;
  blob.logging_mode = (uint8_t) settings_logging_mode_;
  blob.has_api_key = auth_token_.empty() ? 0 : 1;
  size_t n = std::min(auth_token_.size(), API_KEY_MAX);
  memcpy(blob.api_key, auth_token_.data(), n);
  blob.api_key[n] = '\0';
  blob.has_weather_key = weather_api_key_.empty() ? 0 : 1;
  size_t wn = std::min(weather_api_key_.size(), WEATHER_KEY_MAX);
  memcpy(blob.weather_api_key, weather_api_key_.data(), wn);
  blob.weather_api_key[wn] = '\0';
  blob.weather_location_set = weather_location_set_ ? 1 : 0;
  blob.weather_lat = weather_lat_;
  blob.weather_lon = weather_lon_;
  settings_pref_.save(&blob);
  global_preferences->sync();
}

void Actron485Api::load_timezone_() {
  // Own NVS slot, independent of the versioned SettingsBlob, so editing the
  // timezone never orphans the api_key / weather config.
  uint32_t hash = fnv1_hash(std::string("actron485_api_timezone_v1"));
  timezone_pref_ = global_preferences->make_preference<TimezoneBlob>(hash);
  TimezoneBlob blob{};
  if (timezone_pref_.load(&blob)) {
    blob.tz[TIMEZONE_MAX] = '\0';  // defensive
    if (blob.tz[0] != '\0') {
      timezone_ = std::string(blob.tz);
    }
  }
  ESP_LOGCONFIG(TAG, "  Timezone: %s", timezone_.c_str());
}

void Actron485Api::save_timezone_() {
  TimezoneBlob blob{};
  size_t n = std::min(timezone_.size(), TIMEZONE_MAX);
  memcpy(blob.tz, timezone_.data(), n);
  blob.tz[n] = '\0';
  timezone_pref_.save(&blob);
  global_preferences->sync();
}

std::string Actron485Api::timezone() {
  return timezone_;
}

void Actron485Api::set_timezone_runtime(const std::string &tz) {
  std::string trimmed = tz;
  const char *ws = " \t\r\n";
  size_t b = trimmed.find_first_not_of(ws);
  size_t e = trimmed.find_last_not_of(ws);
  trimmed = (b == std::string::npos) ? "" : trimmed.substr(b, e - b + 1);
  if (trimmed.empty()) {
    return;  // never store an empty TZ — keep the last good value
  }
  timezone_ = trimmed.substr(0, TIMEZONE_MAX);
  this->save_timezone_();
  ESP_LOGI(TAG, "Timezone set to %s", timezone_.c_str());
}

// Friendly Australian-region dropdown over the POSIX TZ string. Keyed by
// distinct offset/DST behaviour (not city), so each label maps 1:1 to a
// unique POSIX string and the reverse lookup is unambiguous. Australia only —
// there are no Actron Que installs outside AU.
namespace {
struct TzOption {
  const char *label;
  const char *posix;
};
const TzOption kTzOptions[] = {
  {"NSW/VIC/ACT/TAS (Eastern, DST)",       "AEST-10AEDT,M10.1.0,M4.1.0/3"},
  {"Queensland (Eastern, no DST)",         "AEST-10"},
  {"South Australia (Central, DST)",       "ACST-9:30ACDT,M10.1.0,M4.1.0/3"},
  {"Northern Territory (Central, no DST)", "ACST-9:30"},
  {"Western Australia (no DST)",           "AWST-8"},
};
}  // namespace

std::string Actron485Api::timezone_label() {
  std::string tz = this->timezone();
  for (const auto &o : kTzOptions) {
    if (tz == o.posix) return o.label;
  }
  return "";  // custom POSIX string — not one of the presets
}

bool Actron485Api::set_timezone_by_label(const std::string &label) {
  for (const auto &o : kTzOptions) {
    if (label == o.label) {
      this->set_timezone_runtime(o.posix);
      return true;
    }
  }
  ESP_LOGW(TAG, "Unknown timezone label: %s", label.c_str());
  return false;
}

void Actron485Api::set_api_key_runtime(const std::string &key) {
  auth_token_ = key;
  this->save_settings_();
  ESP_LOGI(TAG, "API key %s", key.empty() ? "cleared (auth disabled)" : "updated");
}

void Actron485Api::set_act_as_slave_3_runtime(bool on) {
  settings_act_as_slave_3_ = on;
  if (climate_ && climate_->get_controller()) {
    climate_->get_controller()->setSlaveResponderMode(3, on);
  }
  this->save_settings_();
  ESP_LOGW(TAG, "act_as_slave_3 runtime-toggled %s — wall LCD data leads MUST be "
                "physically disconnected when ON; collision risk if both are live",
           on ? "ON" : "OFF");
}

void Actron485Api::apply_logging_mode_() {
  auto *c = climate_ ? climate_->get_controller() : nullptr;
  if (c == nullptr) return;

  // Capture the real sink the climate component attached at its setup(), so a
  // later un-mute can restore it. Lazy + once: by the time loop() first runs
  // every setup() has completed, so Actron485::printOut is the live sink.
  if (saved_log_sink_ == nullptr && Actron485::printOut != nullptr) {
    saved_log_sink_ = Actron485::printOut;
  }

  if (settings_logging_mode_ <= 0) {
    // NONE — true off. The library guards every print on the sink pointer
    // (many lines ignore printOutMode), so detaching the sink is the only way
    // to fully silence it.
    c->configureLogging(nullptr);
    return;
  }

  // Re-attach the sink (in case we were muted), then set verbosity. Settings
  // ints are 1=STATUS..5=DELTA; the enum is 0-based, hence the offset mapping.
  if (saved_log_sink_ != nullptr) c->configureLogging(saved_log_sink_);
  switch (settings_logging_mode_) {
    case 1: c->printOutMode = Actron485::PrintOutMode::StatusOnly; break;
    case 2: c->printOutMode = Actron485::PrintOutMode::ChangedMessages; break;
    case 3: c->printOutMode = Actron485::PrintOutMode::AllMessages; break;
    case 4: c->printOutMode = Actron485::PrintOutMode::CorrelationCapture; break;
    case 5: c->printOutMode = Actron485::PrintOutMode::RegisterDelta; break;
    default: c->printOutMode = Actron485::PrintOutMode::ChangedMessages; break;
  }
}

void Actron485Api::set_logging_mode_runtime(int mode) {
  if (mode < 0 || mode > 5) return;
  settings_logging_mode_ = mode;
  this->apply_logging_mode_();  // runtime — all components are up
  this->save_settings_();
}

// ---- Weather runtime config (dashboard + PATCH /api/v1/settings) ----
// All four fields are read by the weather task on another core, so config
// writes/reads are guarded by weather_mutex_. save_settings_() is called
// after releasing the lock (it only reads, on this task).

void Actron485Api::set_weather_api_key_runtime(const std::string &key) {
  // Trim surrounding whitespace — pasting into the dashboard field often
  // drags a trailing space/newline, which would corrupt the appid and 401.
  std::string trimmed = key;
  const char *ws = " \t\r\n";
  size_t b = trimmed.find_first_not_of(ws);
  size_t e = trimmed.find_last_not_of(ws);
  trimmed = (b == std::string::npos) ? "" : trimmed.substr(b, e - b + 1);
  trimmed = trimmed.substr(0, WEATHER_KEY_MAX);
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  weather_api_key_ = trimmed;
  // Re-detect the provider on the next forecast cycle — a new key may belong
  // to a different OpenWeather plan (One Call 3.0 vs free tier).
  weather_source_ = WeatherSource::UNKNOWN;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  this->save_settings_();
  ESP_LOGI(TAG, "Weather API key %s", trimmed.empty() ? "cleared" : "updated");
}

void Actron485Api::set_weather_location_runtime(float lat, float lon) {
  bool valid = std::isfinite(lat) && std::isfinite(lon) &&
               lat >= -90.0f && lat <= 90.0f &&
               lon >= -180.0f && lon <= 180.0f &&
               !(lat == 0.0f && lon == 0.0f);
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  if (valid) {
    weather_lat_ = lat;
    weather_lon_ = lon;
    weather_location_set_ = true;
  } else {
    weather_location_set_ = false;
  }
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  this->save_settings_();
  if (valid) {
    ESP_LOGI(TAG, "Weather location set lat=%.4f lon=%.4f", lat, lon);
  } else {
    ESP_LOGW(TAG, "Weather location cleared (lat=%.4f lon=%.4f out of range)", lat, lon);
  }
}

bool Actron485Api::weather_api_key_set() {
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  bool r = !weather_api_key_.empty();
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  return r;
}

bool Actron485Api::weather_location_set() {
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  bool r = weather_location_set_;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  return r;
}

float Actron485Api::weather_latitude() {
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  float r = weather_lat_;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  return r;
}

float Actron485Api::weather_longitude() {
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  float r = weather_lon_;
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  return r;
}

std::string Actron485Api::build_settings_json() {
  JsonDocument doc;
  auto root = doc.to<JsonObject>();
  root["act_as_slave_3"] = settings_act_as_slave_3_;
  root["logging_mode"] = settings_logging_mode_;
  // api_key is masked in the response — never echo back the real value.
  // Clients that need to verify they hold the right key should just
  // attempt an authenticated request and check for a 200.
  root["api_key_set"] = !auth_token_.empty();
  root["timezone"] = timezone_;
  // Weather config (key masked, like api_key). Lat/lon are echoed so the
  // dashboard/app can show the configured location.
  root["weather_api_key_set"] = this->weather_api_key_set();
  if (this->weather_location_set()) {
    root["weather_latitude"] = this->weather_latitude();
    root["weather_longitude"] = this->weather_longitude();
  } else {
    root["weather_latitude"] = nullptr;
    root["weather_longitude"] = nullptr;
  }
  std::string out;
  serializeJson(doc, out);
  return out;
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
  // Note: this updates the API-layer override and the Zone N Name text
  // input on the dashboard, but the climate widget's "Zone N" label is
  // baked at compile-time (EntityBase::name_ is a StringRef into static
  // storage, no set_name() exists). Propagating the override to the
  // climate widget label would need a per-entity name-override store +
  // a virtual get_name() override — out of scope for this commit. For
  // now the rename surfaces in the JSON API (/api/v1/info[zones], /state),
  // in the rename input itself, and in the mobile app — but not on the
  // climate card label.
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

// In demo mode the apply_* writers update only demo_* state (httpd
// thread). They deliberately DO NOT call climate_->publish_state() or
// otherwise poke ESPHome entities here — that has to happen on the main
// loop thread. demo_tick() runs at 1 Hz, picks up demo_* changes, syncs
// them into the Climate/fan entities and publishes. Calling publish_state
// from the httpd task serialized rapid-fire writes, which made the httpd
// task start returning 503s and eventually triggered the task watchdog.
void Actron485Api::apply_system_on(bool on) {
  if (demo_mode_) { demo_system_on_ = on; return; }
  controller()->setSystemOn(on);
}
void Actron485Api::apply_operating_mode(Actron485::OperatingMode mode) {
  if (demo_mode_) {
    demo_op_mode_ = mode;
    demo_system_on_ = (mode != Actron485::OperatingMode::Off &&
                        mode != Actron485::OperatingMode::OffAuto &&
                        mode != Actron485::OperatingMode::OffCool &&
                        mode != Actron485::OperatingMode::OffHeat);
    return;
  }
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
void Actron485Api::apply_quiet_mode(bool on) {
  // Demo mode doesn't simulate quiet (the demo simulator only cares about
  // mode/setpoint/fan); silently no-op so the API call still succeeds.
  if (demo_mode_) { return; }
  controller()->setQuietMode(on);
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
  if (demo_mode_) {
    // Match the Que master's clamp to master-setpoint ±2 °C window, so
    // demo state behaves like the real system rather than permitting
    // arbitrary per-zone values. The real firmware path doesn't need
    // this — the Actron master does the clamping itself.
    const float lo = demo_setpoint_ - 2.0f;
    const float hi = demo_setpoint_ + 2.0f;
    float clamped = (float) temperature;
    if (clamped < lo) clamped = lo;
    if (clamped > hi) clamped = hi;
    demo_zone_setpoint_[zone - 1] = clamped;
    return;
  }
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

// Drift zone currents toward their setpoints and publish the simulator's
// state. The Climate/fan entities are authoritative for user-visible
// settings (setpoint, mode, fan, zone on/off) — we READ them at the start
// of each tick so web-UI or Home-Assistant changes stick, and only WRITE
// back the fields we simulate (current_temperature, action).
void Actron485Api::demo_tick_() {
  using esphome::actron485::Converter;
  namespace clm = esphome::climate;

  unsigned long now = millis();
  if (demo_last_tick_ms_ == 0) { demo_last_tick_ms_ = now; return; }
  float dt = (now - demo_last_tick_ms_) / 1000.0f;
  demo_last_tick_ms_ = now;
  if (dt <= 0) return;

  // 1) Merge external (web UI / HA) changes into demo_* state. If a
  //    climate_ field differs from what we last pushed, something else
  //    moved it — adopt it. Otherwise demo_* is authoritative (possibly
  //    just written by an API POST on the httpd thread).
  if (!std::isnan(climate_->target_temperature) &&
      climate_->target_temperature != demo_last_pub_setpoint_) {
    demo_setpoint_ = climate_->target_temperature;
  }
  int current_mode_int = (int) climate_->mode;
  if (current_mode_int != demo_last_pub_mode_) {
    if (climate_->mode == clm::CLIMATE_MODE_OFF) {
      demo_system_on_ = false;
    } else {
      demo_system_on_ = true;
      demo_op_mode_ = Converter::to_actron_operating_mode(climate_->mode);
    }
  }
  int current_fan_int = climate_->fan_mode.has_value() ? (int) *climate_->fan_mode : -1;
  if (current_fan_int != demo_last_pub_fan_ && climate_->fan_mode.has_value()) {
    demo_fan_ = Converter::to_actron_fan_mode(*climate_->fan_mode);
  }

  for (int i = 1; i <= 8; i++) {
    if (auto *zf = climate_->get_zone_fan(i)) {
      if (zf->state != demo_last_pub_zone_on_[i - 1]) {
        demo_zone_on_[i - 1] = zf->state;
      }
    }
    if (auto *zc = climate_->get_zone_climate(i)) {
      if (!std::isnan(zc->target_temperature) &&
          zc->target_temperature != demo_last_pub_zone_setpoint_[i - 1]) {
        demo_zone_setpoint_[i - 1] = zc->target_temperature;
      }
    }
  }

  // 1b) Clamp every zone setpoint to the master-setpoint ±2 °C window.
  //     Mirrors the Que master's behavior: moving the master drags any
  //     zones that were outside the new window back inside it. In live
  //     mode the Actron master does this itself; we only need it here.
  const float zone_lo = demo_setpoint_ - 2.0f;
  const float zone_hi = demo_setpoint_ + 2.0f;
  for (int i = 0; i < 8; i++) {
    if (demo_zone_setpoint_[i] < zone_lo) demo_zone_setpoint_[i] = zone_lo;
    if (demo_zone_setpoint_[i] > zone_hi) demo_zone_setpoint_[i] = zone_hi;
  }

  // 2) Drift simulated currents toward per-zone targets.
  const float rate = 0.05f;  // °C per second
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

  // 3) Publish at most once per second — cheap UI refresh rate, avoids
  //    spamming the ESPHome subscription machinery. This is the ONLY
  //    place we write to climate_/fan/zone entities, and it runs on the
  //    main loop, so no thread races with httpd handlers.
  if (now - demo_last_publish_ms_ < 1000) return;
  demo_last_publish_ms_ = now;

  climate_->target_temperature = demo_setpoint_;
  climate_->mode = demo_system_on_ ? Converter::to_climate_mode(demo_op_mode_)
                                    : clm::CLIMATE_MODE_OFF;
  climate_->fan_mode = Converter::to_fan_mode(demo_fan_);
  climate_->current_temperature = demo_current_;
  Actron485::CompressorMode cmode = Actron485::CompressorMode::Idle;
  if (demo_system_on_) {
    if (demo_op_mode_ == Actron485::OperatingMode::Cool) cmode = Actron485::CompressorMode::Cooling;
    else if (demo_op_mode_ == Actron485::OperatingMode::Heat) cmode = Actron485::CompressorMode::Heating;
  }
  climate_->action = demo_system_on_
                        ? Converter::to_climate_action(cmode, demo_op_mode_)
                        : clm::CLIMATE_ACTION_OFF;
  climate_->publish_state();

  // Remember what we just wrote so next tick can detect external changes.
  demo_last_pub_setpoint_ = demo_setpoint_;
  demo_last_pub_mode_ = (int) climate_->mode;
  demo_last_pub_fan_ = climate_->fan_mode.has_value() ? (int) *climate_->fan_mode : -1;

  for (int i = 1; i <= 8; i++) {
    if (auto *zf = climate_->get_zone_fan(i)) {
      if (zf->state != demo_zone_on_[i - 1]) {
        zf->state = demo_zone_on_[i - 1];
        zf->publish_state();
      }
      demo_last_pub_zone_on_[i - 1] = zf->state;
    }
    if (auto *zc = climate_->get_zone_climate(i)) {
      zc->target_temperature = demo_zone_setpoint_[i - 1];
      zc->current_temperature = demo_zone_current_[i - 1];
      zc->publish_state();
      demo_last_pub_zone_setpoint_[i - 1] = demo_zone_setpoint_[i - 1];
    }
  }
}

void Actron485Api::note_zone_humidity_update(uint8_t zone, float humidity) {
  if (zone < 1 || zone > 8) return;
  zone_humidity_[zone - 1] = humidity;
  last_humidity_update_ms_[zone - 1] = millis();
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
  // Tick the scheduler first — it self-throttles to ~30 s and must run in demo
  // mode too (apply_* honour the simulator). Kept above the early-returns below.
  this->scheduler_.loop();

  // Apply the persisted logging mode once, here rather than in load_settings_,
  // so the climate component has already attached its log sink (avoids a
  // setup-order race when restoring/muting the sink).
  if (!logging_applied_) {
    logging_applied_ = true;
    this->apply_logging_mode_();
  }

  if (demo_mode_) {
    this->demo_tick_();
    return;  // nothing else to do — the real bus isn't in play
  }
  if (sensor_stale_timeout_ms_ == 0) return;
  unsigned long now = millis();
  // Check at most once per second — cheap but avoids per-tick overhead.
  if (now - last_stale_check_ms_ < 1000) return;
  last_stale_check_ms_ = now;

  // In slave-3 responder mode, "release wall-controller role" is
  // meaningless — WE are the wall controller, there's nothing to fall back
  // to. Dropping zoneControlled[] would also break setMasterSetpoint's
  // mapping to the control zone. So in that mode we just log the warning
  // and keep serving the last-known-good temperature; the AMIB sees a
  // stale-but-stable zone offset rather than a sudden NaN.
  bool responderMode = controller()->getSlaveResponderEnabled();

  for (int i = 0; i < 8; i++) {
    if (last_temp_update_ms_[i] == 0) continue;  // never fed
    if (now - last_temp_update_ms_[i] < sensor_stale_timeout_ms_) continue;
    uint8_t zone = (uint8_t) (i + 1);
    if (controller()->getControlZone(zone)) {
      if (responderMode) {
        ESP_LOGW(TAG, "Zone %u sensor stale (no POST for %lu ms); keeping "
                       "last-known-good temp because slave-3 responder is "
                       "active (cannot release role).",
                 zone, now - last_temp_update_ms_[i]);
      } else {
        ESP_LOGW(TAG, "Zone %u sensor stale (no POST for %lu ms); releasing "
                       "wall-controller role",
                 zone, now - last_temp_update_ms_[i]);
        controller()->setControlZone(zone, false);
      }
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
    root["quiet_mode"] = false;
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
    root["quiet_mode"] = c->getQuietMode();
    root["compressor"] = compressor_to_string(c->getCompressorMode());
    root["setpoint"] = c->getMasterSetpoint();
    root["current_temperature"] = c->getMasterCurrentTemperature();
    double outdoor = c->getOutdoorTemperature();
    if (std::isfinite(outdoor)) {
      root["outdoor_temperature"] = outdoor;
    }
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
      // Humidity is independent of Ultima — it's just an external
      // sensor relay through /api/v1/zones/{n}/humidity. Surface NaN
      // as null so clients can cleanly render "—".
      float h = this->zone_humidity((uint8_t) i);
      if (std::isfinite(h)) {
        z["humidity"] = h;
      } else {
        z["humidity"] = nullptr;
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

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  auto url_ref = request->url_to(url_buf);
  std::string url(url_ref);
  http_method method = request->method();

  // /api/v1/info is intentionally unauthenticated — it carries only
  // public discovery info (device name, api version, zone metadata,
  // build time) and is the endpoint sensors / mobile apps poll before
  // they hold a token. Everything else requires auth when api_key is
  // configured.
  bool auth_exempt = (method == HTTP_GET && url == "/api/v1/info");
  if (!auth_exempt && !authorized_(request)) {
    send_error_(request, 401, "unauthorized");
    return;
  }

  // Scheduler / away / time routes (self-contained, defined at end of file).
  if (this->handle_scheduler_routes_(request, method, url)) return;

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
  if (method == HTTP_GET && url == "/api/v1/weather") {
    handle_weather_(request);
    return;
  }
  if (method == HTTP_GET && url == "/api/v1/forecast") {
    handle_forecast_(request);
    return;
  }
  if (method == HTTP_GET && url == "/api/v1/forecast/hourly") {
    handle_forecast_hourly_(request);
    return;
  }
  if (method == HTTP_GET && url == "/api/v1/forecast/daily") {
    handle_forecast_daily_(request);
    return;
  }
  if (method == HTTP_GET && url == "/api/v1/bus") {
    handle_bus_(request);
    return;
  }
  if (method == HTTP_GET && url == "/api/v1/settings") {
    handle_settings_get_(request);
    return;
  }
  if (method == HTTP_POST && url == "/api/v1/settings") {
    std::string body = read_body_(request);
    handle_settings_patch_(request, body);
    return;
  }

  if (method == HTTP_POST) {
    std::string body = read_body_(request);
    if (url == "/api/v1/power") { handle_power_(request, body); return; }
    if (url == "/api/v1/mode") { handle_mode_(request, body); return; }
    if (url == "/api/v1/fan") { handle_fan_(request, body); return; }
    if (url == "/api/v1/quiet") { handle_quiet_(request, body); return; }
    if (url == "/api/v1/setpoint") { handle_setpoint_(request, body); return; }
    if (url == "/api/v1/demo") { handle_demo_(request, body); return; }

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
      if (subpath == "humidity") {
        handle_zone_humidity_(request, zone, body);
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
  root["timezone"] = parent_->timezone();
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

// POST /api/v1/demo  {"enabled": bool}
// Toggles the firmware's demo simulator at runtime. SESSION-ONLY — a reboot
// restores whatever the YAML was flashed with. Intended as a dev switch so
// the mobile app can flip between simulated and real state without
// reflashing. Returns {"demo": <new state>}.
void Actron485ApiHandler::handle_demo_(AsyncWebServerRequest *request, const std::string &body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (!doc["enabled"].is<bool>()) { send_error_(request, 400, "missing_enabled"); return; }
  bool enabled = doc["enabled"].as<bool>();
  if (enabled != parent_->demo_mode()) {
    if (enabled) {
      ESP_LOGW(TAG, "Switching to DEMO mode at runtime (session-only)");
    } else {
      ESP_LOGI(TAG, "Exiting DEMO mode at runtime");
    }
    parent_->set_demo_mode(enabled);
  }
  JsonDocument resp;
  resp["demo"] = parent_->demo_mode();
  std::string out;
  serializeJson(resp, out);
  send_json_(request, 200, out);
}

void Actron485ApiHandler::handle_state_(AsyncWebServerRequest *request) {
  if (!parent_->state_receiving_data()) {
    send_error_(request, 409, "rs485_not_receiving");
    return;
  }
  send_json_(request, 200, parent_->build_state_json());
}

// GET /api/v1/weather — last-known-good outdoor weather, polled server-side
// from OpenWeather. Always 200: {"available": false, ...} until configured
// and a first fetch succeeds (friendlier for the wall's fast poller than a
// 409). See build_weather_json for the shape.
void Actron485ApiHandler::handle_weather_(AsyncWebServerRequest *request) {
  send_json_(request, 200, parent_->build_weather_json());
}

// GET /api/v1/forecast[/hourly|/daily] — last-known-good forecast, polled
// server-side. Always 200; {"available": false, "reason": ...} until ready.
// See build_forecast_json[_hourly|_daily] for the shapes.
void Actron485ApiHandler::handle_forecast_(AsyncWebServerRequest *request) {
  send_json_(request, 200, parent_->build_forecast_json());
}
void Actron485ApiHandler::handle_forecast_hourly_(AsyncWebServerRequest *request) {
  send_json_(request, 200, parent_->build_forecast_hourly_json());
}
void Actron485ApiHandler::handle_forecast_daily_(AsyncWebServerRequest *request) {
  send_json_(request, 200, parent_->build_forecast_daily_json());
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

// GET /api/v1/bus — Modbus register cache + slave-3 responder snapshot.
// Cache is populated only when `logging_mode: DELTA` is set in actron.yaml,
// because that's the path that runs printModbusMessage → updateRegisterCache.
// In other logging modes the registers[] array will be empty. The responder
// section reflects the live state of our slave-3 register buffer when
// `act_as_slave_3: true` (independent of logging mode).
//
// Designed for debugging: lets you watch bus state via curl/jq without
// grepping the ESPHome log, and confirms what the AMIB sees from us when
// in slave-3 responder mode.
void Actron485ApiHandler::handle_bus_(AsyncWebServerRequest *request) {
  JsonDocument doc;
  auto root = doc.to<JsonObject>();
  auto *c = parent_->controller();
  unsigned long now = (unsigned long) millis();

  root["uptime_ms"] = now;
  root["receiving_data"] = parent_->demo_mode() ? true : c->receivingData();

  // Modbus register cache (sniffed bus traffic).
  JsonArray regs = root["registers"].to<JsonArray>();
  if (!parent_->demo_mode()) {
    char hexAddr[8];
    char hexVal[8];
    for (size_t i = 0; i < c->getRegisterCacheCount(); i++) {
      const auto &e = c->getRegisterCacheEntry(i);
      JsonObject r = regs.add<JsonObject>();
      r["slave"] = e.slave;
      snprintf(hexAddr, sizeof(hexAddr), "0x%04X", e.address);
      r["address"] = hexAddr;
      snprintf(hexVal, sizeof(hexVal), "0x%04X", e.value);
      r["value"] = hexVal;
      r["age_ms"] = (now >= e.lastSeenMs) ? (now - e.lastSeenMs) : 0;
    }
  }

  // Slave-responder mode state (what WE are publishing to the bus).
  JsonObject responder = root["responder"].to<JsonObject>();
  bool enabled = parent_->demo_mode() ? false : c->getSlaveResponderEnabled();
  responder["enabled"] = enabled;
  if (enabled) {
    responder["slave_id"] = c->getSlaveResponderId();
    JsonArray buffer = responder["buffer"].to<JsonArray>();
    // Only the live region of the buffer is worth exposing — everything
    // outside is unused inert RAM. Reg 0-12 (mode/fan/zones/setpoints),
    // 21-28 (compressor PWM from AMIB writes), 30-33 (bounds), 100 (alive).
    static const uint16_t kInterestingAddrs[] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
      21, 22, 23, 24, 25, 26, 27, 28,
      30, 31, 32, 33,
      100,
    };
    char hexVal[8];
    for (uint16_t addr : kInterestingAddrs) {
      uint16_t v = c->getSlaveRegister(addr);
      JsonObject r = buffer.add<JsonObject>();
      r["address"] = addr;
      snprintf(hexVal, sizeof(hexVal), "0x%04X", v);
      r["value"] = hexVal;
    }
  }

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

// POST /api/v1/quiet  {"on": bool}
// Toggles the AC's quiet (low-noise compressor) profile. Encoded as bit 7
// of slave-3 reg 2 hi. Only meaningful in slave-3 responder mode; in
// legacy bus mode the setter no-ops (no documented command path).
void Actron485ApiHandler::handle_quiet_(AsyncWebServerRequest *request, const std::string &body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (!doc["on"].is<bool>()) { send_error_(request, 400, "missing_on"); return; }
  parent_->apply_quiet_mode(doc["on"].as<bool>());
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

// POST /api/v1/zones/{n}/humidity  {"humidity": <%>}
// Per-zone relative humidity from an external sensor. The Actron bus
// doesn't carry per-zone humidity (slave 1 has some floats but they're
// AC-head side, not zone-level), so we just relay the most recent value
// through /api/v1/state for clients to consume. Plausibility bounds
// 0-100 %RH; rejects on parse failure. Returns 200 with the stored
// value because it's local state, not an RS485 command.
void Actron485ApiHandler::handle_zone_humidity_(AsyncWebServerRequest *request, int zone, const std::string &body) {
  if (zone < 1 || zone > 8) { send_error_(request, 400, "invalid_zone"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (!doc["humidity"].is<float>() && !doc["humidity"].is<double>() && !doc["humidity"].is<int>()) {
    send_error_(request, 400, "missing_humidity"); return;
  }
  double h = doc["humidity"].as<double>();
  if (h < 0.0 || h > 100.0) { send_error_(request, 400, "humidity_out_of_range"); return; }
  parent_->note_zone_humidity_update((uint8_t) zone, (float) h);
  JsonDocument resp;
  resp["number"] = zone;
  resp["humidity"] = h;
  std::string body_out;
  serializeJson(resp, body_out);
  send_json_(request, 200, body_out);
}

// GET /api/v1/settings — current runtime-mutable settings.
// api_key value is never echoed; only the set/unset flag.
void Actron485ApiHandler::handle_settings_get_(AsyncWebServerRequest *request) {
  send_json_(request, 200, parent_->build_settings_json());
}

// PATCH /api/v1/settings — partial update. Any field absent from the
// body is left unchanged. Fields:
//   - api_key (string): set the bearer token. Empty string disables auth.
//   - act_as_slave_3 (bool): enable/disable slave-3 takeover at runtime.
//     SAFETY: when toggling ON, the wall LCD's data leads must already
//     be physically disconnected from the J6 DATA bus — otherwise
//     two devices respond to slave-3 polls and the AMIB sees corrupted
//     frames. Caller's responsibility.
//   - logging_mode (int 0..5): 1=STATUS, 2=CHANGE, 3=ALL, 4=CAPTURE, 5=DELTA.
// Returns the updated settings (with api_key still masked).
void Actron485ApiHandler::handle_settings_patch_(AsyncWebServerRequest *request, const std::string &body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) { send_error_(request, 400, "invalid_json"); return; }
  if (doc["api_key"].is<const char *>()) {
    parent_->set_api_key_runtime(doc["api_key"].as<const char *>());
  }
  if (doc["timezone"].is<const char *>()) {
    parent_->set_timezone_runtime(doc["timezone"].as<const char *>());
  }
  if (doc["act_as_slave_3"].is<bool>()) {
    parent_->set_act_as_slave_3_runtime(doc["act_as_slave_3"].as<bool>());
  }
  if (doc["logging_mode"].is<int>()) {
    int m = doc["logging_mode"].as<int>();
    if (m < 0 || m > 5) { send_error_(request, 400, "invalid_logging_mode"); return; }
    parent_->set_logging_mode_runtime(m);
  }
  // Weather. weather_api_key: empty string disables. Latitude + longitude
  // must be supplied together; out-of-range clears the location.
  if (doc["weather_api_key"].is<const char *>()) {
    parent_->set_weather_api_key_runtime(doc["weather_api_key"].as<const char *>());
  }
  bool has_lat = doc["weather_latitude"].is<float>();
  bool has_lon = doc["weather_longitude"].is<float>();
  if (has_lat || has_lon) {
    if (!(has_lat && has_lon)) {
      send_error_(request, 400, "lat_lon_must_be_paired");
      return;
    }
    parent_->set_weather_location_runtime(doc["weather_latitude"].as<float>(),
                                          doc["weather_longitude"].as<float>());
  }
  send_json_(request, 200, parent_->build_settings_json());
}

// ---- Scheduler input accessors (mutex-encapsulated) -------------------------
// The weather task owns weather_*/forecast_* behind weather_mutex_. These give
// the scheduler a safe, typed peek without it reaching into the structs or
// spawning its own access path. Return false = "no usable reading".
bool Actron485Api::weather_current_temp(float &out) {
  bool ok = false;
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  if (weather_available_ && !std::isnan(weather_temp_)) {
    out = weather_temp_;
    ok = true;
  }
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  return ok;
}

bool Actron485Api::forecast_today(float &temp_min, float &temp_max, int &pop) {
  bool ok = false;
  if (weather_mutex_) xSemaphoreTake(weather_mutex_, portMAX_DELAY);
  if (forecast_available_ && forecast_days_count_ > 0) {
    temp_min = forecast_days_[0].temp_min;
    temp_max = forecast_days_[0].temp_max;
    pop = forecast_days_[0].pop;
    ok = true;
  }
  if (weather_mutex_) xSemaphoreGive(weather_mutex_);
  return ok;
}

// ---- Scheduler / away / time routes -----------------------------------------
// Self-contained dispatcher, deliberately placed at the end of the file (well
// away from the forecast handler region) to keep merge surface minimal. Auth
// has already been enforced by the caller. Returns true once it has handled
// (and responded to) the request; false lets the main ladder try other routes.
bool Actron485ApiHandler::handle_scheduler_routes_(AsyncWebServerRequest *request,
                                                   http_method method,
                                                   const std::string &url) {
  // GET /api/v1/time — bridge clock + site timezone.
  if (method == HTTP_GET && url == "/api/v1/time") {
    auto *rtc = parent_->get_time();
    bool synced = rtc != nullptr && rtc->now().is_valid();
    JsonDocument doc;
    doc["synced"] = synced;
    if (synced) {
      ESPTime t = rtc->now();
      doc["epoch"] = (uint32_t) t.timestamp;
      doc["iso_local"] = t.strftime("%Y-%m-%dT%H:%M:%S");
    } else {
      doc["epoch"] = nullptr;
      doc["iso_local"] = nullptr;
    }
    doc["timezone"] = parent_->timezone();
    std::string out;
    serializeJson(doc, out);
    send_json_(request, 200, out);
    return true;
  }

  // /api/v1/away — GET status, POST {active, return_at?}.
  if (url == "/api/v1/away") {
    if (method == HTTP_GET) {
      send_json_(request, 200, parent_->scheduler().away_json());
      return true;
    }
    if (method == HTTP_POST) {
      std::string err;
      if (!parent_->scheduler().set_away_from_json(read_body_(request), err)) {
        send_error_(request, 400, err.c_str());
        return true;
      }
      send_json_(request, 200, parent_->scheduler().away_json());
      return true;
    }
    return false;
  }

  // POST /api/v1/timers — convenience one-shot ("off in 30 min").
  if (url == "/api/v1/timers" && method == HTTP_POST) {
    uint16_t id;
    std::string err;
    if (!parent_->scheduler().create_timer_from_json(read_body_(request), id, err)) {
      send_error_(request, 400, err.c_str());
      return true;
    }
    JsonDocument doc;
    doc["id"] = id;
    std::string out;
    serializeJson(doc, out);
    send_json_(request, 200, out);
    return true;
  }

  // /api/v1/schedules (collection).
  const std::string base = "/api/v1/schedules";
  if (url == base) {
    if (method == HTTP_GET) {
      send_json_(request, 200, parent_->scheduler().list_json());
      return true;
    }
    if (method == HTTP_POST) {
      uint16_t id;
      std::string err;
      if (!parent_->scheduler().upsert_from_json(read_body_(request), false, 0, id, err)) {
        send_error_(request, 400, err.c_str());
        return true;
      }
      std::string out;
      parent_->scheduler().get_json(id, out);
      send_json_(request, 200, out);
      return true;
    }
    return false;
  }

  // /api/v1/schedules/<status|{id}[/enable|/delete]>.
  if (url.rfind(base + "/", 0) == 0) {
    std::string rest = url.substr(base.size() + 1);
    if (rest == "status" && method == HTTP_GET) {
      send_json_(request, 200, parent_->scheduler().status_json());
      return true;
    }
    std::string idpart = rest, action;
    auto slash = rest.find('/');
    if (slash != std::string::npos) {
      idpart = rest.substr(0, slash);
      action = rest.substr(slash + 1);
    }
    char *endp = nullptr;
    long idv = strtol(idpart.c_str(), &endp, 10);
    if (idpart.empty() || endp == idpart.c_str() || *endp != '\0' || idv <= 0 || idv > 65535)
      return false;
    uint16_t id = (uint16_t) idv;

    if (action.empty()) {
      if (method == HTTP_GET) {
        std::string out;
        if (!parent_->scheduler().get_json(id, out)) {
          send_error_(request, 404, "not_found");
          return true;
        }
        send_json_(request, 200, out);
        return true;
      }
      if (method == HTTP_POST) {
        uint16_t out_id;
        std::string err;
        if (!parent_->scheduler().upsert_from_json(read_body_(request), true, id, out_id, err)) {
          send_error_(request, (err == "not found") ? 404 : 400, err.c_str());
          return true;
        }
        std::string out;
        parent_->scheduler().get_json(id, out);
        send_json_(request, 200, out);
        return true;
      }
    } else if (action == "enable" && method == HTTP_POST) {
      JsonDocument doc;
      bool enabled = true;
      if (!deserializeJson(doc, read_body_(request))) enabled = doc["enabled"] | true;
      if (!parent_->scheduler().set_enabled(id, enabled)) {
        send_error_(request, 404, "not_found");
        return true;
      }
      send_json_(request, 200, "{\"ok\":true}");
      return true;
    } else if (action == "delete" && method == HTTP_POST) {
      if (!parent_->scheduler().remove(id)) {
        send_error_(request, 404, "not_found");
        return true;
      }
      send_json_(request, 200, "{\"ok\":true}");
      return true;
    }
    return false;
  }

  return false;
}

}  // namespace actron485_api
}  // namespace esphome
