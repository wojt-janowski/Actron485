#include "actron485_climate.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "utilities.h"
#include "zone_fan.h"
#include "zone_climate.h"

#include <cstring>
#include <cmath>

namespace esphome {
namespace actron485 {

Actron485Climate::Actron485Climate() = default;

// Global Actron485 controller
static Actron485::Controller actron_controller = Actron485::Controller();
static long counter = 0;

size_t LogStream::write(uint8_t data) {
    if (_bufferIndex >= (bufferSize - 1)) {
        // Buffer full: emit current chunk instead of dropping log data.
        _buffer[_bufferIndex] = '\0';
        ESP_LOGD(TAG, "%s", _buffer);
        _bufferIndex = 0;
    }

    if (data == '\r' || data == '\0') {
        // Ignore
    } else if (data == '\n') {
        _buffer[_bufferIndex] = '\0';
        ESP_LOGD(TAG, "%s", _buffer);
        _bufferIndex = 0;
    } else {
        _buffer[_bufferIndex] = data;
        _bufferIndex++;
    }

    return 1;
}
    
size_t LogStream::write(const uint8_t *data, size_t size) {
    for (int i=0; i<size; i++) {
        write(data[i]);
    }
    return size;
}

void LogStream::flush() {
    if (_bufferIndex <= 0) {
        return;
    }
    _buffer[_bufferIndex] = '\0';
    ESP_LOGD(TAG, "%s", _buffer);
    _bufferIndex = 0;
}

void Actron485ZoneFan::setup() {
}

void Actron485Climate::setup() {
    uint8_t we_pin = 0;
    if (we_pin_ != NULL) {
        we_pin = we_pin_->get_pin();
        we_pin_->pin_mode(gpio::FLAG_OUTPUT);
    }
    actron_controller.configure(stream_, we_pin);
    // Tell the controller which zone hosts the wall thermostat. Master
    // setpoint surfaces from that zone's setpoint (the LCD shows what the
    // wall-controller zone is targeting).
    if (control_zone_number_ >= 1 && control_zone_number_ <= 8) {
        actron_controller.setControlZone((uint8_t) control_zone_number_, true);
    }
    // Enable slave-3 responder if the YAML flag is set. Must happen AFTER
    // setControlZone so the initial renderSlave3State() inside
    // setSlaveResponderMode picks up the right zoneControlled[] mapping —
    // master setpoint writes need that to know which zone to update.
    if (act_as_slave_3_) {
        ESP_LOGW(TAG, "act_as_slave_3 enabled — impersonating wall controller "
                       "on Modbus slave 3. Wall LCD data leads MUST be "
                       "disconnected from J6 DATA bus to avoid frame "
                       "collisions.");
        // Restore last-known-good state from NVS BEFORE setSlaveResponderMode
        // primes the buffer, so initSlave3Defaults' fallback values don't
        // briefly clobber the restored state. initSlave3Defaults is
        // load-aware: it only fills in zero / uninitialised fields.
        load_slave3_state_();
        actron_controller.setSlaveResponderMode(3, true);
    }
    logStream_ = LogStream();
    if (logging_mode_ > 0) {
        actron_controller.configureLogging(&logStream_);
        switch (logging_mode_) {
            case 1:
                actron_controller.printOutMode = Actron485::PrintOutMode::StatusOnly;
                break;
            case 2:
                actron_controller.printOutMode = Actron485::PrintOutMode::ChangedMessages;
                break;
            case 3:
                actron_controller.printOutMode = Actron485::PrintOutMode::AllMessages;
                break;
            case 4:
                actron_controller.printOutMode = Actron485::PrintOutMode::CorrelationCapture;
                break;
            case 5:
                actron_controller.printOutMode = Actron485::PrintOutMode::RegisterDelta;
                break;
        }
    }

}

void Actron485Climate::loop() {
    actron_controller.loop();
    unsigned long now = millis();
    if (now-counter > 1000) {
        counter = now;
        update_status();
        if (act_as_slave_3_) {
            maybe_save_slave3_state_();
        }
    }
}

void Actron485Climate::power_on() { 
    actron_controller.setSystemOn(true);
}

void Actron485Climate::power_off() {
    actron_controller.setSystemOn(false);
}

void Actron485Climate::power_toggle() {
    actron_controller.setSystemOn(!actron_controller.getSystemOn());
}

Actron485::Controller *Actron485Climate::get_controller() {
    return &actron_controller;
}

void Actron485Climate::add_zone(int number, Actron485ZoneFan *fan) {
    if (number < 1 || number > 8) {
        ESP_LOGE(TAG, "Zone out of bounds %d, 1-8 accepted", number);
        return;
    }
    fan->set_controller(&actron_controller);
    fan->set_zone_number(number);
    zones_[number-1] = fan;
}

void Actron485Climate::add_ultima_zone(int number, Actron485ZoneClimate *climate) {
    if (number < 1 || number > 8) {
        ESP_LOGE(TAG, "Zone out of bounds %d, 1-8 accepted", number);
        return;
    }
    climate->set_controller(&actron_controller);
    climate->set_zone_number(number);
    climate->set_ultima_adjusts_master_setpoint(ultima_adjusts_master_setpoint_);
    zone_climates_[number-1] = climate;
}

void Actron485Climate::update_status() {
    if (actron_controller.dataLastSentTime >= actron_controller.statusLastReceivedTime || actron_controller.totalPendingMainCommands() > 0) {
        // Don't check until we received a new status message after sending a command
        // to debounce status changes 
        return;
    }

    unsigned long debounce_since = command_last_sent_;
    if (actron_controller.dataLastSentTime > debounce_since) {
        debounce_since = actron_controller.dataLastSentTime;
    }
    if ((debounce_since + DEBOUNCE_MILLIS) >= millis()) {
        // debounce our commands
        return;
    }

    bool has_changed = false;

    // Target/Setpoint Temperature
    update_property(this->target_temperature, (float)actron_controller.getMasterSetpoint(), has_changed);
    // Current Temperature
    update_property(this->current_temperature, (float)actron_controller.getMasterCurrentTemperature(), has_changed);

    // Continuous Fan Mode
    bool continuous_mode = actron_controller.getContinuousFanMode();
    has_changed = has_changed || (this->set_custom_preset_(Converter::to_preset(continuous_mode)));

    // Fan Speed Mode
    Actron485::FanMode fan_mode = actron_controller.getFanSpeed();
    has_changed = has_changed || (this->set_fan_mode_(Converter::to_fan_mode(fan_mode)));

    // Operating Mode
    auto mode = actron_controller.getSystemOn() ? Converter::to_climate_mode(actron_controller.getOperatingMode()) : ClimateMode::CLIMATE_MODE_OFF;
    update_property(this->mode, mode, has_changed);

    // Action Mode
    auto action = actron_controller.getSystemOn() ? Converter::to_climate_action(actron_controller.getCompressorMode(), actron_controller.getOperatingMode()) : ClimateAction::CLIMATE_ACTION_OFF;
    update_property(this->action, action, has_changed);

    if (has_changed) {
        ESP_LOGD(TAG, "Has Changed, Publishing State");
        this->publish_state();
    }

    // Zone updates
    for (int z=0; z<8; z++) {
        if (zones_[z]) {
            zones_[z]->update_status();
        }
        if (zone_climates_[z]) {
            zone_climates_[z]->update_status();
        }
    }
}

void Actron485Climate::control(const climate::ClimateCall &call) {
    command_last_sent_ = millis();

    if (call.get_mode().has_value()) {
        Actron485::OperatingMode operating_mode = Converter::to_actron_operating_mode(call.get_mode().value());
        actron_controller.setOperatingMode(operating_mode);
        this->mode = call.get_mode().value();
    }
    if (call.get_target_temperature().has_value()) {
        actron_controller.setMasterSetpoint(call.get_target_temperature().value());
        this->target_temperature = call.get_target_temperature().value();
    }
    if (call.has_custom_preset()) {
        bool continuous_mode = Converter::to_continuous_mode(call.get_custom_preset());
        if (actron_controller.getContinuousFanMode() != continuous_mode) {
            actron_controller.setContinuousFanMode(continuous_mode);
        }
        this->set_custom_preset_(call.get_custom_preset());
    }
    if (call.get_fan_mode().has_value()) {
        Actron485::FanMode fan_mode = Converter::to_actron_fan_mode(call.get_fan_mode().value());
        actron_controller.setFanSpeed(fan_mode);
        this->fan_mode = call.get_fan_mode().value();
    }

    this->publish_state();
}

climate::ClimateTraits Actron485Climate::traits() {
    auto traits = climate::ClimateTraits();
    traits.add_feature_flags(
        climate::ClimateFeature::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE |
        climate::ClimateFeature::CLIMATE_SUPPORTS_ACTION
    );
    traits.set_visual_min_temperature(16);
    traits.set_visual_max_temperature(30);
    traits.set_visual_temperature_step(0.5);
    traits.set_visual_current_temperature_step(0.1);
    traits.set_supported_modes({
        ClimateMode::CLIMATE_MODE_OFF,
        ClimateMode::CLIMATE_MODE_COOL,
        ClimateMode::CLIMATE_MODE_HEAT,
        ClimateMode::CLIMATE_MODE_HEAT_COOL,
        ClimateMode::CLIMATE_MODE_FAN_ONLY,
    });
    traits.set_supported_fan_modes({
        ClimateFanMode::CLIMATE_FAN_LOW,
        ClimateFanMode::CLIMATE_FAN_MEDIUM,
        ClimateFanMode::CLIMATE_FAN_HIGH,
    });
    traits.set_supported_custom_presets({
        Converter::FAN_STANDARD,
        Converter::FAN_CONTINUOUS
    });
    if (has_esp_auto_) {
        traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_AUTO);
    }

    return traits;
}

void Actron485Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "Actron485 Status:");
  ESP_LOGCONFIG(TAG, "  Receiving Data: %s", actron_controller.receivingData() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Act as Slave 3: %s", act_as_slave_3_ ? "YES" : "NO");
  this->dump_traits_(TAG);
}

void Actron485Climate::load_slave3_state_() {
    // Stable hash for the preference slot. Bumping this key (or the layout
    // version inside the blob) invalidates any previously-saved state on
    // upgrade — desired when the persisted schema changes.
    uint32_t hash = fnv1_hash(std::string("actron485_slave3_state_v1"));
    slave3_state_pref_ = global_preferences->make_preference<Slave3PersistedState>(hash);
    slave3_persist_ready_ = true;

    Slave3PersistedState blob{};
    if (!slave3_state_pref_.load(&blob)) {
        ESP_LOGI(TAG, "No saved slave-3 state — initSlave3Defaults will seed.");
        return;
    }
    if (blob.magic != kSlave3StateMagic || blob.version != kSlave3StateVersion) {
        ESP_LOGW(TAG, "Saved slave-3 state has wrong magic/version "
                       "(magic=0x%08X ver=%u); discarding.", blob.magic, blob.version);
        return;
    }

    // Restore: directly populate the controller's typed state. We do this
    // BEFORE setSlaveResponderMode runs initSlave3Defaults, which is
    // load-aware (only fills in zero / uninitialised fields).
    //
    // Defensive bounds checks on the loaded zone setpoints. Observed in the
    // wild on 2026-05-30 boot: NVS came back with several zoneSetpoint[]
    // entries at 127.5 / 62.5 / 32.5 °C — root cause uncertain (possibly
    // partially-corrupt broadcast bytes that flowed through
    // applySlave11StateBroadcast → save during an earlier session). The
    // encoder happily clamped these and published reg 4-12 with garbage
    // setpoints; the AMIB then drove the system off-spec. Substitute the
    // 22 °C default for any out-of-range value and log so it's noticeable.
    bool anyClamped = false;
    for (int z = 0; z < 8; z++) {
        double sp = blob.zone_setpoints[z];
        if (!std::isfinite(sp) || sp < 10.0 || sp > 40.0) {
            ESP_LOGW(TAG, "Saved zone %d setpoint %.1f °C out of [10, 40] — "
                          "substituting 22 °C", z + 1, sp);
            blob.zone_setpoints[z] = 22.0;
            anyClamped = true;
        }
    }

    actron_controller.stateMessage2.operatingMode = (Actron485::OperatingMode) blob.operating_mode;
    actron_controller.stateMessage2.fanMode       = (Actron485::FanMode) blob.fan_mode;
    actron_controller.stateMessage2.continuousFan = blob.continuous_fan != 0;
    actron_controller.stateMessage2.quietMode     = blob.quiet_mode != 0;
    for (int z = 0; z < 8; z++) {
        actron_controller.stateMessage2.zoneOn[z] = (blob.zone_on_bitmap & (1u << z)) != 0;
        actron_controller.zoneSetpoint[z]         = blob.zone_setpoints[z];
    }
    actron_controller.stateMessage2.initialised = true;
    // last_persisted_ tracks "what's currently on disk" for memcmp-skip
    // in maybe_save_slave3_state_(). If we just substituted values, the
    // sanitised state diverges from the disk blob — force last_persisted_
    // back to a zero state so the next save unconditionally overwrites
    // the bad NVS contents.
    last_persisted_ = anyClamped ? Slave3PersistedState{} : blob;
    ESP_LOGI(TAG, "Restored slave-3 state from NVS (mode=%u fan=%u zones=0x%02X%s)",
             blob.operating_mode, blob.fan_mode, blob.zone_on_bitmap,
             anyClamped ? " — clamped" : "");
}

void Actron485Climate::maybe_save_slave3_state_() {
    if (!slave3_persist_ready_) {
        // load_slave3_state_() prepares the ESPPreferenceObject. If
        // act_as_slave_3 is true we always run load first, so this branch
        // only triggers when the flag flips at runtime — paranoia.
        return;
    }

    // Throttle to once per ~5 s. NVS write cycles are bounded (~100k); a flurry
    // of API setpoint nudges shouldn't burn the flash. The contents-equal
    // short-circuit below is the real saver for typical use — this gate just
    // bounds the worst case.
    unsigned long now = millis();
    if ((now - last_save_attempt_ms_) < 5000) {
        return;
    }
    last_save_attempt_ms_ = now;

    Slave3PersistedState cur{};
    cur.magic   = kSlave3StateMagic;
    cur.version = kSlave3StateVersion;
    cur.operating_mode = (uint8_t) actron_controller.stateMessage2.operatingMode;
    cur.fan_mode       = (uint8_t) actron_controller.stateMessage2.fanMode;
    cur.continuous_fan = actron_controller.stateMessage2.continuousFan ? 1 : 0;
    cur.quiet_mode     = actron_controller.stateMessage2.quietMode     ? 1 : 0;
    uint8_t bitmap = 0;
    for (int z = 0; z < 8; z++) {
        if (actron_controller.stateMessage2.zoneOn[z]) bitmap |= uint8_t(1 << z);
        cur.zone_setpoints[z] = actron_controller.zoneSetpoint[z];
    }
    cur.zone_on_bitmap = bitmap;

    if (memcmp(&cur, &last_persisted_, sizeof(cur)) == 0) {
        // No change since last save — skip.
        return;
    }
    if (!slave3_state_pref_.save(&cur)) {
        ESP_LOGW(TAG, "Slave-3 state save failed");
        return;
    }
    last_persisted_ = cur;
    ESP_LOGD(TAG, "Persisted slave-3 state (mode=%u fan=%u zones=0x%02X)",
             cur.operating_mode, cur.fan_mode, cur.zone_on_bitmap);
}

}
}
