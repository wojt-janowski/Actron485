#include "utilities.h"
#include "zone_climate.h"

namespace esphome {
namespace actron485 {

Actron485ZoneClimate::Actron485ZoneClimate() = default;

void Actron485ZoneClimate::update_status() {
    if (actron_controller_->isPendingZoneCommand(number_)) {
        // If this zone is pending a message to send, wait, to prevent the home assistant UI from bouncing
        return;
    }

    unsigned long debounce_since = command_last_sent_;
    if (actron_controller_->dataLastSentTime > debounce_since) {
        debounce_since = actron_controller_->dataLastSentTime;
    }
    if ((debounce_since + DEBOUNCE_MILLIS) >= millis()) {
        // debounce our commands
        return;
    }

    bool has_changed = false;

    Actron485::MasterToZoneMessage *master = &(actron_controller_->masterToZoneMessage[zindex(number_)]);
    Actron485::ZoneToMasterMessage *zone = &(actron_controller_->zoneMessage[zindex(number_)]);

    // Target/Setpoint Temperature
    update_property(this->target_temperature, (float)actron_controller_->getZoneSetpointTemperature(number_), has_changed);
    // Current Temperature
    update_property(this->current_temperature, (float)actron_controller_->getZoneCurrentTemperature(number_), has_changed);

    // Operating Mode — present as CLIMATE_MODE_AUTO when the zone is on.
    // "Auto" reads as a clean on-label in HA / web frontends; HEAT_COOL
    // (the closer-but-uglier alternative) renders as "Heat / Cool" which
    // confuses users since the master climate already picks heat vs cool.
    // The supported_modes set in traits() must match: [OFF, AUTO].
    auto zone_on = actron_controller_->getZoneOn(number_) ? ClimateMode::CLIMATE_MODE_AUTO : ClimateMode::CLIMATE_MODE_OFF;
    update_property(this->mode, zone_on, has_changed);

    // Action Mode
    float damperPosition = (float)actron_controller_->getZoneDamperPosition(number_);

    auto action = (damperPosition > 0) ? Converter::to_climate_action(actron_controller_->getCompressorMode(), actron_controller_->getOperatingMode()) : ClimateAction::CLIMATE_ACTION_OFF;
    update_property(this->action, action, has_changed);

    if (has_changed) {
        ESP_LOGD(TAG, "Zone Changed, Publishing State");
        this->publish_state();
    }
}

void Actron485ZoneClimate::control(const climate::ClimateCall &call) {
    command_last_sent_ = millis();

    if (call.get_mode().has_value()) {
        bool isOn = call.get_mode().value() != ClimateMode::CLIMATE_MODE_OFF;
        actron_controller_->setZoneOn(number_, isOn);
        this->mode = isOn ? ClimateMode::CLIMATE_MODE_AUTO : ClimateMode::CLIMATE_MODE_OFF;
    }
    if (call.get_target_temperature().has_value()) {
        float target = call.get_target_temperature().value();
        // Clamp to master ± 2 °C (the wall LCD's authoritative per-zone
        // setpoint range) before forwarding. The dashboard slider also
        // advertises this range via traits(), but a HA service call can
        // still send out-of-range values — clamp defensively.
        float master = (float) actron_controller_->getMasterSetpoint();
        float lo = master - 2.0f;
        float hi = master + 2.0f;
        if (lo < 16.0f) lo = 16.0f;
        if (hi > 30.0f) hi = 30.0f;
        if (target < lo) target = lo;
        if (target > hi) target = hi;
        actron_controller_->setZoneSetpointTemperature(number_, target, ultima_adjusts_master_setpoint_);
        this->target_temperature = target;
    }

    this->publish_state();
}

climate::ClimateTraits Actron485ZoneClimate::traits() {
    auto traits = climate::ClimateTraits();
    traits.add_feature_flags(
        climate::ClimateFeature::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE |
        climate::ClimateFeature::CLIMATE_SUPPORTS_ACTION
    );

    // Per-zone setpoint clamps to master ± 2 °C — this is the wall LCD's
    // documented constraint (Que Ultima behaviour). Falls back to the
    // system bounds (16-30 °C) before any master state is known, or when
    // the master setpoint is itself out of range. traits() is invoked
    // fresh by ESPHome on every get_traits() call, so the slider range
    // tracks master setpoint changes without further plumbing.
    float master = (float) actron_controller_->getMasterSetpoint();
    float lo = 16.0f;
    float hi = 30.0f;
    if (master >= 16.0f && master <= 30.0f) {
        lo = master - 2.0f;
        hi = master + 2.0f;
        if (lo < 16.0f) lo = 16.0f;
        if (hi > 30.0f) hi = 30.0f;
    }
    traits.set_visual_min_temperature(lo);
    traits.set_visual_max_temperature(hi);
    traits.set_visual_temperature_step(0.5);
    traits.set_visual_current_temperature_step(0.1);
    // [OFF, AUTO] — see update_status() comment. "Auto" reads as a clean
    // on-label without lying (the system DOES decide heat vs cool).
    traits.set_supported_modes({
        ClimateMode::CLIMATE_MODE_OFF,
        ClimateMode::CLIMATE_MODE_AUTO,
    });

    return traits;
}

void Actron485ZoneClimate::dump_config() { 
    this->dump_traits_(TAG);
}

}
}
