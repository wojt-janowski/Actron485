#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/climate/climate.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/fan/fan.h"
#include "Actron485.h"
#include "zone_fan.h"
#include "zone_climate.h"

namespace esphome {
namespace actron485 {

using fan::Fan;
using climate::ClimateCall;
using climate::ClimatePreset;
using climate::ClimateTraits;
using climate::ClimateMode;
using climate::ClimateSwingMode;
using climate::ClimateFanMode;
using climate::ClimateAction;

/* Stream from UART component (copied from Midea component) */
class UARTStream : public Actron485::SerialStream {
    public:
        void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }

        /* Stream interface implementation */

        int available() override { return this->uart_->available(); }
        int read() override {
            uint8_t data;
            this->uart_->read_byte(&data);
            return data;
        }
        int peek() override {
            uint8_t data;
            this->uart_->peek_byte(&data);
            return data;
        }
        size_t write(uint8_t data) override {
            this->uart_->write_byte(data);
            return 1;
        }
        size_t write(const uint8_t *data, size_t size) override {
            this->uart_->write_array(data, size);
            return size;
        }
        void flush() override { this->uart_->flush(); }

    protected:
        uart::UARTComponent *uart_;
};

class LogStream : public Actron485::LogSink {
    public:
        size_t write(uint8_t data) override;
        size_t write(const uint8_t *data, size_t size) override;
        void flush();
    protected:
        static const int bufferSize = 512;
        char _buffer[bufferSize];
        int _bufferIndex = 0;
};

class Actron485Climate : public climate::Climate, public Component {

    private:
    public:
        Actron485Climate();
        void setup() override;
        void loop() override;

        void set_we_pin(InternalGPIOPin *pin) { we_pin_ = pin; }
        void set_has_esp(bool available) { has_esp_auto_ = available; }
        void set_logging_mode(int logging_mode) { logging_mode_ = logging_mode; }
        void set_uart_parent(uart::UARTComponent *parent) { this->stream_.set_uart(parent); }
        void set_ultima_settings(bool available, bool adjusts_master_target) {
            has_ultima_ = available;
            ultima_adjusts_master_setpoint_ = adjusts_master_target;
        }
        // 1-indexed zone number where the wall controller lives. The master
        // setpoint shown on the LCD is sourced from this zone's setpoint
        // (see applySlave11StateBroadcast).
        void set_control_zone(int zone_number) { control_zone_number_ = zone_number; }

        void add_zone(int number, Actron485ZoneFan *fan);
        void add_ultima_zone(int number, Actron485ZoneClimate *climate);

        void dump_config() override;
        void update_status();

        void power_on();
        void power_off();
        void power_toggle();

        // Accessor for external components (e.g. actron485_api) that need to
        // read/write the underlying Actron485 controller state directly.
        Actron485::Controller *get_controller();

        // Snapshot helpers for the API layer to read zone fan assignments
        // without re-plumbing them from YAML.
        Actron485ZoneFan *get_zone_fan(int number) const {
            return (number >= 1 && number <= 8) ? zones_[number - 1] : nullptr;
        }
        Actron485ZoneClimate *get_zone_climate(int number) const {
            return (number >= 1 && number <= 8) ? zone_climates_[number - 1] : nullptr;
        }
        bool has_ultima() const { return has_ultima_; }

    protected:
        InternalGPIOPin *we_pin_ = NULL;
        UARTStream stream_;
        LogStream logStream_;
        
        int logging_mode_;
        bool has_esp_auto_;
        bool has_ultima_;
        bool ultima_adjusts_master_setpoint_;
        int control_zone_number_ = 1;
        Actron485ZoneFan *zones_[8] = {};
        Actron485ZoneClimate *zone_climates_[8] = {};

        // For debouncing
        unsigned long command_last_sent_ = 0;

        /// Override control to change settings of the climate device.
        void control(const climate::ClimateCall &call) override;

        /// Return the traits of this controller.
        climate::ClimateTraits traits() override;
};

}
}
