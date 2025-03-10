#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"

#include <cinttypes>

namespace esphome {
namespace ultrasonic3 {

class UltrasonicSensor3Component : public sensor::Sensor, public PollingComponent {
 public:
  void set_signal_pin(InternalGPIOPin *signal_pin) { signal_pin_ = signal_pin; }

  /// Set the timeout for waiting for the echo in µs.
  void set_timeout_us(uint32_t timeout_us);

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  /// Set up pins and register interval.
  void setup() override;
  void dump_config() override;

  void update() override;

  float get_setup_priority() const override;

  /// Set the time in µs the trigger pin should be enabled for in µs, defaults to 10µs (for HC-SR04)
  void set_pulse_time_us(uint32_t pulse_time_us);

 protected:
  /// Helper function to convert the specified echo duration in µs to meters.
  static float us_to_m(uint32_t us);
  /// Helper function to convert the specified distance in meters to the echo duration in µs.

  InternalGPIOPin *signal_pin_;
  ISRInternalGPIOPin signal_isr_;
  uint32_t timeout_us_{};  /// 2 meters.
  uint32_t pulse_time_us_{};
};

}  // namespace ultrasonic3
}  // namespace esphome
