#include "ultrasonic_sensor3.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ultrasonic3 {

static const char *const TAG = "ultrasonic.sensor3";

void UltrasonicSensor3Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Ultrasonic Sensor3...");
  this->signal_pin_->setup();
  this->signal_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);
  this->signal_pin_->digital_write(false);

  // isr is faster to access
  signal_isr_ = signal_pin_->to_isr();
}
void UltrasonicSensor3Component::update() {
  this->signal_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);
  this->signal_pin_->digital_write(true);
  
  delayMicroseconds(this->pulse_time_us_);
  this->signal_pin_->digital_write(false);
  this->signal_pin_->pin_mode(esphome::gpio::FLAG_INPUT);
  
  const uint32_t start = micros();
  while (micros() - start < timeout_us_ && signal_isr_.digital_read())
    ;
  while (micros() - start < timeout_us_ && !signal_isr_.digital_read())
    ;
  const uint32_t pulse_start = micros();
  while (micros() - start < timeout_us_ && signal_isr_.digital_read())
    ;
  const uint32_t pulse_end = micros();

  ESP_LOGV(TAG, "Echo took %" PRIu32 "µs", pulse_end - pulse_start);

  if (pulse_end - start >= timeout_us_) {
    ESP_LOGD(TAG, "'%s' - Distance measurement timed out!", this->name_.c_str());
    this->publish_state(NAN);
  } else {
    float result = UltrasonicSensor3Component::us_to_m(pulse_end - pulse_start);
    ESP_LOGD(TAG, "'%s' - Got distance: %.3f m", this->name_.c_str(), result);
    this->publish_state(result);
  }
}
void UltrasonicSensor3Component::dump_config() {
  LOG_SENSOR("", "Ultrasonic Sensor3", this);
  LOG_PIN("  Data Pin: ", this->signal_pin_);
  ESP_LOGCONFIG(TAG, "  Pulse time: %" PRIu32 " µs", this->pulse_time_us_);
  ESP_LOGCONFIG(TAG, "  Timeout: %" PRIu32 " µs", this->timeout_us_);
  LOG_UPDATE_INTERVAL(this);
}
float UltrasonicSensor3Component::us_to_m(uint32_t us) {
  const float speed_sound_m_per_s = 343.0f;
  const float time_s = us / 1e6f;
  const float total_dist = time_s * speed_sound_m_per_s;
  return total_dist / 2.0f;
}
float UltrasonicSensor3Component::get_setup_priority() const { return setup_priority::DATA; }
void UltrasonicSensor3Component::set_pulse_time_us(uint32_t pulse_time_us) { this->pulse_time_us_ = pulse_time_us; }
void UltrasonicSensor3Component::set_timeout_us(uint32_t timeout_us) { this->timeout_us_ = timeout_us; }

}  // namespace ultrasonic3
}  // namespace esphome
