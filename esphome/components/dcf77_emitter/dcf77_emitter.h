#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/switch/switch.h"

// Platform-specific includes
#ifdef USE_ESP32
#include <Arduino.h>
#include <esp32-hal-timer.h>
#elif defined(USE_ESP8266)
#include <Arduino.h>
#include <Ticker.h>
#endif

namespace esphome {
namespace dcf77_emitter {

class DCF77Emitter : public Component {
 public:
  void set_time_id(time::RealTimeClock *time_id) { this->time_id_ = time_id; }
  void set_antenna_pin(InternalGPIOPin *pin) { this->antenna_pin_ = pin; }
  void set_led_pin(InternalGPIOPin *pin) { this->led_pin_ = pin; }
  void set_sync_switch(switch_::Switch *sync_switch) { this->sync_switch_ = sync_switch; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // Timer handling functions that must be public for callbacks
  void dcf_out_tick();
  void setup_timer_();
  
 protected:
  // Core functionality
  void code_time_();
  int bin2bcd_(int dato);
  void generate_signal_(int current_second);
  void setup_carrier_();
  void stop_carrier_();
  
  // Safe timing with ESPHome conventions
  void schedule_next_tick_();
  
  time::RealTimeClock *time_id_{nullptr};
  InternalGPIOPin *antenna_pin_{nullptr};
  InternalGPIOPin *led_pin_{nullptr};
  switch_::Switch *sync_switch_{nullptr};
  
  // Signal generation
  int impulse_array_[60];
  volatile int impulse_count_ = 0;
  volatile bool carrier_enabled_ = false;
  
  // Time tracking
  int actual_hours_ = 0, actual_minutes_ = 0, actual_second_ = 0;
  int actual_day_ = 0, actual_month_ = 0, actual_year_ = 0, day_of_week_ = 0;
  volatile int last_second_ = -1;
  
  // Control state
  uint8_t pwm_channel_ = 0;
  uint32_t last_status_log_ = 0;
  uint32_t sync_start_millis_ = 0;
  bool is_initialized_ = false;
  
  // Timing resilience variables
  uint32_t last_tick_time_ = 0;
  int32_t timing_drift_ms_ = 0;
  uint32_t last_sync_millis_ = 0;
  uint16_t consecutive_drift_corrections_ = 0;

#ifdef USE_ESP32  
  hw_timer_t *timer_ = nullptr;
#elif defined(USE_ESP8266)
  Ticker ticker_;
#endif
};

}  // namespace dcf77_emitter
}  // namespace esphome