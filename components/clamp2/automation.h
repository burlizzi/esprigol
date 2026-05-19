#pragma once

#include "esphome/core/automation.h"
#include "adc_sensor.h"


namespace esphome {
namespace adc_continous {

// Trigger that forwards ADC data as a const std::vector<uint8_t>& to automations.
class DataTrigger : public Trigger<const float &> {
 public:
  explicit DataTrigger(ADCContinuousSensor *parent) {
    parent->add_adc_callback([this](const float &data) { this->trigger(data); });
  }
};

}  // namespace adc_continous
}  // namespace esphome