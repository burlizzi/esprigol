#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esp_adc/adc_continuous.h"
#include "esphome/core//gpio.h" 

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


namespace esphome
{

namespace adc_continous
{
    class ADCContinuousSensor : public sensor::Sensor, public PollingComponent, public voltage_sampler::VoltageSampler
    {
    public:
        #ifdef USE_ESP32
        /// Set the attenuation for this pin. Only available on the ESP32.
        void set_attenuation(adc_atten_t attenuation) { this->attenuation_ = attenuation; }
/*        void set_channel1(adc1_channel_t channel) {
          this->channel1_ = channel;
          this->channel2_ = ADC2_CHANNEL_MAX;
        }
        void set_channel2(adc2_channel_t channel) {
          this->channel2_ = channel;
          this->channel1_ = ADC1_CHANNEL_MAX;
        }*/
        void set_autorange(bool autorange) { this->autorange_ = autorange; }
      #endif  // USE_ESP32
      
        /// Update ADC values
        void update() override;
        /// Setup ADC
        void setup() override;
        void dump_config() override;
        /// `HARDWARE_LATE` setup priority
        float get_setup_priority() const override;
        void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
        void set_output_raw(bool output_raw) { this->output_raw_ = output_raw; }
        void set_sample_count(uint8_t sample_count);
        void set_sampling_mode(SamplingMode sampling_mode);
        float sample() override;
      
      #ifdef USE_ESP8266
        std::string unique_id() override;
      #endif  // USE_ESP8266
      
      #ifdef USE_RP2040
        void set_is_temperature() { this->is_temperature_ = true; }
      #endif  // USE_RP2040
      
       protected:
        InternalGPIOPin *pin_;
        bool output_raw_{false};
        uint8_t sample_count_{1};
        SamplingMode sampling_mode_{SamplingMode::AVG};
      
      #ifdef USE_RP2040
        bool is_temperature_{false};
      #endif  // USE_RP2040
      
      #ifdef USE_ESP32
        adc_atten_t attenuation_{ADC_ATTEN_DB_0};
        adc1_channel_t channel1_{ADC1_CHANNEL_MAX};
        adc2_channel_t channel2_{ADC2_CHANNEL_MAX};
        bool autorange_{false};
      #if ESP_IDF_VERSION_MAJOR >= 5
        esp_adc_cal_characteristics_t cal_characteristics_[SOC_ADC_ATTEN_NUM] = {};
      #else
        esp_adc_cal_characteristics_t cal_characteristics_[ADC_ATTEN_MAX] = {};
      #endif  // ESP_IDF_VERSION_MAJOR
      #endif  // USE_ESP32
    };
    
    
} // namespace adc_continous
} // namespace esphome