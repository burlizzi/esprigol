#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esp_adc/adc_continuous.h"
#include "esphome/core//gpio.h" 

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


namespace esphome
{

namespace adc_continous
{
    static const char *const TAG = "ADCContinuousSensor";
    class ADCContinuousSensor :  public  sensor::Sensor, public virtual PollingComponent, public voltage_sampler::VoltageSampler
    {
    public:
        ADCContinuousSensor() {};
  
        /// Update ADC values
        void update() override;
        /// Setup ADC
        void setup() override;
        void dump_config() override;
        /// `HARDWARE_LATE` setup priority
        float get_setup_priority() const override;
        void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
        void set_output_raw(bool output_raw) { this->output_raw_ = output_raw; }
        void set_sample_count(uint32_t sample_count){ this->sample_count_ = sample_count; buffer_ = (char*)malloc(sample_count_); }
        void set_attenuation(adc_atten_t attenuation) { this->attenuation_ = attenuation; }
        void set_autorange(bool autorange) { this->autorange_ = autorange; }  
        void set_channel(adc_channel_t channel) { this->channel = channel; }
        void set_frequency(uint32_t samplefreq) { this->samplefreq = samplefreq; }
        inline uint32_t get_frequency() { return this->samplefreq; }
        inline uint32_t get_sample_count() { return this->sample_count_; }
        float sample() override;
        virtual void data(char* data);
        void start() ;
        void stop() ;
        void add_adc_callback(std::function<void(const char*)> &&callback) {
          this->adc_callback_.add(std::move(callback));
        }
        const char* get_buffer() { return buffer_; }
       protected:
        InternalGPIOPin *pin_;
        bool output_raw_{false};
        uint32_t sample_count_{1};
        uint32_t samplefreq{200000};
        TaskHandle_t adc_task_handle = NULL;
        adc_continuous_handle_t adc_handle = NULL;        
        CallbackManager<void(const char*)> adc_callback_{};
        char * buffer_{nullptr};
        static void adc_task(void *param);
        static bool IRAM_ATTR callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

     
      #ifdef USE_ESP32
        adc_atten_t attenuation_{ADC_ATTEN_DB_0};
        adc_channel_t channel;
        bool autorange_{false};
      #endif  // USE_ESP32
    };
    
    
} // namespace adc_continous
} // namespace esphome