#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#ifdef USE_UDP
#include "esphome/components/udp/udp_component.h"
#endif

#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esp_adc/adc_continuous.h"
#include "esphome/core//gpio.h" 

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifdef USE_OTA_STATE_CALLBACK
#include "esphome/components/ota/ota_backend.h"
#endif

namespace esphome
{

namespace adc_continous
{
    bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
    static const char *const TAG = "CLAMP";
    class ADCContinuousSensor :  public  sensor::Sensor, public virtual PollingComponent, public voltage_sampler::VoltageSampler
#ifdef USE_OTA_STATE_LISTENER
    ,public ota::OTAGlobalStateListener
#endif
    {
    public:
#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif        
        ADCContinuousSensor():PollingComponent(1500) {};
        void calcIV(uint32_t len,uint8_t* buffer);
        uint16_t analogRead(uint8_t pin);
        /// Update ADC values
        void update() override;
        /// Setup ADC
        void setup() override;
        void dump_config() override;
        /// `HARDWARE_LATE` setup priority
        float get_setup_priority() const override;
        //void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
        void set_output_raw(bool output_raw) { this->output_raw_ = output_raw; }
        void set_sample_count(uint32_t sample_count){ this->sample_count_ = sample_count; 
          buffer_ = (uint8_t*)malloc(sample_count_ * SOC_ADC_DIGI_RESULT_BYTES * 4);memset(buffer_,0,sample_count_ * SOC_ADC_DIGI_RESULT_BYTES * 4);
        } 
        void set_attenuation(adc_atten_t attenuation) { this->attenuation_ = attenuation; }
        void set_phaseShift1(float phaseShift1) { this->phaseShift1 = phaseShift1; }
        void set_phaseShift2(float phaseShift2) { this->phaseShift2 = phaseShift2; }
        void set_phaseShift3(float phaseShift3) { this->phaseShift3 = phaseShift3; }
        void set_calibrationV(float calibrationV) { this->calibrationV = calibrationV; }
        void set_calibrationI1(float calibrationI1) { this->calibrationI1 = calibrationI1; }
        void set_calibrationI2(float calibrationI2) { this->calibrationI2 = calibrationI2; }
        void set_calibrationI3(float calibrationI3) { this->calibrationI3 = calibrationI3; }
        //void set_autorange(bool autorange) { this->autorange_ = autorange; }  
        void set_channel(uint8_t pin, adc_channel_t channel) { this->channel[pin] = channel; channels[channel]=pin;}
        void set_frequency(uint32_t samplefreq) { this->samplefreq = samplefreq; }
        inline uint32_t get_frequency() { return this->samplefreq; }
        inline uint32_t get_sample_count() { return this->sample_count_; }
        float sample() override;
        float getRealPower1(){ return lastRealPower1;}
        float getRealPower2(){ return lastRealPower2;}
        float getRealPower3(){ return lastRealPower3;}
        //virtual void data(char* data);
        void start() ;
        void stop() ;
        void add_adc_callback(std::function<void(const float&)> &&callback) {
          this->adc_callback_.add(std::move(callback));
        }
#ifdef USE_UDP
        void set_udp(udp::UDPComponent* udp) {
          udp_=udp;
          udp->set_should_broadcast();

        }
        udp::UDPComponent* udp_=nullptr;
#endif        

       protected:
        //InternalGPIOPin *pin_;
        void  calcPhaseShift(uint8_t lChannel);
        void loop() override;
        bool output_raw_{false};
        uint32_t sample_count_{1};
        uint32_t samplefreq{50};
        uint8_t cycles{0};
        TaskHandle_t adc_task_handle = NULL;
        adc_continuous_handle_t adc_handle = NULL;        
        CallbackManager<void(const float&)> adc_callback_{};
        uint8_t* buffer_{nullptr};
        static bool IRAM_ATTR callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
        float sumPower1=0;
        float sumPower2=0;
        float sumPower3=0;        
        float lastRealPower1=0;
        float lastRealPower2=0;
        float lastRealPower3=0;        
        float phaseShift1=0;
        float phaseShift2=1.0/3.0;
        float phaseShift3=2.0/3.0;
        float calibrationV=1;
        float calibrationI1=1;
        float calibrationI2=1;
        float calibrationI3=1;

     
      #ifdef USE_ESP32
        adc_atten_t attenuation_{ADC_ATTEN_DB_12};
        adc_channel_t channel[4];
        uint8_t channels[10];
      #endif  // USE_ESP32
    };
    
    
} // namespace adc_continous
} // namespace esphome