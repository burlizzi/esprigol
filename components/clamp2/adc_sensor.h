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
//    template <int VPINS, int CPINS>
    class ADCContinuousSensor :  public  sensor::Sensor, public virtual PollingComponent, public voltage_sampler::VoltageSampler
#ifdef USE_OTA_STATE_LISTENER
    ,public ota::OTAGlobalStateListener
#endif
    {
    public:
#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif        
        ADCContinuousSensor(int vpins,int cpins)
          :PollingComponent(1500)
          ,vPins_(vpins)
          ,cPins_(cpins) 
          ,calibrationVs(vpins,1.0)
          ,sumPower(cpins,0.0)
          ,lastRealPower(cpins,0.0)
          ,phaseShifts(cpins,0.0)
          ,calibrationIs(cpins,1.0)
          {}
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
          buffer_ = (uint8_t*)malloc(sample_count_ * SOC_ADC_DIGI_RESULT_BYTES * (vPins_+cPins_));
          memset(buffer_,0,sample_count_ * SOC_ADC_DIGI_RESULT_BYTES * (vPins_+cPins_));
          sinTable.resize(sample_count_);
          for(size_t i=0;i<sample_count_;i++)          
          {
            sinTable[i]=sin(2.0f*M_PI*i/sample_count_);
          }
        } 
        void set_attenuation(adc_atten_t attenuation) { 
          this->attenuation_ = attenuation; 
        }
        void set_phaseShift(size_t index, float phaseShift) { 
          this->phaseShifts[index] = phaseShift/360.0f; 
        }
        void set_calibrationV(size_t index, float calibrationV) { 
          this->calibrationVs[index] = calibrationV; 
        }
        void set_calibrationI(size_t index, float calibrationI) { 
          this->calibrationIs[index] = calibrationI; 
        }
        
        //void set_autorange(bool autorange) { this->autorange_ = autorange; }  
        void set_autostart(bool autostart) { this->autostart_ = autostart; }  
        void set_channel(uint8_t pin, adc_channel_t channel) { 
          this->channel[pin] = channel; channels[channel]=pin;
        }
        void set_frequency(uint32_t samplefreq) { this->samplefreq = samplefreq; }
        inline uint32_t get_frequency() { return this->samplefreq; }
        inline uint32_t get_sample_count() { return this->sample_count_; }
        float sample() override;
        float getRealPower(size_t index){ return this->lastRealPower[index]; }
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
        int vPins_{0};
        int cPins_{0};
        uint32_t sample_count_{1};
        uint32_t samplefreq{50};
        uint8_t cycles{0};
        TaskHandle_t adc_task_handle = NULL;
        adc_continuous_handle_t adc_handle = NULL;        
        CallbackManager<void(const float&)> adc_callback_{};
        uint8_t* buffer_{nullptr};
        static bool IRAM_ATTR callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
        
        bool started_{false};
        bool autostart_{true};

        std::vector<float> calibrationVs;
        std::vector<float> sumPower;
        std::vector<float> lastRealPower;
        std::vector<float> phaseShifts;
        std::vector<float> calibrationIs;
        std::vector<float> sinTable;
        

     
      #ifdef USE_ESP32
        adc_atten_t attenuation_{ADC_ATTEN_DB_12};
        adc_channel_t channel[10];
        uint8_t channels[10];
      #endif  // USE_ESP32
    };
    
    
} // namespace adc_continous
} // namespace esphome