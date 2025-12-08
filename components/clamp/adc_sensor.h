#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/udp/udp_component.h"

#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esp_adc/adc_continuous.h"
#include "esphome/core//gpio.h" 

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


namespace esphome
{

namespace adc_continous
{
    bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
    static const char *const TAG = "ADCContinuousSensor";
    class ADCContinuousSensor :  public  sensor::Sensor, public virtual PollingComponent, public voltage_sampler::VoltageSampler
    {
    public:
        bool printwave=false;
        ADCContinuousSensor():PollingComponent(1000) {};
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
          //buffer_ = (uint8_t*)malloc(sample_count_*SOC_ADC_DIGI_RESULT_BYTES);memset(buffer_,0,sample_count_*SOC_ADC_DIGI_RESULT_BYTES);
        } 
        void set_attenuation(adc_atten_t attenuation) { this->attenuation_ = attenuation; }
        void set_autorange(bool autorange) { this->autorange_ = autorange; }  
        void set_channel(uint8_t pin, adc_channel_t channel) { this->channel[pin] = channel; channels[channel]=pin;}
        void set_frequency(uint32_t samplefreq) { this->samplefreq = samplefreq; }
        inline uint32_t get_frequency() { return this->samplefreq; }
        inline uint32_t get_sample_count() { return this->sample_count_; }
        float sample() override;
        float getRealPower1(){ return lastRealPower1;}
        float getRealPower2(){ return lastRealPower2;}
        float getRealPower3(){ return lastRealPower3;}
        virtual void data(char* data);
        void start() ;
        void stop() ;
        void add_adc_callback(std::function<void(const uint8_t*)> &&callback) {
          this->adc_callback_.add(std::move(callback));
        }
        void set_udp(udp::UDPComponent* udp) {
          udp_=udp;
          udp->set_should_broadcast();

        }
        udp::UDPComponent* udp_=nullptr;
        //const uint8_t* get_buffer() { return buffer_; }
    double 
      lastRealPower1=0,lastRealPower2=0,lastRealPower3=0,
      Vrms,
      realPower1,apparentPower1,powerFactor1,Irms1,
      realPower2,apparentPower2,powerFactor2,Irms2,
      realPower3,apparentPower3,powerFactor3,Irms3;
      double ICAL1 = -62.84;    // Current calibration constant
      double ICAL2 = -8.21;
      double ICAL3 = -8.21;
float avgp1 = 0;
float avgp2 = 0;
float avgp3 = 0;
    uint8_t phaseCal1=0,phaseCal2=1,phaseCal3=2;         //Holds the phase calibration value.

       protected:
        //InternalGPIOPin *pin_;
        void  calcPhaseShift(uint8_t lChannel);
        void loop() override;
        bool output_raw_{false};
        uint32_t sample_count_{1};
        uint32_t samplefreq{200000};
        TaskHandle_t adc_task_handle = NULL;
        adc_continuous_handle_t adc_handle = NULL;        
        CallbackManager<void(const uint8_t*)> adc_callback_{};
        //uint8_t* buffer_{nullptr};
        static void adc_task(void *param);
        static bool IRAM_ATTR callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
        
        double lastFilteredV1=0,lastFilteredV2=0,lastFilteredV3=0; 
    double offsetV=0;                          //Low-pass filter output
    double offsetI1=0;                          //Low-pass filter output
    double offsetI2=0;                          //Low-pass filter output
    double offsetI3=0;                          //Low-pass filter output

    double phaseShiftedV1=0;                             //Holds the calibrated phase shifted voltage.
    double phaseShiftedV2=0;                             //Holds the calibrated phase shifted voltage.
    double phaseShiftedV3=0;                             //Holds the calibrated phase shifted voltage.

    double 
        sqI1=0,sumI1=0,instP1=0,sumP1=0,              //sq = squared, sum = Sum, inst = instantaneous
        sqI2=0,sumI2=0,instP2=0,sumP2=0,              //sq = squared, sum = Sum, inst = instantaneous
        sqI3=0,sumI3=0,instP3=0,sumP3=0;              //sq = squared, sum = Sum, inst = instantaneous

    int startV=0;                                       //Instantaneous voltage at start of sample window.
bool lastVCross=0, checkVCross=0;  

     
      #ifdef USE_ESP32
        adc_atten_t attenuation_{ADC_ATTEN_DB_12};
        adc_channel_t channel[4];
        uint8_t channels[10];
        uint32_t values[10]={0};
        bool autorange_{false};
      #endif  // USE_ESP32
    };
    
    
} // namespace adc_continous
} // namespace esphome