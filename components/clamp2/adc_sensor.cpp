#include "adc_sensor.h"
#include "esphome/core/component.h"
#include "esp_adc/adc_monitor.h"
#include "esp_adc/adc_filter.h"
#include "esphome/core/application.h"

namespace esphome {
namespace adc_continous {





//uint16_t ADCContinuousSensor::analogRead(uint8_t pin) { return values[pin] / values[pin + 4]; }



bool IRAM_ATTR ADCContinuousSensor::callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    Application::wake_loop_isrsafe(&mustYield);

    return (mustYield == pdTRUE);
}


bool low=false;
bool high=false;


void ADCContinuousSensor::loop() {
  uint32_t len = 0;
  if (!started_) {
    return;
  }
  if( adc_continuous_read(adc_handle, buffer_, sample_count_ * SOC_ADC_DIGI_RESULT_BYTES * 4, &len, 0) != ESP_OK ) {
    //ESP_LOGW(TAG, "ADC Read failed");
   return;
  }
  const int numPins = vPins_+cPins_;

  const int n=len/SOC_ADC_DIGI_RESULT_BYTES/numPins;
  int32_t avg[numPins] = {0};
  float max[numPins] = {0};
  float min[numPins] = {0};
  float values[numPins] = {0};
  unsigned int zerocrossings[numPins]={0};
  float prevValues[numPins]={0};

  uint16_t volts[n]={0};
  #ifdef USE_UDP
  uint8_t udpkt[n*numPins];
  int pktsize=0;
  #endif
  #if (SOC_ADC_DIGI_RESULT_BYTES == 2)
    #define ADC_TYPE type1
  #else
    #define ADC_TYPE type2
  #endif

  //calculate averages
  for (int i = 0; i < len; i += SOC_ADC_DIGI_RESULT_BYTES) {
    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&buffer_[i];
    auto pin=channels[p->ADC_TYPE.channel];
    avg[pin]+=p->ADC_TYPE.data;
    if(p->ADC_TYPE.data>max[pin])
      max[pin]=p->ADC_TYPE.data;
    if(p->ADC_TYPE.data<min[pin])
      min[pin]=p->ADC_TYPE.data;
    if(pin==0)
      volts[i/SOC_ADC_DIGI_RESULT_BYTES/numPins]=p->ADC_TYPE.data;

  }
  for(int i=0;i<numPins;i++)
  {
    avg[i]/=n;
    min[i]-=avg[i];
    max[i]-=avg[i];
    min[i]/=10240.0;//10% of 1024, to avoid noise around zero
    max[i]/=10240.0;//10% of 1024, to avoid noise around zero
  }

  float sumP[cPins_] = {0};
  


  //we find zerocrossings
  for (int i = 0; i < len; i += SOC_ADC_DIGI_RESULT_BYTES) {
    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&buffer_[i];
    auto pin=channels[p->ADC_TYPE.channel];
    auto value=(((float)p->ADC_TYPE.data)-avg[pin])/1024.0;
    if (prevValues[pin]<min[pin] && value>max[pin]) 
    {
      zerocrossings[pin]=i/SOC_ADC_DIGI_RESULT_BYTES/numPins;
    }
    prevValues[pin]=value;
  }  

  for (int i = 0; i < len; i += SOC_ADC_DIGI_RESULT_BYTES*numPins) {
    for(int j=0;j<4*SOC_ADC_DIGI_RESULT_BYTES;j+= SOC_ADC_DIGI_RESULT_BYTES)
    {
      adc_digi_output_data_t *p = (adc_digi_output_data_t*)&buffer_[i+j];
      auto pin = channels[p->ADC_TYPE.channel];
      auto value=(((float)p->ADC_TYPE.data)-avg[pin])/1024.0;
      if(pin<numPins)
      {
        values[pin] = value;
      }  
      else
      {
        ESP_LOGW(TAG, "Unknown channel %d with value %d", p->ADC_TYPE.channel, p->ADC_TYPE.data);
      }

    }

    //auto V1=(sinTable[(int)(i/SOC_ADC_DIGI_RESULT_BYTES/numPins +n- zerocrossings[1]) % n]);
    
    auto V1=(volts[(int)(i/SOC_ADC_DIGI_RESULT_BYTES/numPins + phaseShifts[0]*n) % n]-avg[0])/1024.0;
    auto V2=(volts[(int)(i/SOC_ADC_DIGI_RESULT_BYTES/numPins + phaseShifts[1]*n) % n]-avg[0])/1024.0;
    auto V3=(volts[(int)(i/SOC_ADC_DIGI_RESULT_BYTES/numPins + phaseShifts[2]*n) % n]-avg[0])/1024.0;
    sumP[0] += ((max[1]-min[1])>0.2?V1:values[1])*values[1];
    sumP[1] += V2*values[2];
    sumP[2] += V3*values[3];
  #ifdef USE_UDP
    udpkt[pktsize++]=(V2+0.5)*256;
    udpkt[pktsize++]=(values[1]+0.5)*256;
    udpkt[pktsize++]=(values[2]+0.5)*256;
    udpkt[pktsize++]=(values[3]+0.5)*256;
  #endif
  #undef ADC_TYPE
  }
  
  
  ESP_LOGV(TAG, "p: %f %f %f", sumP[0]/n, sumP[1]/n, sumP[2]/n);
  sumPower[0]+=(sumP[0]*calibrationIs[0]/(float)n);
  sumPower[1]+=(sumP[1]*calibrationIs[1]/(float)n);
  sumPower[2]+=(sumP[2]*calibrationIs[2]/(float)n);
  cycles++;
  adc_callback_.call(( sumP[0] + sumP[1] + sumP[2])/n);

  static uint32_t lastSend=0;
  if(millis()-lastSend>1000)
  {
    ESP_LOGD(TAG, "avg: %d %d %d %d", avg[0], avg[1], avg[2], avg[3]);
    ESP_LOGD(TAG, "min: %f max: %f low: %s high: %s zero:%d", min[1], max[1], low ? "true" : "false", high ? "true" : "false",zerocrossings[1]);
    low=false;
    high=false;
#ifdef USE_UDP
    if (udp_) {
      lastSend=millis();
      udp_->send_packet(udpkt, pktsize);
      pktsize=0;
    }
#endif  
  }


}


void ADCContinuousSensor::update() {
  this->publish_state(sample());
}

#if SOC_ADC_MONITOR_SUPPORTED1
bool example_on_exceed_high_thresh(adc_monitor_handle_t monitor_handle,
                                   const adc_monitor_evt_data_t *event_data, void *user_data) {
  high=true;
  return true;
}
bool example_on_below_low_thresh(adc_monitor_handle_t monitor_handle,
                                 const adc_monitor_evt_data_t *event_data, void *user_data) {
  low=true;
  return true;
}
#endif



#ifdef USE_OTA_STATE_LISTENER
void ADCContinuousSensor::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
      this->stop();
    }
}
#endif
void ADCContinuousSensor::setup() {

  #ifdef USE_OTA_STATE_LISTENER
    ota::get_global_ota_callback()->add_global_state_listener(this);
  #endif

    const char numChannels = vPins_+cPins_;

    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = sample_count_ * 2 * SOC_ADC_DIGI_RESULT_BYTES * numChannels,
        .conv_frame_size = sample_count_ * SOC_ADC_DIGI_RESULT_BYTES * numChannels,
        .flags={
          .flush_pool = true,
        },
  
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &adc_handle));

    static adc_continuous_config_t adc_cnfig = {
        .pattern_num = numChannels,
        .sample_freq_hz = samplefreq*numChannels*sample_count_,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
  #if (SOC_ADC_DIGI_RESULT_BYTES == 2)
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  #else
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  #endif
    };

    adc_digi_pattern_config_t channel_config[numChannels];
    for (int i = 0; i < numChannels; i++) {
      channel_config[i].channel = channel[i];
      channel_config[i].atten = attenuation_;
      channel_config[i].bit_width = ADC_BITWIDTH_12;
      channel_config[i].unit = ADC_UNIT_1;

      adc_cali_handle_t adc1_cali_chan0_handle = NULL;
      example_adc_calibration_init(ADC_UNIT_1, channel[i], attenuation_, &adc1_cali_chan0_handle);
    }
    adc_cnfig.adc_pattern = channel_config;
    if (adc_continuous_config(adc_handle, &adc_cnfig) != ESP_OK) {
      ESP_LOGW(TAG, "ADC Configuration failed");
    }

    adc_continuous_evt_cbs_t cb_config = {
        .on_conv_done = ADCContinuousSensor::callback,
        //.on_pool_ovf = overflow,
    };
    if (adc_continuous_register_event_callbacks(adc_handle, &cb_config, this) != ESP_OK) {
      ESP_LOGW(TAG, "ADC Callback failed");
    }

  #if SOC_ADC_MONITOR_SUPPORTED1
  
    adc_monitor_handle_t adc_monitor_handle = NULL;

    adc_monitor_config_t zero_crossing_config = {
        .adc_unit = ADC_UNIT_1,
        .channel = channel[0],
        .h_threshold = 1100,
        .l_threshold = 900,
    };

    ESP_ERROR_CHECK(adc_new_continuous_monitor(adc_handle, &zero_crossing_config, &adc_monitor_handle));

    adc_monitor_evt_cbs_t zero_crossing_cbs = {
        .on_over_high_thresh = example_on_exceed_high_thresh,
        .on_below_low_thresh = example_on_below_low_thresh,
    };

    ESP_ERROR_CHECK(adc_continuous_monitor_register_event_callbacks(adc_monitor_handle, &zero_crossing_cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_monitor_enable(adc_monitor_handle));
  #endif

  if (autostart_) {
    start();
  }
}

void ADCContinuousSensor::dump_config() {}

void ADCContinuousSensor::start() {
  if (adc_continuous_start(adc_handle) != ESP_OK) {
    ESP_LOGW(TAG, "ADC Start failed");
  }
  else
  {
    started_=true;
  }
}

void ADCContinuousSensor::stop() {
  if (adc_continuous_stop(adc_handle) != ESP_OK) {
    ESP_LOGW(TAG, "ADC Start failed");
  }
  else
  {
    started_=false;
  }
}

float ADCContinuousSensor::get_setup_priority() const { return setup_priority::LATE; }

float ADCContinuousSensor::sample() {
  if (!started_) {
    return {};
  }
  float sum=0;
  for(int i=0;i<cPins_;i++)
  {
    lastRealPower[i]=sumPower[i]/cycles;
    sum+=lastRealPower[i];
    sumPower[i]=0;
  }
  cycles=0;
  return 
    lastRealPower[0]
    //sum
    ;
}

} // namespace adc_continous
} // namespace esphome
