#include "adc_sensor.h"
#include "esphome/core/component.h"
#ifdef USE_OTA_STATE_CALLBACK
#include "esphome/components/ota/ota_backend.h"
#endif
#include "esp_adc/adc_monitor.h"
#include "esp_adc/adc_filter.h"

namespace esphome {
namespace adc_continous {



double  filteredV1=0,filteredV2=0,filteredV3=0; // Filtered_ is the raw analog value minus the DC offset
uint32_t samples = 0;
unsigned int crossCount = 0; // Used to measure number of times threshold is crossed.
unsigned int numberOfSamples = 0;
uint32_t lastv=0;
uint32_t lasti=0;
uint32_t unknown = 0;
uint16_t maxv = 0;
uint16_t minv = 4096;


uint16_t ADCContinuousSensor::analogRead(uint8_t pin) { return values[pin] / values[pin + 4]; }




inline adc_digi_output_data_t *getNext(adc_digi_output_data_t *data, adc_digi_output_data_t *end,
                                       uint8_t channel) {
  while (data->type1.channel != channel) {
    data++;
    if (data >= end)
      return nullptr;
  }
  return data;
}

void ADCContinuousSensor::loop() {
  uint32_t len = 0;
  if( adc_continuous_read(adc_handle, buffer_, sample_count_ * SOC_ADC_DIGI_RESULT_BYTES * 4, &len, 0) != ESP_OK ) {
   return;
  }

  calcIV(len, buffer_);
  set_->calc_count++;
  set_->avgp1 += realPower1;
  set_->avgp2 += realPower2;
  set_->avgp3 += realPower3;
  adc_callback_.call(realPower1+ realPower2+ realPower3);

}

//uint8_t udpkt[1500];
//int pktsize=0;

void IRAM_ATTR ADCContinuousSensor::calcIV(uint32_t len, uint8_t *buffer) {
  constexpr double PHASECAL1 = 1.7; // Phase calibration value
  constexpr double PHASECAL2 = 1.7;
  constexpr double PHASECAL3 = 1.7;
  constexpr double VCAL = 365.286779661;    // Voltage calibration constant
  constexpr const unsigned int SupplyVoltage = 3300; // mV
  constexpr const unsigned int ADC_COUNTS = 4096;
  double sumV = 0;
  numberOfSamples = 0;
  constexpr const unsigned int crossings = 20;

  maxv = 0;
  minv = 4096;

  auto datav1 = ((adc_digi_output_data_t *)buffer)+phaseCal1;
  auto datav2 = ((adc_digi_output_data_t *)buffer)+phaseCal2;
  auto datav3 = ((adc_digi_output_data_t *)buffer)+phaseCal3;

  auto datai1 = (adc_digi_output_data_t *)buffer;
  auto datai2 = (adc_digi_output_data_t *)buffer;
  auto datai3 = (adc_digi_output_data_t *)buffer;
  auto end = ((adc_digi_output_data_t *)buffer) + len / SOC_ADC_DIGI_RESULT_BYTES;

  //uint32_t sumPA_CT[2]={0,0};
  //uint32_t sumPB_CT[2]={0,0};



  crossCount = 0;
  datav1 = getNext(datav1, end, channel[0]);
  startV = datav1->type1.data; // initial voltage for threshold detection

  while ((crossCount < crossings)) {
    lastFilteredV1 = filteredV1;
    lastFilteredV2 = filteredV2;
    lastFilteredV3 = filteredV3;

    // A) Read raw voltage and current samples
    datav1 = getNext(datav1, end, channel[0]);
    datav2 = getNext(datav2, end, channel[0]);
    datav3 = getNext(datav3, end, channel[0]);

    datai1 = getNext(datai1, end, channel[1]);
    datai2 = getNext(datai2, end, channel[2]);
    datai3 = getNext(datai3, end, channel[3]);
    if ( !datav1 ||!datav2 ||!datav3 || !datai1 || !datai2 || !datai3)
      break;


    auto sampleV1 = datav1->type1.data;
    auto sampleV2 = datav2->type1.data;
    auto sampleV3 = datav3->type1.data;
    auto sampleI1 = datai1->type1.data;
    auto sampleI2 = datai2->type1.data;
    auto sampleI3 = datai3->type1.data;
    //pktsize=(numberOfSamples+1)*6;
    
   
    datav1++;
    datav2++;
    datav3++;
    datai1++;
    datai2++;
    datai3++;

    // B) Apply digital low pass filters to extract DC offset, then subtract
    offsetV = offsetV + ((sampleV1 - offsetV) / 1024);
    filteredV1 = sampleV1 - offsetV;
    filteredV2 = sampleV2 - offsetV;
    filteredV3 = sampleV3 - offsetV;
    offsetI1 = offsetI1 + ((sampleI1 - offsetI1) / 1024);
    auto filteredI1 = sampleI1 - offsetI1;
    offsetI2 = offsetI2 + ((sampleI2 - offsetI2) / 1024);
    auto filteredI2 = sampleI2 - offsetI2;
    offsetI3 = offsetI3 + ((sampleI3 - offsetI3) / 1024);
    auto filteredI3 = sampleI3 - offsetI3;



    /*udpkt[(numberOfSamples)*6+0]=filteredV1/16;
    udpkt[(numberOfSamples)*6+1]=filteredI1/16;
    udpkt[(numberOfSamples)*6+2]=filteredV2/16;
    udpkt[(numberOfSamples)*6+3]=filteredI2/16;
    udpkt[(numberOfSamples)*6+4]=filteredV3/16;
    udpkt[(numberOfSamples)*6+5]=filteredI3/16;*/

    numberOfSamples++;

    // C) RMS voltage
    sumV += filteredV1 * filteredV1;


    // D) RMS current
    sqI1 = filteredI1 * filteredI1;
    sumI1 += sqI1;
    sqI2 = filteredI2 * filteredI2;
    sumI2 += sqI2;
    sqI3 = filteredI3 * filteredI3;
    sumI3 += sqI3;

    // E) Phase calibration
    phaseShiftedV1 = lastFilteredV1 + PHASECAL1 * (filteredV1 - lastFilteredV1);
    phaseShiftedV2 = lastFilteredV2 + PHASECAL2 * (filteredV2 - lastFilteredV1);
    phaseShiftedV3 = lastFilteredV3 + PHASECAL3 * (filteredV3 - lastFilteredV1);

    // F) Instantaneous power calc
    instP1 = phaseShiftedV1 * filteredI1;
    sumP1 += instP1;
    instP2 = phaseShiftedV2 * filteredI2;
    sumP2 += instP2;
    instP3 = phaseShiftedV3 * filteredI3;
    sumP3 += instP3;

    // G) Threshold crossing detection
    lastVCross = checkVCross;
    if (sampleV1 > startV)
      checkVCross = true;
    else
      checkVCross = false;
    if (numberOfSamples == 1)
      lastVCross = checkVCross;

    if (lastVCross != checkVCross)
      crossCount++;
  }

  // Post loop calculations
  double V_RATIO = VCAL * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  double I_RATIO1 = ICAL1 * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Irms1 = I_RATIO1 * sqrt(sumI1 / numberOfSamples);
  double I_RATIO2 = ICAL2 * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Irms2 = I_RATIO2 * sqrt(sumI2 / numberOfSamples);
  double I_RATIO3 = ICAL3 * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Irms3 = I_RATIO3 * sqrt(sumI3 / numberOfSamples);

  realPower1 = V_RATIO * I_RATIO1 * sumP1 / numberOfSamples;
  apparentPower1 = Vrms * Irms1;
  powerFactor1 = realPower1 / apparentPower1;

  realPower2 = V_RATIO * I_RATIO2 * sumP2 / numberOfSamples;
  apparentPower2 = Vrms * Irms2;
  powerFactor2 = realPower2 / apparentPower2;

  realPower3 = V_RATIO * I_RATIO3 * sumP3 / numberOfSamples;
  apparentPower3 = Vrms * Irms3;
  powerFactor3 = realPower3 / apparentPower3;

  // Reset accumulators
  sumI1 = 0;
  sumP1 = 0;
  sumI2 = 0;
  sumP2 = 0;
  sumI3 = 0;
  sumP3 = 0;
}

void ADCContinuousSensor::update() {
  //ESP_LOGI(TAG, "samples= %d overflow=%d cross=%d", numberOfSamples, overflow_count,crossCount);
  if (udp_) {
    //udp_->send_packet(udpkt, pktsize);
  }

  this->publish_state(sample());
  std::swap(this->set_, this->get_);
}

#if SOC_ADC_MONITOR_SUPPORTED
bool example_on_exceed_high_thresh(adc_monitor_handle_t monitor_handle,
                                   const adc_monitor_evt_data_t *event_data, void *user_data) {
  ESP_LOGW(TAG, "ADC Monitor: Exceeded high threshold!");
  return true;
}
bool example_on_below_low_thresh(adc_monitor_handle_t monitor_handle,
                                 const adc_monitor_evt_data_t *event_data, void *user_data) {
  ESP_LOGW(TAG, "ADC Monitor: Below low threshold!");
  return true;
}
#endif

void ADCContinuousSensor::setup() {
#ifdef USE_OTA_STATE_CALLBACK
  ota::get_global_ota_callback()->add_on_state_callback([this](ota::OTAState state, float progress,
                                                               uint8_t error, ota::OTAComponent *comp) {
    if (state == ota::OTA_STARTED) {
      this->stop();
    }
  });
#endif

  constexpr const int numChannels = 4;

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
      .sample_freq_hz = samplefreq,
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
//      .on_conv_done = ADCContinuousSensor::callback,
//      .on_pool_ovf = overflow,
  };
  if (adc_continuous_register_event_callbacks(adc_handle, &cb_config, this) != ESP_OK) {
    ESP_LOGW(TAG, "ADC Callback failed");
  }

#if SOC_ADC_MONITOR_SUPPORTED
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

  start();
}

void ADCContinuousSensor::dump_config() {}

void ADCContinuousSensor::start() {
  if (adc_continuous_start(adc_handle) != ESP_OK) {
    ESP_LOGW(TAG, "ADC Start failed");
  }
}

void ADCContinuousSensor::stop() {
  if (adc_continuous_stop(adc_handle) != ESP_OK) {
    ESP_LOGW(TAG, "ADC Start failed");
  }
}

float ADCContinuousSensor::get_setup_priority() const { return setup_priority::HARDWARE; }

void ADCContinuousSensor::data(char *data) {
  // Sample logic here
  // this->adc_callback_.call(buffer_);
}

float ADCContinuousSensor::sample() {
  //auto ret=avgPower / calc_count;
  lastRealPower1=get_->avgp1 / get_->calc_count;
  lastRealPower2=get_->avgp2 / get_->calc_count;
  lastRealPower3=get_->avgp3 / get_->calc_count;
  get_->calc_count = 0;
  get_->avgp1 = 0;
  get_->avgp2 = 0;
  get_->avgp3 = 0;
  return lastRealPower1+lastRealPower2+lastRealPower3;

}

} // namespace adc_continous
} // namespace esphome