#include "adc_sensor.h"
#include "esphome/core/component.h"
#ifdef USE_OTA
#include "esphome/components/ota/ota_backend.h"
#endif

namespace esphome
{
  namespace adc_continous
  {
    void ADCContinuousSensor::update()
    {
      // Update logic here
    }

    bool IRAM_ATTR ADCContinuousSensor::callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
    {
      ADCContinuousSensor *component = (ADCContinuousSensor *)user_data;
      TaskHandle_t task_handle = component->adc_task_handle;
      BaseType_t mustYield = pdFALSE;
      
      for (int i = 0; i < edata->size; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)(void*)&(edata->conv_frame_buffer[i]);
#if (SOC_ADC_DIGI_RESULT_BYTES == 2)
        component->buffer_[i/SOC_ADC_DIGI_RESULT_BYTES] = p->type1.data;
#else
component->buffer_[i/SOC_ADC_DIGI_RESULT_BYTES] =  p->type2.data;
#endif
      }


      vTaskNotifyGiveFromISR(task_handle, &mustYield);
      return (mustYield == pdTRUE);
    }

    void ADCContinuousSensor::adc_task(void *param)
    {
      ADCContinuousSensor *component = (ADCContinuousSensor *)param;
      while (1)
      {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        component->data(NULL);
      }
      vTaskDelete(NULL);
    }

    static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
    {
      adc_cali_handle_t handle = NULL;
      esp_err_t ret = ESP_FAIL;
      bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
      if (!calibrated)
      {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
          calibrated = true;
        }
      }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
      if (!calibrated)
      {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
          calibrated = true;
        }
      }
#endif

      *out_handle = handle;
      if (ret == ESP_OK)
      {
        ESP_LOGI(TAG, "Calibration Success");
      }
      else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
      {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
      }
      else
      {
        ESP_LOGE(TAG, "Invalid arg or no memory");
      }

      return calibrated;
    }

    void ADCContinuousSensor::setup()
    {
      #ifdef USE_OTA
        ota::get_global_ota_callback()->add_on_state_callback(
          [this](ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
            if (state == ota::OTA_STARTED) {
              this->stop();
            }
          });
      #endif

      const int numChannels = 1; // Number of ADC channels to be used

      // handle configuration
      adc_continuous_handle_cfg_t handle_config = {
          .max_store_buf_size = sample_count_ * 2 * SOC_ADC_DIGI_RESULT_BYTES,
          .conv_frame_size = sample_count_ * SOC_ADC_DIGI_RESULT_BYTES,
      };
      ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &adc_handle));

      // ADC Configuration with Channels
      adc_continuous_config_t adc_cnfig = {
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
      for (int i = 0; i < numChannels; i++)
      {
        channel_config[i].channel = channel;
        channel_config[i].atten = attenuation_;
        channel_config[i].bit_width = ADC_BITWIDTH_12;
        channel_config[i].unit = ADC_UNIT_1;
      }
      adc_cnfig.adc_pattern = channel_config;
      if (adc_continuous_config(adc_handle, &adc_cnfig) != ESP_OK)
      {
        ESP_LOGW(TAG, "ADC Configuration failed");
      }
      adc_cali_handle_t adc1_cali_chan0_handle = NULL;

      example_adc_calibration_init(ADC_UNIT_1, channel, attenuation_, &adc1_cali_chan0_handle);
#ifndef CONFIG_FREERTOS_UNICORE
      xTaskCreatePinnedToCore(ADCContinuousSensor::adc_task, "adc_thread",
                              10000,            // stack size (in words)
                              this,             // input params
                              1,                // priority
                              &adc_task_handle, // Handle, not needed
                              1                 // core
      );
#else
      xTaskCreate(ADCContinuousSensor::adc_task, "adc_thread",
                  10000,           // stack size (in words)
                  this,            // input params
                  1,               // priority
                  &adc_task_handle // Handle, not needed
      );
#endif
      // Callback Configuration
      adc_continuous_evt_cbs_t cb_config = {
          .on_conv_done = ADCContinuousSensor::callback,
      };
      if (adc_continuous_register_event_callbacks(adc_handle, &cb_config, this) != ESP_OK)
      {
        ESP_LOGW(TAG, "ADC Callback failed");
      }

    }
    void ADCContinuousSensor::dump_config()
    {
    }
    void ADCContinuousSensor::start()
    {
      if(adc_continuous_start(adc_handle)!=ESP_OK)
      {
        ESP_LOGW(TAG, "ADC Start failed");
      }
    }
    void ADCContinuousSensor::stop()
    {
      if(adc_continuous_stop(adc_handle)!=ESP_OK)
      {
        ESP_LOGW(TAG, "ADC Start failed");
      }
    }

    float ADCContinuousSensor::get_setup_priority() const
    {
      return setup_priority::HARDWARE;
    }

    void ADCContinuousSensor::data(char* data)
    {
      // Sample logic here
      ESP_LOGD(TAG, "ADC data received");
      this->adc_callback_.call(buffer_);
    }

    float ADCContinuousSensor::sample()
    {
      // Sample logic here
      return 0.0f;
    }

  } // namespace adc_continous

} // namespace esphome
