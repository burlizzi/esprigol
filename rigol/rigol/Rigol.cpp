#include "esphome/components/network/util.h"
#include "esphome/core/log.h"
#include "Rigol.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h" // Include GPIO driver

#define S(X) X, sizeof(X) - 1
#define LED_PIN GPIO_NUM_8 // Define the GPIO pin for the LED (adjust as needed)

namespace esphome
{
  namespace rigol
  {
    socket::Socket *shared_socket = nullptr;
    bool available = false;
    adc_continuous_handle_t adc_handle = NULL;
    static TaskHandle_t s_task_handle;
    int bytes_fed=0;
    char adc_buffer[1407] = "#41400";
    float samplerate=1e-3;

    static bool IRAM_ATTR callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
    {
      TaskHandle_t task_handle = (TaskHandle_t)user_data;
      char* raw = &adc_buffer[6];
      for (int i = 0; i < edata->size; i += SOC_ADC_DIGI_RESULT_BYTES) {
          adc_digi_output_data_t *p = (adc_digi_output_data_t*)(void*)&(edata->conv_frame_buffer[i]);
  #if (SOC_ADC_DIGI_RESULT_BYTES == 2)
      raw[i/SOC_ADC_DIGI_RESULT_BYTES] = p->type1.data;
  #else
      raw[i/SOC_ADC_DIGI_RESULT_BYTES] =  p->type2.data;
  #endif
      }
      bytes_fed+=edata->size/SOC_ADC_DIGI_RESULT_BYTES;
      //sock->write(adc_buffer, 1406);
      BaseType_t mustYield = pdFALSE;
      gpio_set_level(LED_PIN, !available); // Turn off the LED
      vTaskNotifyGiveFromISR(task_handle, &mustYield);
      available = true;
      return (mustYield == pdTRUE);
    }

    void ADC_Init(adc_channel_t *channels, uint8_t numChannels)
    {
      // handle configuration
      adc_continuous_handle_cfg_t handle_config = {
          .max_store_buf_size = 2800*SOC_ADC_DIGI_RESULT_BYTES,
          .conv_frame_size = 1400*SOC_ADC_DIGI_RESULT_BYTES,
      };
      ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &adc_handle));

      // ADC Configuration with Channels
      adc_continuous_config_t adc_cnfig = {
          .pattern_num = numChannels,
          .sample_freq_hz = 1.0 / samplerate,
          .conv_mode = ADC_CONV_SINGLE_UNIT_1,
          .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
      };
      adc_digi_pattern_config_t channel_config[numChannels];
      for (int i = 0; i < numChannels; i++)
      {
        channel_config[i].channel = channels[i];
        channel_config[i].atten = ADC_ATTEN_DB_12;
        channel_config[i].bit_width = ADC_BITWIDTH_12;
        channel_config[i].unit = ADC_UNIT_1;
      }
      adc_cnfig.adc_pattern = channel_config;
      ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_cnfig));

      // Callback Configuration
      adc_continuous_evt_cbs_t cb_config = {
          .on_conv_done = callback,
      };
      TaskHandle_t task_handle =  xTaskGetCurrentTaskHandle();
      ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cb_config, task_handle));
    }

    void setup_led()
    {
      // Configure the LED pin as an output
      gpio_config_t io_conf = {};
      io_conf.intr_type = GPIO_INTR_DISABLE; // Disable interrupts
      io_conf.mode = GPIO_MODE_OUTPUT;      // Set as output mode
      io_conf.pin_bit_mask = (1ULL << LED_PIN); // Select the pin
      io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down
      io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // Disable pull-up
      gpio_config(&io_conf);

      // Turn on the LED
      gpio_set_level(LED_PIN, 1); // Set the pin to high (1)
    }

    void adc_task(void *param)
    {
      
      adc_channel_t channels[] = {
        //ADC_CHANNEL_0,
        //ADC_CHANNEL_1,
        //ADC_CHANNEL_2,
        //ADC_CHANNEL_3,
        ADC_CHANNEL_4,
        //ADC_CHANNEL_5,
        //ADC_CHANNEL_6,
        //ADC_CHANNEL_7,
      };
      ADC_Init(channels, sizeof(channels) / sizeof(adc_channel_t)); 
      ESP_LOGCONFIG(TAG, "ADC Initialized...");
      while (1)
      {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (shared_socket == nullptr)
        {
          ESP_LOGW(TAG, "Socket is null");
          continue;
        }

        if (available)
        {
          available = false;
          //ESP_LOGD(TAG, "ADC data available");
          gpio_set_level(LED_PIN, 0); // Turn on the LED
          
          gpio_set_level(LED_PIN, 1); // Turn off the LED
        }
        else
        {
          ESP_LOGD(TAG, "ADC data not available");
        }
        vTaskDelay(1);
        
      }
      adc_continuous_deinit(adc_handle);
      vTaskDelete(NULL);

    }

    void Rigol::setup()
    {

      ESP_LOGCONFIG(TAG, "Setting up Rigol...");


      #ifndef CONFIG_FREERTOS_UNICORE

      if (1)
      {
        xTaskCreatePinnedToCore(adc_task, "adc_thread",
          10000,   // stack size (in words)
          NULL,    // input params
          1,       // priority
          &s_task_handle, // Handle, not needed
          1        // core
        );
        }
      else
      {
#endif
        xTaskCreate(adc_task, "adc_thread",
          10000,  // stack size (in words)
          NULL,   // input params
          1,      // priority
          &s_task_handle // Handle, not needed
);
#ifndef CONFIG_FREERTOS_UNICORE
      }
#endif


      // Initialize the LED
      setup_led();
      gpio_set_level(LED_PIN, 1); // Turn off the LED

      socket_ = socket::socket_ip(SOCK_STREAM, 0);
      if (socket_ == nullptr)
      {
        ESP_LOGW(TAG, "Could not create socket.");
        this->mark_failed();
        return;
      }
      int enable = 1;
      int err = socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
      if (err != 0)
      {
        ESP_LOGW(TAG, "Socket unable to set reuseaddr: errno %d", err);
        // we can still continue
      }
      err = socket_->setblocking(false);
      if (err != 0)
      {
        ESP_LOGW(TAG, "Socket unable to set nonblocking mode: errno %d", err);
        this->mark_failed();
        return;
      }

      struct sockaddr_storage server;

      socklen_t sl = socket::set_sockaddr_any((struct sockaddr *)&server, sizeof(server), this->port_);
      if (sl == 0)
      {
        ESP_LOGW(TAG, "Socket unable to set sockaddr: errno %d", errno);
        this->mark_failed();
        return;
      }

      err = socket_->bind((struct sockaddr *)&server, sl);
      if (err != 0)
      {
        ESP_LOGW(TAG, "Socket unable to bind: errno %d", errno);
        this->mark_failed();
        return;
      }

      err = socket_->listen(4);
      if (err != 0)
      {
        ESP_LOGW(TAG, "Socket unable to listen: errno %d", errno);
        this->mark_failed();
        return;
      }
    }
    std::stringstream ss;


    void Rigol::tcp_task(void *param)
    {
      std::unique_ptr<socket::Socket> client_(static_cast<socket::Socket *>(param));
      char data;
      int len = 0;
      while ((len = client_->read(&data, 1)) > 0)
      {
        if (data == '\n')
        {
          ESP_LOGD(TAG, "rec: %s", ss.str().c_str());
          auto resp = parse(ss.str(),client_.get());
            ESP_LOGD(TAG, "Sending: %s", resp.c_str());
          resp+= "\0";
          client_->write(resp.c_str(), resp.length());
          ss.str(std::string()); // clear the stringstream
          ss.clear();            // clear the error state
        }
        else
          ss << data;
      }
      client_->close();
      ESP_LOGD(TAG, "Client disconnected %s", client_->getpeername().c_str());
      if(adc_continuous_stop(adc_handle)!=ESP_OK)
        {
          ESP_LOGW(TAG, "ADC Stop failed");
        }
        shared_socket=nullptr;
      vTaskNotifyGiveFromISR(s_task_handle, NULL);
        vTaskDelete(NULL);
    }

    void Rigol::loop()
    {
      if (!network::is_connected())
      {
        // when network is disconnected force disconnect immediately
        // don't wait for timeout
        return;
      }
      while (true)
      {
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        auto sock = socket_->accept((struct sockaddr *)&source_addr, &addr_len).release();
        int enable = 1;
        if (!sock)
          break;
        shared_socket = sock;
        

        int err = shared_socket->setsockopt(IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(int));
        if (err != 0)
        {
          ESP_LOGW(TAG, "Socket could not enable tcp nodelay, errno: %d", errno);
          return;
        }
        ESP_LOGW(TAG, "Socket could enable tcp nodelay, errno: %d", enable);

#ifndef CONFIG_FREERTOS_UNICORE

        if (1)
        {
          xTaskCreatePinnedToCore(Rigol::tcp_task, "tcp_thread",
            10000,   // stack size (in words)
            shared_socket,    // input params
            1,       // priority
            nullptr, // Handle, not needed
            1        // core
          );
          }
        else
        {
#endif
          xTaskCreate(Rigol::tcp_task, "tcp_thread",
                      10000,  // stack size (in words)
                      shared_socket,   // input params
                      1,      // priority
                      nullptr // Handle, not needed
          );
#ifndef CONFIG_FREERTOS_UNICORE
        }
#endif

        ESP_LOGD(TAG, "Accepted %s", shared_socket->getpeername().c_str());
      }
    }

    void Rigol::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Rigol: ");
    }

    constexpr unsigned int str2int(const char *str, int h = 0)
    {
      return !str[h] ? 5381 : (str2int(str, h + 1) * 33) ^ str[h];
    }

    std::string Rigol::parse(const std::string &data,socket::Socket *sock)
    {
      static char block = 0;
      // ESP_LOGD(TAG, "received: %s", data.c_str());
      switch (str2int(data.c_str()))
      {
      case str2int("*IDN?"): return "Rigol Technologies,MSO2302A,DS1EXXXXXXXXXX,00.02.05.02.00";
      case str2int(":CHAN1:DISP?"): return "1";
      case str2int(":CHAN2:DISP?"): return "0";
      case str2int(":LA:STAT?"): return "1";
      case str2int(":LA:DIG0:DISP?"):
      case str2int(":LA:DIG1:DISP?"):
      case str2int(":LA:DIG2:DISP?"):
      case str2int(":LA:DIG3:DISP?"):
      case str2int(":LA:DIG4:DISP?"):
      case str2int(":LA:DIG5:DISP?"):
      case str2int(":LA:DIG6:DISP?"):
      case str2int(":LA:DIG7:DISP?"):
      case str2int(":LA:DIG8:DISP?"):
      case str2int(":LA:DIG9:DISP?"):
      case str2int(":LA:DIG10:DISP?"):
      case str2int(":LA:DIG11:DISP?"):
      case str2int(":LA:DIG12:DISP?"):
      case str2int(":LA:DIG13:DISP?"):
      case str2int(":LA:DIG14:DISP?"):
      case str2int(":LA:DIG15:DISP?"): return "0";
      case str2int(":TIM:SCAL?"): return std::to_string(samplerate);
      case str2int(":CHAN1:PROB?"): return "1";
      case str2int(":CHAN2:PROB?"): return "1";
      case str2int(":TRIG:STAT?"): return block++ ? "AUTO" : "STOP";
      case str2int(":WAV:STAT?"): return "IDLE,"+std::to_string(bytes_fed);
      case str2int("*OPC?"): return "1";
      case str2int(":CHAN1:SCAL?"): return "1";
      case str2int(":CHAN2:SCAL?"): return "1";
      case str2int(":CHAN1:OFFS?"): return "0";
      case str2int(":CHAN2:OFFS?"): return "0";
      case str2int(":CHAN1:COUP?"):
      case str2int(":CHAN2:COUP?"): return "DC";
      case str2int(":WAV:YINC?"): return "0.01";
      case str2int(":WAV:YOR?"): return "127";
      case str2int(":WAV:YREF?"): return "256";
      case str2int(":WAV:BEG"): 
      case str2int(":RUN"): bytes_fed=0; return "";
      case str2int(":STOP"): bytes_fed=0;if(adc_continuous_stop(adc_handle)!=ESP_OK)
        {
          ESP_LOGW(TAG, "ADC Stop failed");
        }

        return "";

      case str2int(":TRIG:EDGE:SLOP?"): return "POS";

      case str2int(":WAV:DATA?"):
      {
        if(adc_continuous_start(adc_handle)!=ESP_OK)
        {
          ESP_LOGW(TAG, "ADC Start failed");
        }
        ESP_LOGD(TAG, "received: %s", data.c_str());
        shared_socket->write(adc_buffer, sizeof(adc_buffer));
        return "";
      }
      default:
        ESP_LOGW(TAG, "Unknown command: %s", data.c_str());
        return data.back() == '?' ? "0" : "";
      }
    }

  }
}