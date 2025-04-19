#include "esphome/components/network/util.h"
#include "esphome/core/log.h"
#include "Rigol.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h" // Include GPIO driver

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#define S(X) X, sizeof(X) - 1
#define LED_PIN GPIO_NUM_2 // Define the GPIO pin for the LED (adjust as needed)
#define xstr(a) STR(a)
#define STR(a) #a
namespace esphome
{
  namespace rigol
  {
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



    void Rigol::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up Rigol...");
      // Initialize the LED
      setup_led();
      server_->add_tcp_callback([this](const std::string &data, socket::Socket *sock) {
        ESP_LOGD(TAG, "Received data: %s", data.c_str());
        std::string response = this->parse(data, sock);
        if (!response.empty())
        {
          ESP_LOGD(TAG, "Sending response: %s", response.c_str());
          sock->write(response.c_str(), response.size());
        }
      });
      adc_sensor_->add_adc_callback([this](const char *data) {
        ESP_LOGD(TAG, "ADC data: %c", data[0]);
        // Handle ADC data here
      });
    }
    std::stringstream ss;



    void Rigol::loop()
    {
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
      case str2int(":TIM:SCAL?"): return std::to_string(100.0/adc_sensor_->get_frequency());
      case str2int(":CHAN1:PROB?"): return "1";
      case str2int(":CHAN2:PROB?"): return "1";
      case str2int(":TRIG:STAT?"): return block++ ? "AUTO" : "STOP";
      case str2int(":WAV:STAT?"): return "IDLE,"+std::to_string(adc_sensor_->get_sample_count());
      case str2int("*OPC?"): return "1";
      case str2int(":CHAN1:SCAL?"): return "1";
      case str2int(":CHAN2:SCAL?"): return "1";
      case str2int(":CHAN1:OFFS?"): return "0";
      case str2int(":CHAN2:OFFS?"): return "0";
      case str2int(":CHAN1:COUP?"):
      case str2int(":CHAN2:COUP?"): return "DC";
      case str2int(":WAV:YINC?"): return std::to_string(3.3/256);;
      case str2int("WAV:XINC?"): return std::to_string(1.0/adc_sensor_->get_frequency());
      case str2int(":WAV:YOR?"): return "0";
      case str2int(":WAV:YREF?"): return "-1.27";
      case str2int(":WAV:BEG"): 
      case str2int(":RUN"): 
      adc_sensor_->start();
        return "";
      case str2int(":STOP"): 
      adc_sensor_->stop();
        return "";
      case str2int(":TRIG:EDGE:SLOP?"): return "POS";
      case str2int(":WAV:DATA?"):
      {
        ESP_LOGD(TAG, "received: %s", data.c_str());
        sock->write(S("#41400"));
        sock->write(adc_sensor_->get_buffer(), adc_sensor_->get_sample_count());
        sock->write(S("\n"));
        
        return "";
      }
      default:
        ESP_LOGW(TAG, "Unknown command: %s", data.c_str());
        return data.back() == '?' ? "0" : "";
      }
    }

  }
}