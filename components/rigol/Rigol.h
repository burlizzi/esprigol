#pragma once
#include <sstream>
#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/adc_continous/adc_sensor.h"
#include "esphome/components/tcp_server/TcpServer.h"
namespace esphome
{
    namespace rigol
    {
        static const char *const TAG = "Rigol";
        class Rigol : public Component
        {
        public:
            Rigol(tcp_server::TcpServer* server,adc_continous::ADCContinuousSensor *adc_sensor):server_(server),adc_sensor_(adc_sensor){};
            void setup() override;
            float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
            void loop() override;
            void dump_config() override;
            std::string parse(const std::string &data,socket::Socket *sock);
        protected:
            tcp_server::TcpServer* server_;
            adc_continous::ADCContinuousSensor *adc_sensor_;
        };
    }
}