#pragma once
#include <sstream>
#include <freertos/task.h>

#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"
namespace esphome
{
    namespace tcp_server
    {
        static const char *const TAG = "TCP_SERVER";
        class TcpServer : public virtual Component
        {
        public:
            TcpServer(uint16_t port):port_(port){};
            void setup() override;
            float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
            void loop() override;
            void dump_config() override;
            virtual std::string parse(const std::string &data,socket::Socket *sock);
        protected:
            TaskHandle_t s_task_handle;
            static void tcp_task(void *params);
            std::unique_ptr<socket::Socket> socket_ = nullptr;
            socket::Socket *shared_socket = nullptr;
            std::stringstream ss_;
            uint16_t port_;
        };
    }
}