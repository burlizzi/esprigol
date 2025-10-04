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
            void setup() ;
            float get_setup_priority() const  { return setup_priority::AFTER_WIFI; }
            void loop() ;
            void dump_config() ;
            virtual std::string parse(const std::string &data,socket::Socket *sock);
            void add_tcp_callback(std::function<void(const std::string&,socket::Socket *)> &&callback) {
                this->tcp_callback_.add(std::move(callback));
              }
            
        protected:
            static void tcp_task(void *params);
            std::unique_ptr<socket::Socket> socket_ = nullptr;
            socket::Socket *shared_socket = nullptr;
            std::stringstream ss_;
            uint16_t port_;
            CallbackManager<void(const std::string&,socket::Socket *)> tcp_callback_{};
        };
    }
}