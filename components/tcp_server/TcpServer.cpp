#include "esphome/components/network/util.h"
#include "esphome/core/log.h"
#include "TcpServer.h"

#include "esphome/components/network/util.h"
#include "esphome/core/log.h"



#define S(X) X, sizeof(X) -1
namespace esphome
{
  namespace tcp_server
  {

    void TcpServer::setup()
    {
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



    void TcpServer::tcp_task(void *param)
    {
      TcpServer *tcp_server = static_cast<TcpServer *>(param);
      std::unique_ptr<socket::Socket> client_(tcp_server->shared_socket);
      char data;
      int len = 0;
      while ((len = client_->read(&data, 1)) > 0)
      {
        if (data == '\n')
        {
          ESP_LOGD(TAG, "rec: %s", tcp_server->ss_.str().c_str());
          auto resp = tcp_server->parse(tcp_server->ss_.str(),client_.get());
            ESP_LOGD(TAG, "Sending: %s", resp.c_str());
          resp+= "\0";
          client_->write(resp.c_str(), resp.length());
          tcp_server->ss_.str(std::string()); // clear the stringstream
          tcp_server->ss_.clear();            // clear the error state
        }
        else
        tcp_server->ss_ << data;
      }
      client_->close();
      ESP_LOGD(TAG, "Client disconnected %s", client_->getpeername().c_str());
/*      if(adc_continuous_stop(adc_handle)!=ESP_OK)
        {
          ESP_LOGW(TAG, "ADC Stop failed");
        }*/
        tcp_server->shared_socket=nullptr;
    //  vTaskNotifyGiveFromISR(tcp_server->s_task_handle, NULL);
        vTaskDelete(NULL);
    }

    void TcpServer::loop()
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
          xTaskCreatePinnedToCore(tcp_task, "tcp_thread",
            10000,   // stack size (in words)
            this,    // input params
            1,       // priority
            nullptr, // Handle, not needed
            1        // core
          );
          }
        else
        {
#endif
          xTaskCreate(tcp_task, "tcp_thread",
                      10000,  // stack size (in words)
                      this,   // input params
                      1,      // priority
                      nullptr // Handle, not needed
          );
#ifndef CONFIG_FREERTOS_UNICORE
        }
#endif

        ESP_LOGD(TAG, "Accepted %s", shared_socket->getpeername().c_str());
      }
    }

    void TcpServer::dump_config()
    {
      ESP_LOGCONFIG(TAG, "TcpServer: ");
    }

    constexpr unsigned int str2int(const char* str, int h = 0)
    {
        return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
    }

    std::string TcpServer::parse(const std::string &data,socket::Socket *sock)
    {
      ESP_LOGD(TAG, "received: %s", data.c_str());
      this->tcp_callback_.call( data,sock);

      return "";
    }

  }
}