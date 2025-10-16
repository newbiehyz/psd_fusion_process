#pragma once

#include <iostream>
#include <string>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <vector>
#include <thread>
#include <chrono>
#include <sys/stat.h>
#include <sys/mman.h>
#include <chrono>

#include "c_sdk/log/log.h"

namespace patac {
namespace psd {
namespace aux {

enum class UdpErrorNo: uint8_t {
  UDP_STAT_DEFAULT = 0xFF,
  UDP_STAT_OK = 0x00,
  UDP_STAT_ESTABLISH_FAIL = 0x01,
  UDP_STAT_BIND_FAIL = 0x02,
  UDP_STAT_SETFL_FAIL = 0x03,
  UDP_STAT_SEND_FAIL = 0x04,
  UDP_STAT_UNKNOWN_FAIL = 0x99,
};

class UdpSender {
 private:
  int sockfd;
  struct sockaddr_in server_addr;
  socklen_t server_len;
  uint32_t send_suc = 0;
  uint32_t send_fail = 0;
  
 public:
  std::string PrintErrNo(UdpErrorNo err_no) {
    switch (err_no) {
      case UdpErrorNo::UDP_STAT_OK:               { return "UDP_STAT_OK";             break;}
      case UdpErrorNo::UDP_STAT_ESTABLISH_FAIL:   { return "UDP_STAT_ESTABLISH_FAIL"; break;}
      case UdpErrorNo::UDP_STAT_BIND_FAIL:        { return "UDP_STAT_BIND_FAIL";      break;}
      case UdpErrorNo::UDP_STAT_SETFL_FAIL:       { return "UDP_STAT_SETFL_FAIL";      break;}
      case UdpErrorNo::UDP_STAT_SEND_FAIL:        { return "UDP_STAT_SEND_FAIL";      break;}
      default:                                    { return "UDP_STAT_DEFAULT";        break;}
    }
    return "";
  }

 public:
  UdpSender() = default;

  bool setNonBlocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
      std::string str_err = strerror(errno);
      LOGI(" [ UdpSender error]: %s", str_err.c_str());
      return false;
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
      std::string str_err = strerror(errno);
      LOGI(" [ UdpSender error]: %s", str_err.c_str());
      return false;
    }
    return true;
  }

  bool Initialize(const std::string& server_ip, uint16_t port, UdpErrorNo& error_no) {
    error_no = UdpErrorNo::UDP_STAT_DEFAULT;

    // 创建 UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0); 
    if (sockfd < 0) {
      error_no = UdpErrorNo::UDP_STAT_ESTABLISH_FAIL;
      std::string str_err = strerror(errno);
      LOGI(" [ UdpSender error]: %s", str_err.c_str());
      return false;
    }   
    
    // 设置为非阻塞模式
    if (!setNonBlocking(sockfd)) {
      error_no = UdpErrorNo::UDP_STAT_SETFL_FAIL;
      std::string str_err = strerror(errno);
      LOGI(" [ UdpReceive error]: %s", str_err.c_str());
      return false;
    }   
    
    // 设置服务器地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr);

    server_len = sizeof(server_addr);

    error_no = UdpErrorNo::UDP_STAT_OK;
    return true;
  }

  bool SendNonBlock(char* data_ptr, int sz, UdpErrorNo& error_no) {
    ssize_t bytes_sent = sendto(sockfd, data_ptr, sz, 0,
                                (const struct sockaddr *)&server_addr,
                                server_len);

    if (bytes_sent < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // 发送缓冲区满，等待后重试
        std::string str_err = strerror(errno);
        LOGI(" [ UdpSender error]: %s", str_err.c_str());
        error_no = UdpErrorNo::UDP_STAT_SEND_FAIL;
      } else {
        error_no = UdpErrorNo::UDP_STAT_UNKNOWN_FAIL;
      }
      send_fail++;
      return false;
    } else {
      send_suc++;
      LOGI(" [ UdpSender Success]: %d Bytes", bytes_sent);
      return true;
    }
  }

  static int64_t get_microseconds_timestamp() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
  }

};  // class UdpSender
}   // namespace aux
}   // namespace psd
}   // namespace patac