/**
  * Copyright (C) 2019 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING, Xu WANG
  * Description: This file used to define the udp input
  */
#ifndef _INPUT_H_
#define _INPUT_H_

#include <unistd.h>
#include <string>
#include <netinet/in.h>
#include <vector>
#include <pcap.h>

namespace itd_lidar {

namespace lidar_driver {

enum StatusTable {
  PCAP_DUMPERR = -11,
  PCAP_READ_END = -10,
  PCAP_READING_PACKETERR = -9,
  SET_OPTIONERR = -8,
  NON_BLOCKERR = -7,
  BINDERR = -6,
  SOCKETERR = -5,
  RECEIVEERR = -4,
  TIMEOUTERR = -3,
  POLLERR = -2,
  DEVICEERR = -1,
  NO_ERROR = 0
};

class InputBase {
 public:
  virtual ~InputBase() {}
  virtual int initParam() = 0;
  virtual int GetPacket(uint8_t *pkt) = 0;
  virtual void set_source_ip_str(const std::string& source_ip_str) = 0;
  virtual void set_multicast_ip_str(const std::string& multicast_ip_str) = 0;
  virtual void set_port(const int& port) = 0;
  virtual void set_receive_len(const int& receive_len) = 0;
  virtual void add_valid_packet_size(const int& packet_size) = 0;
 protected:
  // one second (in msec)
  static const int POLL_TIMEOUT = 1000;
};

class Input : public InputBase {
 public:
  Input(const std::string& source_ip,
        const std::string& multicast_ip,
        const int& port);
  ~Input();
  int initParam();
  int GetPacket(uint8_t *pkt);
  void set_source_ip_str(const std::string& source_ip_str);
  void set_multicast_ip_str(const std::string& multicast_ip_str);
  void set_port(const int& port);
  void set_receive_len(const int& receive_len);
  void add_valid_packet_size(const int& packet_size);
 private:
  std::string source_ip_str_;
  std::string multicast_ip_str_;
  int port_;
  int sockfd_;
  in_addr source_ip_;
  int receive_len_;
  std::vector<int> valid_packet_size_list_;

};

class InputPCAP : public InputBase {
 public:
  InputPCAP(const std::string& source_ip,
            const std::string& multicast_ip,
            const int& port,
            const std::string& filename = "",
            const bool& read_once = true,
            const bool& read_fast = false,
            const int& packet_rate = 0.0
            );
  ~InputPCAP();
  int initParam();
  int GetPacket(uint8_t *pkt);
  void set_source_ip_str(const std::string& source_ip_str);
  void set_multicast_ip_str(const std::string& multicast_ip_str);
  void set_port(const int& port);
  void set_receive_len(const int& receive_len);
  void add_valid_packet_size(const int& packet_size);
 private:
  std::string source_ip_str_;
  std::string multicast_ip_str_;
  int port_;
  in_addr source_ip_;
  int receive_len_;
  std::vector<int> valid_packet_size_list_;
  std::string filename_;
  pcap_t *pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  int packet_rate_;
};

}  // namespace lidar_driver
}  // namespace itd_lidar

#endif  // _INPUT_H_
