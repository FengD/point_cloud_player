/**
  * Copyright (C) 2019 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING, Xu WANG
  * Description: This file used to define the udp input
  */
#include "input.h"
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <string>
#include <algorithm>
#include <sstream>

namespace itd_lidar {

namespace lidar_driver {
Input::Input(const std::string& source_ip,
             const std::string& multicast_ip,
             const int& port)
  : source_ip_str_(source_ip),
    multicast_ip_str_(multicast_ip),
    port_(port),
    sockfd_(-1) {
  if (source_ip_str_.empty()) {
    printf("empty source ip\n");
    return;
  }
  inet_aton(source_ip_str_.c_str(), &source_ip_);
}

Input::~Input() {
  close(sockfd_);
}

int Input::initParam() {
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1) {
    // Socket Error
    printf("socket\n");
    return SOCKETERR;
  }

  // my address information
  sockaddr_in my_addr;
  // initialize to zeros
  memset(&my_addr, 0, sizeof(my_addr));
  // host byte order
  my_addr.sin_family = AF_INET;
  // port in network byte order
  my_addr.sin_port = htons(port_);
  // automatically fill in my IP
  my_addr.sin_addr.s_addr = INADDR_ANY;
  // used for open multipule udp connnect
  int opt = 1;
  setsockopt( sockfd_, SOL_SOCKET, SO_REUSEADDR,
              (const void *)&opt, sizeof(opt) );

  if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
    // Bind Error
    printf("bind\n");
    return BINDERR;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
    printf("non-block\n");
    return NON_BLOCKERR;
  }

  if (!multicast_ip_str_.empty()) {
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip_str_.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if ( setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                    (char*) &mreq, sizeof(mreq)) < 0) {
      printf("setsockopt\n");
      return SET_OPTIONERR;
    }
  }
  return NO_ERROR;
}

int Input::GetPacket(uint8_t *pkt) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true) {
    do {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      // poll() error?
      if (retval < 0) {
        if (errno != EINTR)
          printf("poll() error\n");
        return POLLERR;
      }
      // poll() timeout?
      if (retval == 0) {
        printf("poll() timeout\n");
        return TIMEOUTERR;
      }
      // device error?
      if ((fds[0].revents & POLLERR)
          || (fds[0].revents & POLLHUP)
          || (fds[0].revents & POLLNVAL)) {
        printf("poll() reports error\n");
        return DEVICEERR;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = recvfrom(sockfd_, &pkt[0],
                              receive_len_,  0,
                              (sockaddr*) &sender_address,
                              &sender_address_len);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        printf("recvfail");
        return RECEIVEERR;
      }
    } else if ( find(valid_packet_size_list_.begin(),
                     valid_packet_size_list_.end(),
                     nbytes) != valid_packet_size_list_.end()) {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (source_ip_str_ != "" &&
          sender_address.sin_addr.s_addr != source_ip_.s_addr)
        continue;
      else
        break; // done
    }
  }
  return NO_ERROR;
}

void Input::set_source_ip_str(const std::string& source_ip_str) {
  source_ip_str_ = source_ip_str;
}

void Input::set_multicast_ip_str(const std::string& multicast_ip_str) {
  multicast_ip_str_ = multicast_ip_str;
}

void Input::set_port(const int& port) {
  port_ = port;
}

void Input::set_receive_len(const int& receive_len) {
  receive_len_ = receive_len;
}

void Input::add_valid_packet_size(const int& packet_size) {
  valid_packet_size_list_.push_back(packet_size);
}

InputPCAP::InputPCAP(const std::string& source_ip,
                 const std::string& multicast_ip,
                 const int& port,
                 const std::string& filename,
                 const bool& read_once,
                 const bool& read_fast,
                 const int& packet_rate)
  : source_ip_str_(source_ip),
    multicast_ip_str_(multicast_ip),
    port_(port),
    filename_(filename),
    read_once_(read_once),
    read_fast_(read_fast),
    packet_rate_(packet_rate) {
  pcap_ = NULL;
  empty_ = true;
  if (source_ip_str_.empty()) {
    printf("empty source ip\n");
    return;
  }
  inet_aton(source_ip_str_.c_str(), &source_ip_);
}

int InputPCAP::initParam() {
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL) {
    printf("Error opening Velodyne socket dump file.\n");
    return PCAP_DUMPERR;
  }
  std::stringstream filter;
  filter << "src host " << source_ip_str_ << " && ";
  filter << "udp dst port " << port_;
  pcap_compile(pcap_, &pcap_packet_filter_,
           filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  return NO_ERROR;
}

InputPCAP::~InputPCAP() {
  pcap_close(pcap_);
}

void InputPCAP::set_source_ip_str(const std::string& source_ip_str) {
  source_ip_str_ = source_ip_str;
}

void InputPCAP::set_multicast_ip_str(const std::string& multicast_ip_str) {
  multicast_ip_str_ = multicast_ip_str;
}

void InputPCAP::set_port(const int& port) {
  port_ = port;
}

void InputPCAP::set_receive_len(const int& receive_len) {
  receive_len_ = receive_len;
}

void InputPCAP::add_valid_packet_size(const int& packet_size) {
  valid_packet_size_list_.push_back(packet_size);
}

int InputPCAP::GetPacket(uint8_t *pkt) {
  struct pcap_pkthdr *header;
  const u_char *pkt_data;

  while (true) {
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
      if (0 == pcap_offline_filter(&pcap_packet_filter_,
                                   header, pkt_data))
        continue;

      if (read_fast_ == false)
        usleep(packet_rate_);

      memcpy(&pkt[0], pkt_data+42, receive_len_);
      empty_ = false;
      return NO_ERROR;
    }

   if (empty_) {
     printf("Error %d reading packet: %s\n",
              res, pcap_geterr(pcap_));
     return PCAP_READING_PACKETERR;
   }

   if (read_once_) {
     printf("end of file reached -- done reading.\n");
     return PCAP_READ_END;
   }


   printf("replaying Velodyne dump file\n");

   pcap_close(pcap_);
   pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
   empty_ = true;
  }
}


}  // namespace lidar_driver
}  // namespace itd_lidar
