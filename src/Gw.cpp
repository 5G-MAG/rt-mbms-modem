// OBECA - Open Broadcast Edge Cache Appliance
// Receive Process
//
// Copyright (C) 2021 Klaus Kühnhammer (Österreichische Rundfunksender GmbH & Co KG)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "Gw.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_tun.h>
#include <net/if.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <algorithm>
#include <cerrno>

#include "spdlog/spdlog.h"

void Gw::write_pdu_mch(uint32_t lcid, srslte::unique_byte_buffer_t pdu) {
  char* err_str = nullptr;
  if (pdu->N_bytes > 2) {
    spdlog::debug("RX MCH PDU ({} B). Stack latency: {} us", pdu->N_bytes, pdu->get_latency_us());

    if (_tun_fd < 0) {
      spdlog::warn("TUN/TAP not up - dropping gw RX message\n");
    } else {
      auto ip_hdr = reinterpret_cast<iphdr*>(pdu->msg);
      if (ip_hdr->protocol == 17 /*UDP*/) {
        auto udp_hdr = reinterpret_cast<udphdr*>(pdu->msg + 4U * ip_hdr->ihl);
        char dest[INET6_ADDRSTRLEN] = "";   // NOLINT
        inet_ntop(AF_INET, (const void*)&ip_hdr->daddr, dest, sizeof(dest));

        _phy.set_dest_for_lcid(lcid, std::string(dest) + ":" + std::to_string(ntohs(udp_hdr->dest)));

        auto ptr = reinterpret_cast<uint16_t*>(ip_hdr);
        int32_t sum = 0;
        sum += *ptr++;   // ihl / version / dscp
        sum += *ptr++;   // total len
        sum += *ptr++;   // id
        sum += *ptr++;   // frag offset
        sum += *ptr++;   // ttl + protocol
        ptr++;           // skip checksum
        sum += *ptr++;   // src
        sum += *ptr++;   // src
        sum += *ptr++;   // dst
        sum += *ptr;     // dst

        sum = (sum >> 16) + (sum & 0xffff);
        sum += (sum >> 16);

        uint16_t chk = ~sum;
        if (ip_hdr->check != chk) {
          spdlog::info("Wrong IP header checksum {}, should be {}. Correcting.", ip_hdr->check, chk);
          ip_hdr->check = chk;
        }
      }

      _wr_mutex.lock();
      int n = write(_tun_fd, pdu->msg, pdu->N_bytes);
      _wr_mutex.unlock();

      if (n > 0 && (pdu->N_bytes != static_cast<uint32_t>(n))) {
        spdlog::warn("DL TUN/TAP short write");
      }
      if (n == 0) {
        spdlog::warn("DL TUN/TAP 0 write");
      }
      if (n < 0) {
        err_str = strerror(errno);
        spdlog::warn("DL TUN/TAP write error  {}", err_str);
      }
    }
  }
}

Gw::~Gw() {
  if (_tun_fd != -1) {
    close(_tun_fd);
  }
}

void Gw::init() {
  char* err_str = nullptr;
  struct ifreq ifr = {};

  _tun_fd = open("/dev/net/tun", O_RDWR | O_CLOEXEC);
  spdlog::info("TUN file descriptor {}", _tun_fd);
  if (0 > _tun_fd) {
    err_str = strerror(errno);
    spdlog::error("Failed to open TUN device {}", err_str);
    _tun_fd = -1;
  }

  std::string dev_name = "rp_tun";
  if (nullptr != std::getenv("RP_TUN_INTERFACE")) {
    dev_name = std::getenv("RP_TUN_INTERFACE");
  }

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_UP | IFF_TUN | IFF_NO_PI;
  strncpy(ifr.ifr_ifrn.ifrn_name, dev_name.c_str(),
          std::min(dev_name.length(), static_cast<size_t>(IFNAMSIZ - 1)));
  ifr.ifr_ifrn.ifrn_name[IFNAMSIZ - 1] = 0;

  if (0 > ioctl(_tun_fd, TUNSETIFF, &ifr)) {
    err_str = strerror(errno);
    spdlog::error("Failed to set TUN device name {}", err_str);
    close(_tun_fd);
    _tun_fd = -1;
  }

  if (0 > ioctl(_tun_fd, TUNSETPERSIST, 1)) {
    err_str = strerror(errno);
    spdlog::warn("Failed to set TUNSETPERSIST\n");
  }
}
