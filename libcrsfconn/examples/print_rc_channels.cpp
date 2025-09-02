#include <cstdint>
#include <libcrsfconn/crsf.hpp>
#include "libcrsfconn/messages.hpp"
#include "libcrsfconn/serial_transport.hpp"
#include <iostream>
#include <chrono>
#include <ratio>
#include <thread>
#include <iomanip>

void callback(const crsf::CRSFFrame* frame, const crsf::Frame_status status) {
  if (status != crsf::Frame_status::ok)
    return;
  // std::cout << "recived frame of type " << std::hex << unsigned(frame->type) << std::dec << std::endl;
  if (frame->type == crsf::MSG_TYPES::RC_CHANNELS_PACKED_PAYLOAD && frame->payload.size() == 22)
  {
    crsf::MSG_RC_Channels channels(*frame);
    if (channels.channels[5] > 171)
    {
      std::cout << "channel 1 :" << std::setw(4) << channels.channels[0] << "  channel 2 :" << std::setw(4)
                << channels.channels[1] << "  channel 3 :" << std::setw(4) << channels.channels[2]
                << "  channel 4 :" << std::setw(4) << channels.channels[3] << "  channel 5 :" << std::setw(4)
                << channels.channels[4] << "  channel 6 :" << std::setw(4) << channels.channels[5]
                << " crc check :" << int(status) << std::endl;
    }
  }
  return;
  if (frame->type == crsf::MSG_TYPES::LINK_STATISTICS && frame->payload.size() == 10)
  {
    crsf::MSG_Link_Statistics link_stats(*frame);
    std::cout << "rssi :" << unsigned(link_stats.down_rssi) << "  antenna:" << unsigned(link_stats.active_antenna)
              << std::endl;
  }
}

int main(int argc [[maybe_unused]], char** argv [[maybe_unused]]) {
  crsf::CRSFSerial::Ptr crsf_serial = crsf::CRSFSerial::create_crsf_serial("/dev/ttyACM0");

  std::cout << "created" << std::endl;

  crsf_serial->connect(callback);

  std::cout << "connected" << std::endl;

  while (true)
  {
    std::cout << "sleeping" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}