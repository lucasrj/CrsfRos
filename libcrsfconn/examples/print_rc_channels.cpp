#include <cstdint>
#include <libcrsfconn/crsf.hpp>
#include "libcrsfconn/messages.hpp"
#include "libcrsfconn/serial_transport.hpp"
#include <iostream>
#include <chrono>
#include <ratio>
#include <thread>

void callback(const crsf::CRSFFrame* frame, const crsf::Frame_status status) {
  if (status != crsf::Frame_status::ok)
    return;
  std::cout << "recived frame of type " << std::hex << unsigned(frame->type) << std::dec << std::endl;
  if (frame->type == crsf::MSG_TYPES::RC_CHANNELS_PACKED_PAYLOAD)
  {
    crsf::MSG_RC_Channels channels(*frame);
    std::cout << "channel 1 :" << channels.channel_01 << "channel 2 :" << channels.channel_02 << std::endl;
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