#pragma once

#include <asio.hpp>
#include <libcrsfconn/crsf.hpp>
#include <libcrsfconn/messages.hpp>
#include <string>
#include <deque>
#include <libcrsfconn/framebuffer.hpp>

namespace crsf {

class CRSFSerial : public CRSFInterface, public std::enable_shared_from_this<CRSFSerial> {
public:
  using Ptr = std::shared_ptr<CRSFSerial>;
  virtual ~CRSFSerial();

  static Ptr create_crsf_serial(std::string device = "/dev/ttyACM0", unsigned baudrate = 416666, bool hwflow = false,
                                ADDRESS_TYPE address = ADDRESS_TYPE::FLIGHT_CONTROLLER) {
    return Ptr(new CRSFSerial(device, baudrate, hwflow, address));
  };

  void connect(const ReceivedCb& message_cb) override;

  void close() override;

  void send_msg(const crsf::CRSFFrame* message) override;

  inline bool is_open() override {
    return serial_dev_.is_open();
  }

private:
  CRSFSerial(std::string device, unsigned baudrate, bool hwflow = false,
             ADDRESS_TYPE address = ADDRESS_TYPE::FLIGHT_CONTROLLER);
  asio::io_service io_service_;
  std::thread io_thread_;
  asio::serial_port serial_dev_;

  std::recursive_mutex tx_mutex_;
  std::array<uint8_t, CRSFFrame::MAX_SIZE> rx_buffer_;
  std::deque<FrameBuffer> tx_queue_;
  bool tx_in_progress = false;

  void read();
  void write(bool check_tx_state);
};

}  // namespace crsf