#pragma once

#include <sys/types.h>

#include <libcrsfconn/messages.hpp>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <system_error>
#include <vector>

namespace crsf {

enum class Frame_status : int8_t {
  bad_crc = -1,
  incomplete = 0,
  ok = 1,
};

class CRSFInterface {
private:
  CRSFInterface(const CRSFInterface&) = delete;

public:
  using ReceivedCb = std::function<void(const crsf::CRSFFrame* message, const Frame_status status)>;
  using Ptr = std::shared_ptr<CRSFInterface>;
  using ConstPtr = std::shared_ptr<CRSFInterface const>;
  // using WeakPtr = std::WeakPtr<CRSFInterface>;

  explicit CRSFInterface(ADDRESS_TYPE address);

  virtual void connect(const ReceivedCb& message_cb) = 0;

  virtual void close() = 0;

  virtual void send_msg(const crsf::CRSFFrame* message) = 0;

  virtual bool is_open() = 0;

protected:
  ReceivedCb message_cb_;
  void parse_buffer(uint8_t* buf, size_t bytes_received);

private:
  bool crc_check(CRSFFrame& frame) {
    return frame.crc == CRSFFrame::crc_calc(frame);
  };

  std::vector<uint8_t> parser_buffer_;
  bool parser_frame_started = false;
  uint8_t parser_frame_size_ = 0;

  ADDRESS_TYPE address_ = ADDRESS_TYPE::FLIGHT_CONTROLLER;
};

class DeviceError : public std::runtime_error {
public:
  /**
   * @breif Construct error.
   */
  template <typename T>
  DeviceError(const char* module, T msg) : std::runtime_error(make_message(module, msg)) {
  }

  template <typename T>
  static std::string make_message(const char* module, T msg) {
    std::ostringstream ss;
    ss << "DeviceError:" << module << ":" << msg_to_string(msg);
    return ss.str();
  }

  static std::string msg_to_string(const char* description) {
    return description;
  }

  static std::string msg_to_string(int errnum) {
    return ::strerror(errnum);
  }

  static std::string msg_to_string(std::system_error& err) {
    return err.what();
  }
};

}  // namespace crsf