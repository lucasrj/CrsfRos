#include <algorithm>
#include <libcrsfconn/crsf.hpp>
#include <cstdint>
#include <vector>
#include <iostream>

#include "libcrsfconn/messages.hpp"

namespace crsf {

CRSFInterface::CRSFInterface(ADDRESS_TYPE address) {
  address_ = address;
  parser_buffer_.reserve(CRSFFrame::MAX_SIZE);
}

void CRSFInterface::parse_frame_from_vector() {
  CRSFFrame frame;
  Frame_status frame_status = Frame_status::incomplete;
  if (parser_buffer_[0] > 0)
  {
    frame.length = parser_buffer_[0];
    frame.type = MSG_TYPES(parser_buffer_[1]);
    // std::cout << "frame size " << unsigned(frame.length) << std::endl;
    frame.payload.reserve(frame.length);
    frame.payload.insert(frame.payload.begin(), parser_buffer_.begin() + 2, parser_buffer_.begin() + frame.length);
    frame.crc = parser_buffer_[frame.length];
    // std::cout << "parse_frame_from_vector " << std::endl;
    // std::cout << "type " << std::hex << unsigned(frame.type) << std::dec << std::endl;
    // std::cout << "payload size " << frame.payload.size() << std::endl;
    // std::cout << "crc " << std::hex << unsigned(frame.crc) << std::dec << std::endl;
    // std::cout << "calc crc " << std::hex << unsigned(crc_calc(frame)) << std::dec << std::endl;
    if (crc_check(frame))
      frame_status = Frame_status::ok;
    else
      frame_status = Frame_status::bad_crc;

    if (message_cb_)
    {
      message_cb_(&frame, frame_status);
    }
  }
  parser_buffer_.erase(parser_buffer_.begin(), parser_buffer_.begin() + parser_frame_size_);
  parser_status_ = Parser_Status::WAIT_SYNC;
}

uint8_t CRSFInterface::crc_calc(CRSFFrame& frame) {
  uint8_t crc = crc8tab[0 ^ uint8_t(frame.type)];
  for (uint8_t i : frame.payload)
    crc = crc8tab[crc ^ i];
  return crc;
}

void CRSFInterface::parse_buffer(uint8_t* buf, const size_t bufsize, size_t bytes_received [[maybe_unused]]) {
  uint8_t* buf_end = buf + bufsize;
  parser_buffer_.insert(parser_buffer_.end(), buf, buf_end);
  switch (parser_status_)
  {
    case Parser_Status::WAIT_SYNC: {
      auto pos = std::find(parser_buffer_.begin(), parser_buffer_.end(), SYNC_BYTE);
      if (pos == parser_buffer_.end())
        break;

      parser_buffer_.erase(parser_buffer_.begin(), pos + 1);
      parser_status_ = Parser_Status::READ_LENGTH;

      if (parser_buffer_.size() == 0)
        break;

      [[fallthrough]];
    }
    case Parser_Status::READ_LENGTH:
      parser_frame_size_ = parser_buffer_[0];
      if (parser_frame_size_ > 64 || parser_frame_size_ <= 0)
      {
        parser_frame_size_ = 0;
        parser_status_ = Parser_Status::WAIT_SYNC;
        break;
      }
      parser_status_ = Parser_Status::READ_DATA;

      [[fallthrough]];

    case Parser_Status::READ_DATA:

      if (parser_buffer_.size() >= parser_frame_size_ + 1)
      {
        parse_frame_from_vector();
      }
      break;
  }
  if (parser_buffer_.size() > 400)
  {
    parser_buffer_.erase(parser_buffer_.begin(), parser_buffer_.end() - 255);
    parser_frame_size_ = 0;
    parser_status_ = Parser_Status::WAIT_SYNC;
  }
}

}  // namespace crsf