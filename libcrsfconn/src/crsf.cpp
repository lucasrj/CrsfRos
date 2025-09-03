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

void CRSFInterface::parse_buffer(uint8_t* buf, size_t bytes_received [[maybe_unused]]) {
  for (size_t i = 0; i < bytes_received; ++i)
  {
    if (!parser_frame_started && buf[i] == CRSFFrame::SYNC_BYTE)
    {
      parser_frame_started = true;
    }
    else if (parser_frame_started)
    {
      parser_buffer_.push_back(buf[i]);
      if (parser_buffer_[0]<3 || parser_buffer_[0]>63){
        parser_frame_started = false;
        parser_buffer_.erase(parser_buffer_.begin(), parser_buffer_.end());
      }
      if (parser_buffer_.size() > parser_buffer_[0])
      {
        CRSFFrame frame(parser_buffer_);
        Frame_status frame_status = Frame_status::incomplete;
        if (crc_check(frame))
          frame_status = Frame_status::ok;
        else
          frame_status = Frame_status::bad_crc;

        if (message_cb_)
        {
          message_cb_(&frame, frame_status);
        }

        parser_frame_started = false;
        parser_buffer_.erase(parser_buffer_.begin(), parser_buffer_.end());
      }
    }
  }
}

}  // namespace crsf