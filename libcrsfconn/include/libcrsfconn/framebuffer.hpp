#pragma once
#include <sys/types.h>
#include <algorithm>
#include <cstdint>
#include <libcrsfconn/messages.hpp>
#include <vector>

namespace crsf {

struct FrameBuffer
{
  static constexpr ssize_t MAX_SIZE = CRSFFrame::MAX_SIZE + 16;
  uint8_t data[MAX_SIZE];
  ssize_t len;
  ssize_t curser_pos;

  FrameBuffer() : data{}, len(0), curser_pos(0){};
  FrameBuffer(std::vector<uint8_t> frame_data) : curser_pos(0) {
    std::copy(frame_data.begin(),frame_data.end(),data);
    len = frame_data.size();
  }

  FrameBuffer(const crsf::CRSFFrame* frame) : curser_pos(0) {
    std::vector<uint8_t> frame_data = frame->serialize();
    std::copy(frame_data.begin(),frame_data.end(),data);
    len = frame_data.size();
  }
  FrameBuffer(const uint8_t * bytes, ssize_t nbytes)
  :len(nbytes),
  curser_pos(0){
    std::copy(bytes,bytes+len,data);
  }

  uint8_t * at_curser()
  {
    return data+curser_pos;
  }

  ssize_t bytes_left(){
    return len-curser_pos;
  }
};

}  // namespace crsf
