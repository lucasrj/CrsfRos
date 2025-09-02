#include <algorithm>
#include <libcrsfconn/messages.hpp>
#include <cstdint>
#include <stdexcept>
namespace crsf {

MSG_RC_Channels::MSG_RC_Channels(const CRSFFrame& frame, uint8_t type) {
  if (frame.type != MSG_TYPES::RC_CHANNELS_PACKED_PAYLOAD)
    throw std::invalid_argument("not of type RC_CHANNELS_PACKED_PAYLOAD");

  if (frame.payload.size() != 22)
    throw std::invalid_argument("payload wrong size");

  if (type > 1)
    throw std::invalid_argument("invalid type");
  const uint8_t* p = frame.payload.data();

  channel_01 = (p[0] | (p[1] << 8)) & 0x07FF;
  channel_02 = ((p[1] >> 3) | (p[2] << 5)) & 0x07FF;
  channel_03 = ((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x07FF;
  channel_04 = ((p[4] >> 1) | (p[5] << 7)) & 0x07FF;
  channel_05 = ((p[5] >> 4) | (p[6] << 4)) & 0x07FF;
  channel_06 = ((p[6] >> 7) | (p[7] << 1) | (p[8] << 9)) & 0x07FF;
  channel_07 = ((p[8] >> 2) | (p[9] << 6)) & 0x07FF;
  channel_08 = ((p[9] >> 5) | (p[10] << 3)) & 0x07FF;
  channel_09 = (p[11] | (p[12] << 8)) & 0x07FF;
  channel_10 = ((p[12] >> 3) | (p[13] << 5)) & 0x07FF;
  channel_11 = ((p[13] >> 6) | (p[14] << 2) | (p[15] << 10)) & 0x07FF;
  channel_12 = ((p[15] >> 1) | (p[16] << 7)) & 0x07FF;
  channel_13 = ((p[16] >> 4) | (p[17] << 4)) & 0x07FF;
  channel_14 = ((p[17] >> 7) | (p[18] << 1) | (p[19] << 9)) & 0x07FF;
  channel_15 = ((p[19] >> 2) | (p[20] << 6)) & 0x07FF;
  channel_16 = ((p[20] >> 5) | (p[21] << 3)) & 0x07FF;

  if (type == MSG_RC_Channels::TYPE_US)
  {
    channel_01 = MSG_RC_Channels::TICKS_TO_US(channel_01);
    channel_02 = MSG_RC_Channels::TICKS_TO_US(channel_02);
    channel_03 = MSG_RC_Channels::TICKS_TO_US(channel_03);
    channel_04 = MSG_RC_Channels::TICKS_TO_US(channel_04);
    channel_05 = MSG_RC_Channels::TICKS_TO_US(channel_05);
    channel_06 = MSG_RC_Channels::TICKS_TO_US(channel_06);
    channel_07 = MSG_RC_Channels::TICKS_TO_US(channel_07);
    channel_08 = MSG_RC_Channels::TICKS_TO_US(channel_08);
    channel_09 = MSG_RC_Channels::TICKS_TO_US(channel_09);
    channel_10 = MSG_RC_Channels::TICKS_TO_US(channel_10);
    channel_11 = MSG_RC_Channels::TICKS_TO_US(channel_11);
    channel_12 = MSG_RC_Channels::TICKS_TO_US(channel_12);
    channel_13 = MSG_RC_Channels::TICKS_TO_US(channel_13);
    channel_14 = MSG_RC_Channels::TICKS_TO_US(channel_14);
    channel_15 = MSG_RC_Channels::TICKS_TO_US(channel_15);
    channel_16 = MSG_RC_Channels::TICKS_TO_US(channel_16);
  }
}

}  // namespace crsf