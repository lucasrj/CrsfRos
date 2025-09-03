#include <algorithm>
#include <libcrsfconn/messages.hpp>
#include <cstdint>
#include <stdexcept>
namespace crsf {


CRSFFrame::CRSFFrame(std::vector<uint8_t> data){
    length = data[0];
    type = MSG_TYPES(data[1]);
    payload.insert(payload.begin(),data.begin()+2,data.begin()+length);
    crc = data[length];
  }

std::vector<uint8_t> CRSFFrame::serialize() const {
  std::vector<uint8_t> data;
  data.reserve(length + 2);
  data.push_back(SYNC_BYTE);
  data.push_back(length);
  data.push_back(uint8_t(type));
  data.insert(data.end(), payload.begin(), payload.end());
  data.push_back(crc);
  return data;
}

uint8_t CRSFFrame::crc_calc(const CRSFFrame& frame) {
  uint8_t crc = crc8tab[0 ^ uint8_t(frame.type)];
  for (int i = 0; i < frame.payload.size(); i++)
    crc = crc8tab[crc ^ frame.payload[i]];
  return crc;
}

MSG_RC_Channels::MSG_RC_Channels(const CRSFFrame& frame, uint8_t type) {
  if (frame.type != MSG_TYPES::RC_CHANNELS_PACKED_PAYLOAD)
    throw std::invalid_argument("not of type RC_CHANNELS_PACKED_PAYLOAD");

  if (frame.payload.size() != 22)
    throw std::invalid_argument("payload wrong size");

  if (type > 1)
    throw std::invalid_argument("invalid type");

  for (int i = 0; i < channels.size(); i++)
  {
    channels[i] = get_channel(frame.payload.data(), i);
  }

  if (type == MSG_RC_Channels::TYPE_US)
  {
    for (int i = 0; i < channels.size(); i++)
    {
      channels[i] = TICKS_TO_US(channels[i]);
    }
  }
}

CRSFFrame MSG_RC_Channels::toFrame() {
  CRSFFrame frame;
  frame.length = 24;
  frame.type = MSG_TYPES::RC_CHANNELS_PACKED_PAYLOAD;
  frame.payload.reserve(22);
  uint8_t channels_data[22];

  channels_data[0] = (channels[0] >> 0) & 0xFF;
  channels_data[1] = (channels[0] >> 8) | ((channels[1] & 0x1F) << 3);
  channels_data[2] = (channels[1] >> 5) | ((channels[2] & 0x03) << 6);
  channels_data[3] = (channels[2] >> 2) & 0xFF;
  channels_data[4] = (channels[2] >> 10) | ((channels[3] & 0x7F) << 1);
  channels_data[5] = (channels[3] >> 7) | ((channels[4] & 0x0F) << 4);
  channels_data[6] = (channels[4] >> 4) | ((channels[5] & 0x01) << 7);
  channels_data[7] = (channels[5] >> 1) & 0xFF;
  channels_data[8] = (channels[5] >> 9) | ((channels[6] & 0x3F) << 2);
  channels_data[9] = (channels[6] >> 6) | ((channels[7] & 0x07) << 5);
  channels_data[10] = (channels[7] >> 3) & 0xFF;
  channels_data[11] = (channels[8] >> 0) & 0xFF;
  channels_data[12] = (channels[8] >> 8) | ((channels[9] & 0x1F) << 3);
  channels_data[13] = (channels[9] >> 5) | ((channels[10] & 0x03) << 6);
  channels_data[14] = (channels[10] >> 2) & 0xFF;
  channels_data[15] = (channels[10] >> 10) | ((channels[11] & 0x7F) << 1);
  channels_data[16] = (channels[11] >> 7) | ((channels[12] & 0x0F) << 4);
  channels_data[17] = (channels[12] >> 4) | ((channels[13] & 0x01) << 7);
  channels_data[18] = (channels[13] >> 1) & 0xFF;
  channels_data[19] = (channels[13] >> 9) | ((channels[14] & 0x3F) << 2);
  channels_data[20] = (channels[14] >> 6) | ((channels[15] & 0x07) << 5);
  channels_data[21] = (channels[15] >> 3) & 0xFF;

  frame.crc = CRSFFrame::crc_calc(frame);
  return frame;
}

uint16_t MSG_RC_Channels::get_channel(const uint8_t* buf, int i) {
  int bitIndex = i * 11;
  int byteIndex = bitIndex / 8;
  int bitOffset = bitIndex % 8;

  // collect 3 bytes (24 bits), enough to cover the 11-bit field
  uint32_t word = (uint32_t)buf[byteIndex] | ((uint32_t)buf[byteIndex + 1] << 8) | ((uint32_t)buf[byteIndex + 2] << 16);

  return (word >> bitOffset) & 0x07FF;
}

MSG_Link_Statistics::MSG_Link_Statistics(const CRSFFrame& frame) {
  if (frame.type != MSG_TYPES::LINK_STATISTICS)
    throw std::invalid_argument("not of type LINK_STATISTICS");

  if (frame.payload.size() != 10)
    throw std::invalid_argument("payload wrong size");

  const uint8_t* p = frame.payload.data();

  up_rssi_ant1 = p[0];
  up_rssi_ant2 = p[1];
  up_link_quality = p[2];
  up_snr = static_cast<int8_t>(p[3]);
  active_antenna = p[4];
  rf_profile = p[5];
  up_rf_power = p[6];
  down_rssi = p[7];
  down_link_quality = p[8];
  down_snr = static_cast<int8_t>(p[9]);
}

CRSFFrame MSG_Link_Statistics::toFrame() {
  CRSFFrame frame;
  frame.length = 12;
  frame.type = MSG_TYPES::LINK_STATISTICS;
  frame.payload.reserve(10);
  frame.payload.push_back(up_rssi_ant1);
  frame.payload.push_back(up_rssi_ant2);
  frame.payload.push_back(up_link_quality);
  frame.payload.push_back(static_cast<int8_t>(up_snr));
  frame.payload.push_back(active_antenna);
  frame.payload.push_back(rf_profile);
  frame.payload.push_back(up_rf_power);
  frame.payload.push_back(down_rssi);
  frame.payload.push_back(down_link_quality);
  frame.payload.push_back(down_snr);
  frame.crc = CRSFFrame::crc_calc(frame);
  return frame;
}

}  // namespace crsf