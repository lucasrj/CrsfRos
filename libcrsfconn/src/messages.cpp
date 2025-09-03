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

}  // namespace crsf