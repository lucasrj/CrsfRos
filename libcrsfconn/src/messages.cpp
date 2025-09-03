#include <algorithm>
#include <libcrsfconn/messages.hpp>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace crsf {

CRSFFrame::CRSFFrame(std::vector<uint8_t> data) {
  length = data[0];
  type = MSG_TYPES(data[1]);
  payload.insert(payload.begin(), data.begin() + 2, data.begin() + length);
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

MSG_Attitude::MSG_Attitude(const CRSFFrame& frame) {
  const uint8_t* data = frame.payload.data();

  pitch = ((int16_t)data[0] << 8) | ((int16_t)data[1]);
  roll = ((int16_t)data[2] << 8) | ((int16_t)data[3]);
  yaw = ((int16_t)data[4] << 8) | ((int16_t)data[5]);
}

CRSFFrame MSG_Attitude::toFrame() {
  CRSFFrame frame;
  frame.length = 8;
  frame.type = MSG_TYPES::ATTITUDE;
  frame.payload.resize(6);

  // pitch
  frame.payload[0] = (pitch >> 8) & 0xFF;
  frame.payload[1] = pitch & 0xFF;
  // roll
  frame.payload[2] = (roll >> 8) & 0xFF;
  frame.payload[3] = roll & 0xFF;
  // yaw
  frame.payload[4] = (yaw >> 8) & 0xFF;
  frame.payload[5] = yaw & 0xFF;
  frame.crc = CRSFFrame::crc_calc(frame);
  return frame;
}

MSG_GPS::MSG_GPS(const CRSFFrame& frame) {
  const uint8_t* data = frame.payload.data();

  longitude = ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) | ((int32_t)data[2] << 8) | (int32_t)data[3];

  latitude = ((int32_t)data[4] << 24) | ((int32_t)data[5] << 16) | ((int32_t)data[6] << 8) | (int32_t)data[7];

  groundspeed = ((uint16_t)data[8] << 8) | (uint16_t)data[9];
  heading = ((uint16_t)data[10] << 8) | (uint16_t)data[11];
  altitude = ((uint16_t)data[12] << 8) | (uint16_t)data[13];

  satellites = data[14];
}

CRSFFrame MSG_GPS::toFrame() {
  CRSFFrame frame;
  frame.length = 17;
  frame.type = MSG_TYPES::GPS;
  frame.payload.resize(15);

  frame.payload[0] = (longitude >> 24) & 0xFF;
  frame.payload[1] = (longitude >> 16) & 0xFF;
  frame.payload[2] = (longitude >> 8) & 0xFF;
  frame.payload[3] = (longitude) & 0xFF;

  frame.payload[4] = (latitude >> 24) & 0xFF;
  frame.payload[5] = (latitude >> 16) & 0xFF;
  frame.payload[6] = (latitude >> 8) & 0xFF;
  frame.payload[7] = (latitude) & 0xFF;

  frame.payload[8] = (groundspeed >> 8) & 0xFF;
  frame.payload[9] = (groundspeed) & 0xFF;

  frame.payload[10] = (heading >> 8) & 0xFF;
  frame.payload[11] = (heading) & 0xFF;

  frame.payload[12] = (altitude >> 8) & 0xFF;
  frame.payload[13] = (altitude) & 0xFF;

  frame.payload[14] = satellites;

  frame.crc = CRSFFrame::crc_calc(frame);
  return frame;
}

MSG_GPS_Time::MSG_GPS_Time(const CRSFFrame& frame) {
  const uint8_t* data = frame.payload.data();

  year = (int16_t)data[0] | ((int16_t)data[1] << 8);
  month = data[2];
  day = data[3];
  hour = data[4];
  minute = data[5];
  second = data[6];
  millisecond = (uint16_t)data[7] | ((uint16_t)data[8] << 8);
}

CRSFFrame MSG_GPS_Time::toFrame() {
  CRSFFrame frame;
  frame.length = 11;
  frame.type = MSG_TYPES::GPS_TIME;
  frame.payload.resize(9);

  frame.payload[0] = (year >> 8) & 0xFF;
  frame.payload[1] = (year) & 0xFF;

  frame.payload[2] = month;
  frame.payload[3] = day;
  frame.payload[4] = hour;
  frame.payload[5] = minute;
  frame.payload[6] = second;

  frame.payload[7] = (millisecond >> 8) & 0xFF;
  frame.payload[8] = (millisecond) & 0xFF;

  frame.crc = CRSFFrame::crc_calc(frame);
  return frame;
}

MSG_GPS_Extended::MSG_GPS_Extended(const CRSFFrame& frame) {
  const uint8_t* data = frame.payload.data();

  fix_type = data[0];
  n_speed = (int16_t)data[1] | ((int16_t)data[2] << 8);
  e_speed = (int16_t)data[3] | ((int16_t)data[4] << 8);
  v_speed = (int16_t)data[5] | ((int16_t)data[6] << 8);
  h_speed_acc = (int16_t)data[7] | ((int16_t)data[8] << 8);
  track_acc = (int16_t)data[9] | ((int16_t)data[10] << 8);
  alt_ellipsoid = (int16_t)data[11] | ((int16_t)data[12] << 8);
  h_acc = (int16_t)data[13] | ((int16_t)data[14] << 8);
  v_acc = (int16_t)data[15] | ((int16_t)data[16] << 8);
  reserved = data[17];
  hDOP = data[18];
  vDOP = data[19];
}

CRSFFrame MSG_GPS_Extended::toFrame() {
  CRSFFrame frame;
  frame.length = 22;
  frame.type = MSG_TYPES::GPS_TIME;
  frame.payload.resize(20);

  frame.payload[0] = fix_type;

  frame.payload[1] = (n_speed >> 8) & 0xFF;
  frame.payload[2] = (n_speed) & 0xFF;

  frame.payload[3] = (e_speed >> 8) & 0xFF;
  frame.payload[4] = (e_speed) & 0xFF;

  frame.payload[5] = (v_speed >> 8) & 0xFF;
  frame.payload[6] = (v_speed) & 0xFF;

  frame.payload[7] = (h_speed_acc >> 8) & 0xFF;
  frame.payload[8] = (h_speed_acc) & 0xFF;

  frame.payload[9] = (track_acc >> 8) & 0xFF;
  frame.payload[10] = (track_acc) & 0xFF;

  frame.payload[11] = (alt_ellipsoid >> 8) & 0xFF;
  frame.payload[12] = (alt_ellipsoid) & 0xFF;

  frame.payload[13] = (h_acc >> 8) & 0xFF;
  frame.payload[14] = (h_acc) & 0xFF;

  frame.payload[15] = (v_acc >> 8) & 0xFF;
  frame.payload[16] = (v_acc) & 0xFF;

  frame.payload[17] = reserved;

  frame.payload[18] = hDOP;
  frame.payload[19] = vDOP;

  frame.crc = CRSFFrame::crc_calc(frame);
  return frame;
}

}  // namespace crsf