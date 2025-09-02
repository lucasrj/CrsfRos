#pragma once

#include <sys/types.h>

#include <cstdint>
#include <limits>
#include <vector>

namespace crsf {

enum class MSG_TYPES : uint8_t {
  GPS = 0X02,
  GPS_TIME = 0X03,
  GPS_EXTENDED = 0X06,
  VARIOMETER_SENSOR = 0X07,
  BATTERY_SENSOR = 0X08,
  BAROMETRIC_ALTITUDE_VERTICAL_SPEED = 0X09,
  AIRSPEED = 0X0A,
  HEARTBEAT = 0X0B,
  RPM = 0X0C,
  TEMP = 0X0D,
  VOLTAGES = 0X0E,
  DISCONTINUED = 0X0F,
  VTX_TELEMETRY = 0X10,
  LINK_STATISTICS = 0X14,
  RC_CHANNELS_PACKED_PAYLOAD = 0X16,
  SUBSET_RC_CHANNELS_PACKED = 0X17,
  LINK_STATISTICS_RX = 0X1C,
  LINK_STATISTICS_TX = 0X1D,
  ATTITUDE = 0X1E,
  MAVLINK_FC = 0X1F,
  FLIGHT_MODE = 0X21,
  ESP_NOW_MESSAGES = 0X22
};

enum class ADDRESS_TYPE : uint8_t {
  BROADCAST = 0X00,
  CLOUD = 0X0E,
  USB_DEVICE = 0X10,
  BLUETOOTH_WIFI_MODULE = 0X12,
  WI_FI_RECEIVER = 0X13,
  VIDEO_RECEIVER = 0X14,
  OSD = 0X80,
  ESC_1 = 0X90,
  ESC_2 = 0X91,
  ESC_3 = 0X92,
  ESC_4 = 0X93,
  ESC_5 = 0X94,
  ESC_6 = 0X95,
  ESC_7 = 0X96,
  ESC_8 = 0X97,
  SENSOR = 0XC0,
  GPS = 0XC2,
  TBS_BLACKBOX = 0XC4,
  FLIGHT_CONTROLLER = 0XC8,
  RACE_TAG = 0XCC,
  VTX = 0XCE,
  REMOTE_CONTROL = 0XEA,
  RC_RECEIVER = 0XEC,
  RC_TRANSMITTER = 0XEE
};

struct CRSFFrame
{
  static constexpr ssize_t MAX_SIZE = std::numeric_limits<uint8_t>::max();
  uint8_t length;
  MSG_TYPES type;
  std::vector<uint8_t> payload;
  uint8_t crc;
};

struct CRSFExtendedFrame
{
  static constexpr ssize_t MAX_SIZE = std::numeric_limits<uint8_t>::max();
  uint8_t length;
  uint8_t type;
  uint8_t dist;
  uint8_t orign;
  std::vector<uint8_t> payload;
  uint8_t crc;
};

struct MSG_RC_Channels
{
  int channel_01;
  int channel_02;
  int channel_03;
  int channel_04;
  int channel_05;
  int channel_06;
  int channel_07;
  int channel_08;
  int channel_09;
  int channel_10;
  int channel_11;
  int channel_12;
  int channel_13;
  int channel_14;
  int channel_15;
  int channel_16;

  static const uint8_t TYPE_TICKS = 0;
  static const uint8_t TYPE_US = 1;

  MSG_RC_Channels(const CRSFFrame& frame, uint8_t type = MSG_RC_Channels::TYPE_TICKS);

  static int TICKS_TO_US(int x) {
    return ((x - 992) * 5 / 8 + 1500);
  }
  static int US_TO_TICKS(int x) {
    return ((x - 1500) * 8 / 5 + 992);
  }
};

}  // namespace crsf
