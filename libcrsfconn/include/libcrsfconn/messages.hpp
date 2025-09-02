#pragma once

#include <sys/types.h>

#include <array>
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
  static constexpr ssize_t MAX_SIZE = 64;
  uint8_t length = 0;
  MSG_TYPES type = MSG_TYPES(0);
  std::vector<uint8_t> payload;
  uint8_t crc = 0;
  CRSFFrame() = default;
  CRSFFrame(std::vector<uint8_t> data);
};

struct CRSFExtendedFrame
{
  static constexpr ssize_t MAX_SIZE = 64;
  uint8_t length;
  uint8_t type;
  uint8_t dist;
  uint8_t orign;
  std::vector<uint8_t> payload;
  uint8_t crc;
};

struct MSG_RC_Channels
{
  std::array<int, 16> channels = { 0 };

  static const uint8_t TYPE_TICKS = 0;
  static const uint8_t TYPE_US = 1;

  MSG_RC_Channels(const CRSFFrame& frame, uint8_t type = MSG_RC_Channels::TYPE_TICKS);

  static int TICKS_TO_US(int x) {
    return ((x - 992) * 5 / 8 + 1500);
  }
  static int US_TO_TICKS(int x) {
    return ((x - 1500) * 8 / 5 + 992);
  }

private:
  uint16_t get_channel(const uint8_t* buf, int i);
};

struct MSG_Link_Statistics
{
  uint8_t up_rssi_ant1;       // Uplink RSSI Antenna 1 (dBm * -1)
  uint8_t up_rssi_ant2;       // Uplink RSSI Antenna 2 (dBm * -1)
  uint8_t up_link_quality;    // Uplink Package success rate / Link quality (%)
  int8_t up_snr;              // Uplink SNR (dB)
  uint8_t active_antenna;     // number of currently best antenna
  uint8_t rf_profile;         // enum {4fps = 0 , 50fps, 150fps}
  uint8_t up_rf_power;        // enum {0mW = 0, 10mW, 25mW, 100mW,
                              // 500mW, 1000mW, 2000mW, 250mW, 50mW}
  uint8_t down_rssi;          // Downlink RSSI (dBm * -1)
  uint8_t down_link_quality;  // Downlink Package success rate / Link quality (%)
  int8_t down_snr;            // Downlink SNR (dB)

  MSG_Link_Statistics(const CRSFFrame& frame);
};

}  // namespace crsf
