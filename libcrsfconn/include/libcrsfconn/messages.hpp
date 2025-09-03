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
  static constexpr uint8_t SYNC_BYTE = 0xC8;

  uint8_t length = 0;
  MSG_TYPES type = MSG_TYPES(0);
  std::vector<uint8_t> payload;
  uint8_t crc = 0;

  CRSFFrame() = default;
  CRSFFrame(std::vector<uint8_t> data);

  std::vector<uint8_t> serialize() const;

  static uint8_t crc_calc(const CRSFFrame& frame);
  static constexpr const uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D,
    0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F,
    0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9, 0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF,
    0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E,
    0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67,
    0xB2, 0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE,
    0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45, 0x11,
    0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92, 0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D,
    0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B,
    0xB1, 0x64, 0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F, 0x20,
    0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C,
    0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB, 0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05,
    0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
  };
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

  CRSFFrame toFrame();

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

  CRSFFrame toFrame();
};

struct MSG_Attitude
{
  int16_t pitch;  // Pitch angle (LSB = 100 µrad)
  int16_t roll;   // Roll angle  (LSB = 100 µrad)
  int16_t yaw;    // Yaw angle   (LSB = 100 µrad)

  MSG_Attitude() = default;
  MSG_Attitude(const CRSFFrame& frame);
  MSG_Attitude(float roll, float pitch, float yaw) {
    this->pitch = static_cast<int16_t>(pitch * 10000.0f);
    this->roll = static_cast<int16_t>(roll * 10000.0f);
    this->yaw = static_cast<int16_t>(yaw * 10000.0f);
  }

  void setpitch(float pitch) {
    this->pitch = (int16_t)(pitch * 10000.0f);
  }
  void setroll(float roll) {
    this->roll = (int16_t)(roll * 10000.0f);
  }
  void setyaw(float yaw) {
    this->yaw = (int16_t)(yaw * 10000.0f);
  }

  CRSFFrame toFrame();
};

struct MSG_GPS
{
  int32_t latitude;      // degree / 10`000`000
  int32_t longitude;     // degree / 10`000`000
  uint16_t groundspeed;  // km/h / 100
  uint16_t heading;      // degree / 100
  uint16_t altitude;     // meter - 1000m offset
  uint8_t satellites;    // # of sats in view

  MSG_GPS() = default;
  MSG_GPS(const CRSFFrame& frame);
  CRSFFrame toFrame();
};

struct MSG_GPS_Time
{
  int16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;

  MSG_GPS_Time() = default;
  MSG_GPS_Time(const CRSFFrame& frame);
  CRSFFrame toFrame();
};

struct MSG_GPS_Extended
{
  uint8_t fix_type;       // Current GPS fix quality
  int16_t n_speed;        // Northward (north = positive) Speed [cm/sec]
  int16_t e_speed;        // Eastward (east = positive) Speed [cm/sec]
  int16_t v_speed;        // Vertical (up = positive) Speed [cm/sec]
  int16_t h_speed_acc;    // Horizontal Speed accuracy cm/sec
  int16_t track_acc;      // Heading accuracy in degrees scaled with 1e-1 degrees times 10)
  int16_t alt_ellipsoid;  // Meters Height above GPS Ellipsoid (not MSL)
  int16_t h_acc;          // horizontal accuracy in cm
  int16_t v_acc;          // vertical accuracy in cm
  uint8_t reserved;
  uint8_t hDOP;  // Horizontal dilution of precision,Dimensionless in nits of.1.
  uint8_t vDOP;  // vertical dilution of precision, Dimensionless in nits of .1.

  MSG_GPS_Extended() = default;
  MSG_GPS_Extended(const CRSFFrame& frame);
  CRSFFrame toFrame();
};

struct MSG_Variometer
{
  int16_t v_speed;  // Vertical speed cm/s

  MSG_Variometer() = default;
  MSG_Variometer(const CRSFFrame& frame);
  CRSFFrame toFrame();
};

}  // namespace crsf
