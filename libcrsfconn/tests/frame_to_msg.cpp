#include <gtest/gtest.h>
#include <stdexcept>

#include "libcrsfconn/messages.hpp"

namespace crsf {

uint8_t good_data1[26] = { 0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0x2B, 0xC0, 0xC7, 0x0A, 0x56, 0xB0, 0x02,
                           0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0x8A };

uint8_t good_data2[26] = { 0xC8, 0x18, 0x16, 0xAC, 0x60, 0x05, 0x2B, 0xC0, 0xC7, 0x0A, 0x56, 0xB0, 0x02,
                           0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0x2A };

CRSFFrame get_good_frame1() {
  CRSFFrame frame;
  frame.length = good_data1[1];
  frame.type = MSG_TYPES(good_data1[2]);
  frame.payload.insert(frame.payload.begin(), good_data1 + 3, good_data1 + 25);
  frame.crc = good_data1[25];

  return frame;
}

CRSFFrame get_good_frame2() {
  CRSFFrame frame;
  frame.length = good_data2[1];
  frame.type = MSG_TYPES(good_data2[2]);
  frame.payload.insert(frame.payload.begin(), good_data2 + 3, good_data2 + 25);
  frame.crc = good_data2[25];

  return frame;
}

TEST(CRSFMockTest, rc_msg) {
  CRSFFrame frame = get_good_frame1();

  EXPECT_NO_THROW({
    MSG_RC_Channels rc_channels(frame);
    EXPECT_EQ(rc_channels.channel_01, 992);
    EXPECT_EQ(rc_channels.channel_02, 992);
    EXPECT_EQ(rc_channels.channel_03, 172);
    EXPECT_EQ(rc_channels.channel_04, 992);
    EXPECT_EQ(rc_channels.channel_05, 172);
    EXPECT_EQ(rc_channels.channel_06, 172);
    EXPECT_EQ(rc_channels.channel_07, 172);
    EXPECT_EQ(rc_channels.channel_08, 992);
    EXPECT_EQ(rc_channels.channel_09, 992);
    EXPECT_EQ(rc_channels.channel_10, 992);
    EXPECT_EQ(rc_channels.channel_11, 992);
    EXPECT_EQ(rc_channels.channel_12, 992);
    EXPECT_EQ(rc_channels.channel_13, 992);
    EXPECT_EQ(rc_channels.channel_14, 992);
    EXPECT_EQ(rc_channels.channel_15, 992);
    EXPECT_EQ(rc_channels.channel_16, 992);
  });

  EXPECT_NO_THROW({
    MSG_RC_Channels rc_channels(frame, MSG_RC_Channels::TYPE_US);
    EXPECT_EQ(rc_channels.channel_01, 1500);
    EXPECT_EQ(rc_channels.channel_02, 1500);
    EXPECT_EQ(rc_channels.channel_03, 988);
    EXPECT_EQ(rc_channels.channel_04, 1500);
    EXPECT_EQ(rc_channels.channel_05, 988);
    EXPECT_EQ(rc_channels.channel_06, 988);
    EXPECT_EQ(rc_channels.channel_07, 988);
    EXPECT_EQ(rc_channels.channel_08, 1500);
    EXPECT_EQ(rc_channels.channel_09, 1500);
    EXPECT_EQ(rc_channels.channel_10, 1500);
    EXPECT_EQ(rc_channels.channel_11, 1500);
    EXPECT_EQ(rc_channels.channel_12, 1500);
    EXPECT_EQ(rc_channels.channel_13, 1500);
    EXPECT_EQ(rc_channels.channel_14, 1500);
    EXPECT_EQ(rc_channels.channel_15, 1500);
    EXPECT_EQ(rc_channels.channel_16, 1500);
  });

  frame = get_good_frame2();

  EXPECT_NO_THROW({
    MSG_RC_Channels rc_channels(frame);
    EXPECT_EQ(rc_channels.channel_01, 172);
    EXPECT_EQ(rc_channels.channel_02, 172);
    EXPECT_EQ(rc_channels.channel_03, 172);
    EXPECT_EQ(rc_channels.channel_04, 992);
    EXPECT_EQ(rc_channels.channel_05, 172);
    EXPECT_EQ(rc_channels.channel_06, 172);
    EXPECT_EQ(rc_channels.channel_07, 172);
    EXPECT_EQ(rc_channels.channel_08, 992);
    EXPECT_EQ(rc_channels.channel_09, 992);
    EXPECT_EQ(rc_channels.channel_10, 992);
    EXPECT_EQ(rc_channels.channel_11, 992);
    EXPECT_EQ(rc_channels.channel_12, 992);
    EXPECT_EQ(rc_channels.channel_13, 992);
    EXPECT_EQ(rc_channels.channel_14, 992);
    EXPECT_EQ(rc_channels.channel_15, 992);
    EXPECT_EQ(rc_channels.channel_16, 992);
  });

  EXPECT_NO_THROW({
    MSG_RC_Channels rc_channels(frame, MSG_RC_Channels::TYPE_US);
    EXPECT_EQ(rc_channels.channel_01, 988);
    EXPECT_EQ(rc_channels.channel_02, 988);
    EXPECT_EQ(rc_channels.channel_03, 988);
    EXPECT_EQ(rc_channels.channel_04, 1500);
    EXPECT_EQ(rc_channels.channel_05, 988);
    EXPECT_EQ(rc_channels.channel_06, 988);
    EXPECT_EQ(rc_channels.channel_07, 988);
    EXPECT_EQ(rc_channels.channel_08, 1500);
    EXPECT_EQ(rc_channels.channel_09, 1500);
    EXPECT_EQ(rc_channels.channel_10, 1500);
    EXPECT_EQ(rc_channels.channel_11, 1500);
    EXPECT_EQ(rc_channels.channel_12, 1500);
    EXPECT_EQ(rc_channels.channel_13, 1500);
    EXPECT_EQ(rc_channels.channel_14, 1500);
    EXPECT_EQ(rc_channels.channel_15, 1500);
    EXPECT_EQ(rc_channels.channel_16, 1500);
  });
}

TEST(CRSFMockTest, error_testing) {
  CRSFFrame frame = get_good_frame1();

  frame.type = MSG_TYPES(0x00);

  EXPECT_THROW({ MSG_RC_Channels rc_channels(frame); }, std::invalid_argument);

  frame = get_good_frame1();
  frame.payload.push_back(0x12);

  EXPECT_THROW({ MSG_RC_Channels rc_channels(frame); }, std::invalid_argument);

  frame = get_good_frame1();

  EXPECT_THROW({ MSG_RC_Channels rc_channels(frame, 2); }, std::invalid_argument);
}

}  // namespace crsf