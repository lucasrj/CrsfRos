#include <gtest/gtest.h>
#include <stdexcept>

#include "libcrsfconn/messages.hpp"

namespace crsf {

uint8_t good_data1[26] = { 0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0x2B, 0xC0, 0xC7, 0x0A, 0x56, 0xB0, 0x02,
                           0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0x8A };

int data1_ticks[16] = { 992, 992, 172, 992, 172, 172, 172, 992, 992, 992, 992, 992, 992, 992, 992, 992 };
int data1_us[16] = { 1500, 1500, 988, 1500, 988, 988, 988, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

uint8_t good_data2[26] = { 0xC8, 0x18, 0x16, 0xAC, 0x60, 0x05, 0x2B, 0xC0, 0xC7, 0x0A, 0x56, 0xB0, 0x02,
                           0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0x2A };

int data2_ticks[16] = { 172, 172, 172, 992, 172, 172, 172, 992, 992, 992, 992, 992, 992, 992, 992, 992 };
int data2_us[16] = { 988, 988, 988, 1500, 988, 988, 988, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

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
    for (int i = 0; i < 16; i++)
    {
      std::cout << i << std::endl;
      EXPECT_EQ(rc_channels.channels[i], data1_ticks[i]);
    }
  });

  EXPECT_NO_THROW({
    MSG_RC_Channels rc_channels(frame, MSG_RC_Channels::TYPE_US);
    for (int i = 0; i < 16; i++)
    {
      std::cout << i << std::endl;
      EXPECT_EQ(rc_channels.channels[i], data1_us[i]);
    }
  });

  frame = get_good_frame2();

  EXPECT_NO_THROW({
    MSG_RC_Channels rc_channels(frame);
    for (int i = 0; i < 16; i++)
    {
      std::cout << i << std::endl;
      EXPECT_EQ(rc_channels.channels[i], data2_ticks[i]);
    }
  });

  EXPECT_NO_THROW({
    MSG_RC_Channels rc_channels(frame, MSG_RC_Channels::TYPE_US);
    for (int i = 0; i < 16; i++)
    {
      std::cout << i << std::endl;
      EXPECT_EQ(rc_channels.channels[i], data2_us[i]);
    }
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