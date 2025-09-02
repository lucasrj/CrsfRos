#include <gtest/gtest.h>

#include <libcrsfconn/crsf.hpp>
#include <cstdint>
#include "libcrsfconn/messages.hpp"

using namespace crsf;

class CRSFMock : public CRSFInterface {
public:
  using Ptr = std::shared_ptr<CRSFMock>;

  CRSFMock(ADDRESS_TYPE addr = ADDRESS_TYPE::FLIGHT_CONTROLLER) : CRSFInterface(addr) {
  }

  virtual ~CRSFMock() {
    close();
  }

  void connect(const ReceivedCb& cb) override {
    message_cb_ = cb;
  }

  void close() override {
    // no-op for testing
  }

  void receive_data(uint8_t* ptr, uint8_t length) {
    parse_buffer(ptr, length);
  }

  void send_msg(const crsf::CRSFFrame* /*message*/) override {
    // no-op for testing
  }

  bool is_open() override {
    return true;  // pretend it's always open
  }
};

uint8_t good_data[26] = { 0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0x2B, 0xC0, 0xC7, 0x0A, 0x56, 0xB0, 0x02,
                          0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0x8A };

unsigned const char crc8tab[256] = {
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

uint8_t compute_crc(CRSFFrame& frame) {
  uint8_t crc = crc8tab[0 ^ uint8_t(frame.type)];
  for (uint8_t i : frame.payload)
    crc = crc8tab[crc ^ i];
  return crc;
}

CRSFFrame get_good_frame() {
  CRSFFrame frame;
  frame.length = good_data[1];
  frame.type = MSG_TYPES(good_data[2]);
  frame.payload.insert(frame.payload.begin(), good_data + 3, good_data + 25);
  frame.crc = good_data[25];

  return frame;
}

TEST(CRSFMockTest, crc_test) {
  CRSFMock mock;

  CRSFFrame frame = get_good_frame();

  EXPECT_EQ(compute_crc(frame), 0x8A);
}

TEST(CRSFMockTest, full_msgs) {
  CRSFMock mock;

  CRSFFrame frame = get_good_frame();

  bool called = false;
  mock.connect([&](const CRSFFrame* f, crsf::Frame_status status) {
    called = true;
    EXPECT_EQ(f->type, frame.type);
    EXPECT_EQ(f->payload[0], 0xE0);
    std::cout << int(status) << std::endl;
    EXPECT_EQ(status, crsf::Frame_status::ok);
  });

  mock.receive_data(good_data, 26);

  EXPECT_TRUE(called);
}

TEST(CRSFMockTest, part_msgs) {
  CRSFMock mock;

  CRSFFrame frame = get_good_frame();

  bool called = false;
  mock.connect([&](const CRSFFrame* f, crsf::Frame_status status) {
    called = true;
    EXPECT_EQ(f->type, frame.type);
    EXPECT_EQ(f->payload[0], 0xE0);
    EXPECT_EQ(status, crsf::Frame_status::ok);
  });

  uint8_t test_data1[15];
  std::copy(good_data, good_data + 15, test_data1);

  uint8_t test_data2[11];
  std::copy(good_data + 15, good_data + 26, test_data2);
  mock.receive_data(test_data1, 15);
  mock.receive_data(test_data2, 11);

  EXPECT_TRUE(called);
}

TEST(CRSFMockTest, msgs_random_prefix) {
  CRSFMock mock;

  CRSFFrame frame = get_good_frame();

  bool called = false;
  mock.connect([&](const CRSFFrame* f, crsf::Frame_status status) {
    called = true;
    EXPECT_EQ(f->type, frame.type);
    EXPECT_EQ(f->payload[0], 0xE0);
    EXPECT_EQ(status, crsf::Frame_status::ok);
  });

  uint8_t test_data1[31] = { 0 };
  std::copy(good_data, good_data + 26, test_data1 + 4);

  mock.receive_data(test_data1, 31);

  EXPECT_TRUE(called);
}

TEST(CRSFMockTest, part_msgs_random_prefix) {
  CRSFMock mock;

  CRSFFrame frame = get_good_frame();

  bool called = false;
  mock.connect([&](const CRSFFrame* f, crsf::Frame_status status) {
    called = true;
    EXPECT_EQ(f->type, frame.type);
    EXPECT_EQ(f->payload[0], 0xE0);
    EXPECT_EQ(status, crsf::Frame_status::ok);
  });

  uint8_t test_data1[20] = { 0 };
  std::copy(good_data, good_data + 16, test_data1 + 4);

  uint8_t test_data2[11];
  std::copy(good_data + 16, good_data + 26, test_data2);
  mock.receive_data(test_data1, 20);
  mock.receive_data(test_data2, 11);

  EXPECT_TRUE(called);
}

TEST(CRSFMockTest, two_part_msgs) {
  CRSFMock mock;

  CRSFFrame frame = get_good_frame();

  uint8_t called = 0;
  mock.connect([&](const CRSFFrame* f, crsf::Frame_status status) {
    called++;
    EXPECT_EQ(f->type, frame.type);
    EXPECT_EQ(f->payload[0], 0xE0);
    EXPECT_EQ(status, crsf::Frame_status::ok);
  });

  uint8_t test_data1[20];
  std::copy(good_data, good_data + 20, test_data1);

  uint8_t test_data2[20];
  std::copy(good_data + 20, good_data + 26, test_data2);
  std::copy(good_data, good_data + 14, test_data2 + 6);

  uint8_t test_data3[20] = { 0 };
  std::copy(good_data + 14, good_data + 26, test_data3);
  mock.receive_data(test_data1, 20);
  mock.receive_data(test_data2, 20);
  mock.receive_data(test_data3, 20);

  EXPECT_EQ(called, 2);
}

TEST(CRSFMockTest, two_part_msgs_space) {
  CRSFMock mock;

  CRSFFrame frame = get_good_frame();

  uint8_t called = 0;
  mock.connect([&](const CRSFFrame* f, crsf::Frame_status status) {
    called++;
    EXPECT_EQ(f->type, frame.type);
    EXPECT_EQ(f->payload[0], 0xE0);
    EXPECT_EQ(status, crsf::Frame_status::ok);
  });

  uint8_t test_data1[20];
  std::copy(good_data, good_data + 20, test_data1);

  uint8_t test_data2[20];
  std::copy(good_data + 20, good_data + 26, test_data2);
  std::copy(good_data, good_data + 10, test_data2 + 10);

  uint8_t test_data3[20] = { 0 };
  std::copy(good_data + 10, good_data + 26, test_data3);
  mock.receive_data(test_data1, 20);
  mock.receive_data(test_data2, 20);
  mock.receive_data(test_data3, 20);

  EXPECT_EQ(called, 2);
}
