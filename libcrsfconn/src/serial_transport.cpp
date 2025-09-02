#include <asio/serial_port_base.hpp>
#include <asio/serial_port.hpp>
#include <libcrsfconn/serial_transport.hpp>
#include <thread>
#include <iostream>

#include <libcrsfconn/crsf.hpp>
#include "libcrsfconn/messages.hpp"
#include <sys/ioctl.h>
#include <linux/serial.h>

namespace crsf {

// --- Linux extensions we need (termios2, BOTHER, TCGETS2, TCSETS2) ---
struct termios2
{
  tcflag_t c_iflag; /* input mode flags */
  tcflag_t c_oflag; /* output mode flags */
  tcflag_t c_cflag; /* control mode flags */
  tcflag_t c_lflag; /* local mode flags */
  cc_t c_line;      /* line discipline */
  cc_t c_cc[19];    /* control characters */
  speed_t c_ispeed; /* input speed */
  speed_t c_ospeed; /* output speed */
};

#ifndef BOTHER
#define BOTHER 0x00001000
#endif

#ifndef TCGETS2
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#endif
#ifndef TCSETS2
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif
// --------------------------------------------------------------------

CRSFSerial::CRSFSerial(std::string device, unsigned baudrate, bool hwflow, ADDRESS_TYPE address)
  : CRSFInterface(address), io_service_(), serial_dev_(io_service_) {
  using asio::serial_port_base;

  try
  {
    serial_dev_.open(device);

    int fd = serial_dev_.native_handle();

    struct termios2 tio = { 0 };
    if (ioctl(fd, TCGETS2, &tio) < 0)
    {
      throw std::runtime_error("TCGETS2 failed");
    }

    // tio.c_cflag &= ~CBAUD;
    // tio.c_cflag |= BOTHER;
    // tio.c_ispeed = baudrate;
    // tio.c_ospeed = baudrate;

    tio.c_cflag &= ~CBAUD;    // Clear old baud rate
    tio.c_ispeed = baudrate;  // Input speed
    tio.c_ospeed = baudrate;  // Output speed

    // tio.c_cc[VMIN] = 0;   // Minimum chars to read
    // tio.c_cc[VTIME] = 1;  // Timeout in 0.1 sec (i.e., 1 second)

    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | INPCK);
    tio.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | PARENB | PARODD | CMSPAR);
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CSTOPB;

    tio.c_cflag &= ~CRTSCTS;                 // Disable hardware flow control
    tio.c_cflag |= BOTHER | CLOCAL | CREAD;  // CREAD enables receiver, CLOCAL ignores modem lines
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control

    if (ioctl(fd, TCSETS2, &tio) < 0)
    {
      throw std::runtime_error("TCSETS2 failed");
    }

#if defined(__linux__)
    {
      int fd = serial_dev_.native_handle();
      struct serial_struct ser_info;

      if (ioctl(fd, TIOCGSERIAL, &ser_info) == -1)
        perror("TIOCGSERIAL failed");

      ser_info.flags |= ASYNC_LOW_LATENCY;

      if (ioctl(fd, TIOCSSERIAL, &ser_info) == -1)
        perror("TIOCSSERIAL failed");
    }
#endif
  }
  catch (asio::system_error& err)
  { throw DeviceError("serial", err); }
}

CRSFSerial::~CRSFSerial() {
  close();
}

void CRSFSerial::connect(const ReceivedCb& message_cb) {
  message_cb_ = message_cb;

  io_service_.post(std::bind(&CRSFSerial::read, this));

  io_thread_ = std::thread([this]() { io_service_.run(); });
}

void CRSFSerial::close() {
  std::lock_guard lock(mutex_);
  if (!is_open())
    return;

  serial_dev_.cancel();
  serial_dev_.close();

  io_service_.stop();
  if (io_thread_.joinable())
    io_thread_.join();

  io_service_.reset();
}

void CRSFSerial::read(void) {
  Ptr sthis = shared_from_this();
  serial_dev_.async_read_some(asio::buffer(rx_buf_),
                              [sthis](asio::error_code error, size_t byte_transferred) {
                                if (error)
                                {
                                  sthis->close();
                                  return;
                                }
                                sthis->parse_buffer(sthis->rx_buf_.data(), byte_transferred);
                                sthis->read();
                              }

  );
}

void CRSFSerial::send_msg(const crsf::CRSFFrame* message [[maybe_unused]]) {
}
}  // namespace crsf
