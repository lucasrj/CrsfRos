#include <asio/serial_port_base.hpp>
#include <asio/serial_port.hpp>
#include <libcrsfconn/serial_transport.hpp>
#include <thread>
#include <iostream>

#include <libcrsfconn/crsf.hpp>
#include "libcrsfconn/messages.hpp"
#include <sys/ioctl.h>
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
#define BOTHER 0010000
#endif

#ifndef TCGETS2
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#endif
#ifndef TCSETS2
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif
// --------------------------------------------------------------------

static void set_custom_baudrate(asio::serial_port& port, int baudrate) {
  int fd = port.native_handle();

  struct termios2 tio;
  if (ioctl(fd, TCGETS2, &tio) < 0)
  {
    throw std::runtime_error("TCGETS2 failed");
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;

  if (ioctl(fd, TCSETS2, &tio) < 0)
  {
    throw std::runtime_error("TCSETS2 failed");
  }
}

CRSFSerial::CRSFSerial(std::string device, unsigned baudrate, bool hwflow, ADDRESS_TYPE address)
  : CRSFInterface(address), io_service_(), serial_dev_(io_service_) {
  using asio::serial_port_base;

  try
  {
    serial_dev_.open(device);

    serial_dev_.native_handle();

    // serial_dev_.set_option(serial_port_base::baud_rate(115200));
    set_custom_baudrate(serial_dev_, baudrate);
    serial_dev_.set_option(serial_port_base::character_size(8));
    serial_dev_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_dev_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_dev_.set_option(serial_port_base::flow_control((hwflow) ? serial_port_base::flow_control::hardware :
                                                                     serial_port_base::flow_control::none));
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
                                sthis->parse_buffer(sthis->rx_buf_.data(), sthis->rx_buf_.size(), byte_transferred);
                                sthis->read();
                              }

  );
}

void CRSFSerial::send_msg(const crsf::CRSFFrame* message [[maybe_unused]]) {
}
}  // namespace crsf
