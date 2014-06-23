
//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ublox_gps/gps.h>

namespace ublox_gps {

using namespace ublox_msgs;

boost::posix_time::time_duration Gps::default_timeout_(boost::posix_time::seconds(1.0));
Gps::Gps()
  : configured_(false)
  , baudrate_(57600)
{
}

Gps::~Gps()
{
  close();
}

bool Gps::setBaudrate(unsigned int baudrate)
{
  baudrate_ = baudrate;
  if (!worker_) return true;

  CfgPRT port;
  port.baudRate = baudrate_;
  port.mode = CfgPRT::MODE_RESERVED1 | CfgPRT::MODE_CHAR_LEN_8BIT | CfgPRT::MODE_PARITY_NO | CfgPRT::MODE_STOP_BITS_1;
  port.inProtoMask = CfgPRT::PROTO_UBX | CfgPRT::PROTO_NMEA | CfgPRT::PROTO_RTCM;
  port.outProtoMask = CfgPRT::PROTO_UBX | CfgPRT::PROTO_NMEA;
  port.portID = CfgPRT::PORT_ID_UART1;

  return configure(port);
}

void Gps::initialize(const boost::shared_ptr<Worker> &worker)
{
  if (worker_) return;
  worker_ = worker;
  worker_->setCallback(boost::bind(&Gps::readCallback, this, _1, _2));
  configured_ = worker;
}

template void Gps::initialize(boost::asio::ip::tcp::socket& stream, boost::asio::io_service& io_service);
// template void Gps::initialize(boost::asio::ip::udp::socket& stream, boost::asio::io_service& io_service);

template <>
void Gps::initialize(boost::asio::serial_port& serial_port, boost::asio::io_service& io_service)
{
  if (worker_) return;
  initialize(boost::shared_ptr<Worker>(new AsyncWorker<boost::asio::serial_port>(serial_port, io_service)));

  configured_ = false;

  boost::asio::serial_port_base::baud_rate current_baudrate;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(4800));
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));
  if (debug) { serial_port.get_option(current_baudrate); std::cout << "Set baudrate " << current_baudrate.value() << std::endl; }
  configured_ = setBaudrate(baudrate_);
  if (configured_) return;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(9600));
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));
  if (debug) { serial_port.get_option(current_baudrate); std::cout << "Set baudrate " << current_baudrate.value() << std::endl; }
  configured_ = setBaudrate(baudrate_);
  if (configured_) return;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(19200));
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));
  if (debug) { serial_port.get_option(current_baudrate); std::cout << "Set baudrate " << current_baudrate.value() << std::endl; }
  configured_ = setBaudrate(baudrate_);
  if (configured_) return;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(38400));
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));
  if (debug) { serial_port.get_option(current_baudrate); std::cout << "Set baudrate " << current_baudrate.value() << std::endl; }
  configured_ = setBaudrate(baudrate_);
  if (configured_) return;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));
  if (debug) { serial_port.get_option(current_baudrate); std::cout << "Set baudrate " << current_baudrate.value() << std::endl; }
  configured_ = setBaudrate(baudrate_);
  if (configured_) return;
}

void Gps::close()
{
  worker_.reset();
  configured_ = false;
}

bool Gps::setRate(uint8_t class_id, uint8_t message_id, unsigned int rate)
{
  CfgMSG msg;
  msg.msgClass = class_id;
  msg.msgID = message_id;
  msg.rate = rate;
  return configure(msg);
}

bool Gps::enableSBAS(bool onoff) {
  CfgSBAS msg;
  msg.mode = (onoff ? CfgSBAS::MODE_ENABLED : 0);
  msg.usage = 255;
  msg.maxSBAS = 3;
  return configure(msg);
}

bool Gps::poll(uint8_t class_id, uint8_t message_id, const std::vector<uint8_t>& payload) {
  if (!worker_) return false;

  std::vector<unsigned char> out(1024);
  ublox::Writer writer(out.data(), out.size());
  if (!writer.write(payload.data(), payload.size(), class_id, message_id)) return false;
  worker_->send(out.data(), writer.end() - out.data());

  return true;
}

bool Gps::configure()
{
  configured_ = false;

  // unconfigure all messages
//  for(unsigned int id = 0; id < 256; ++id) {
//    CfgMSG msg;
//    msg.msgClass = Class::NAV;
//    msg.msgID = id;
//    msg.rate = 0;
//    configure(msg, false);

//    msg.msgClass = Class::RXM;
//    msg.msgID = id;
//    msg.rate = 0;
//    configure(msg, false);
//  }

  CfgRATE rate;
  rate.measRate = 250;
  rate.navRate = 1;
  rate.timeRef = CfgRATE::TIME_REF_GPS;
  if (!configure(rate)) return false;

  configured_ = true;
  return true;
}

void Gps::waitForAcknowledge(const boost::posix_time::time_duration& timeout) {
  boost::posix_time::ptime wait_until(boost::posix_time::second_clock::local_time() + timeout);

  while(acknowledge_ == WAIT && boost::posix_time::second_clock::local_time() < wait_until) {
    worker_->wait(timeout);
  }
}

void Gps::readCallback(unsigned char *data, std::size_t& size) {
  ublox::Reader reader(data, size);

  while(reader.search() != reader.end() && reader.found()) {
    if (debug >= 3) {
      std::cout << "received ublox " << reader.length() + 8 << " bytes" << std::endl;
      for(ublox::Reader::iterator it = reader.pos(); it != reader.pos() + reader.length() + 8; ++it) std::cout << std::hex << static_cast<unsigned int>(*it) << " ";
      std::cout << std::dec << std::endl;
    }

    callback_mutex_.lock();
    Callbacks::key_type key = std::make_pair(reader.classId(), reader.messageId());
    for(Callbacks::iterator callback = callbacks_.lower_bound(key); callback != callbacks_.upper_bound(key); ++callback) callback->second->handle(reader);
    callback_mutex_.unlock();

    if (reader.classId() == 0x05) {
      acknowledge_ = (reader.messageId() == 0x00) ? NACK : ACK;
      if (debug) std::cout << "received " << (acknowledge_ == ACK ? "ACK" : "NACK") << std::endl;
    }
  }

  // delete read bytes from input buffer
  std::copy(reader.pos(), reader.end(), data);
  size -= reader.pos() - data;
}

bool CallbackHandler::wait(const boost::posix_time::time_duration &timeout) {
  boost::mutex::scoped_lock lock(mutex_);
  return condition_.timed_wait(lock, timeout);
}

} // namespace ublox_gps
