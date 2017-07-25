//==============================================================================
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

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#include <ublox_gps/gps.h>

namespace ublox_gps {

using namespace ublox_msgs;

boost::posix_time::time_duration Gps::default_timeout_(
    boost::posix_time::seconds(Gps::kDefaultAckTimeout));

Gps::Gps() : configured_(false) {}

Gps::~Gps() { close(); }

void Gps::setWorker(const boost::shared_ptr<Worker>& worker) {
  if (worker_) return;
  worker_ = worker;
  worker_->setCallback(boost::bind(&Gps::readCallback, this, _1, _2));
  configured_ = static_cast<bool>(worker);
}

void Gps::initializeSerial(unsigned int baudrate,
                           uint16_t uart_in,
                           uint16_t uart_out) {
  if (worker_) return;
  setWorker(boost::shared_ptr<Worker>(
      new AsyncWorker<boost::asio::serial_port>(*serial_handle_, io_service_)));

  configured_ = false;

  boost::asio::serial_port_base::baud_rate current_baudrate;
  serial_handle_->get_option(current_baudrate);
  // Incrementally increase the baudrate to the desired value
  for (int i = 0; i < sizeof(kBaudrates)/sizeof(kBaudrates[0]); i++) {
    if (current_baudrate.value() == baudrate)
      break;
    // Don't step down, unless the desired baudrate is lower
    if(current_baudrate.value() > kBaudrates[i] && baudrate > kBaudrates[i])
      continue;
    serial_handle_->set_option(
        boost::asio::serial_port_base::baud_rate(kBaudrates[i]));
    boost::this_thread::sleep(
        boost::posix_time::milliseconds(kSetBaudrateSleepMs));
    serial_handle_->get_option(current_baudrate);
    ROS_DEBUG("U-Blox: Set ASIO baudrate to %u", current_baudrate.value());
  }
  configured_ = configUart1(baudrate, uart_in, uart_out);
  if(!configured_ || current_baudrate.value() != baudrate) {
    throw std::runtime_error("Could not configure serial baudrate");
  }
}

void Gps::initializeTcp() {
  if (worker_) return;
  setWorker(boost::shared_ptr<Worker>(
      new AsyncWorker<boost::asio::ip::tcp::socket>(*tcp_handle_, 
                                                    io_service_)));
}

void Gps::initializeIo(std::string device,
                       unsigned int baudrate,
                       uint16_t uart_in,
                       uint16_t uart_out) {
  boost::smatch match;

  if (boost::regex_match(device, match,
                         boost::regex("(tcp|udp)://(.+):(\\d+)"))) {
    std::string proto(match[1]);
    std::string host(match[2]);
    std::string port(match[3]);
    ROS_INFO("Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(),
             port.c_str());

    if (proto == "tcp") {
      boost::asio::ip::tcp::resolver::iterator endpoint;

      try {
        boost::asio::ip::tcp::resolver resolver(io_service_);
        endpoint =
            resolver.resolve(boost::asio::ip::tcp::resolver::query(host, port));
      } catch (std::runtime_error& e) {
        throw std::runtime_error("U-Blox: Could not resolve" + host + " " +
                                 port + " " + e.what());
      }

      boost::asio::ip::tcp::socket* socket =
          new boost::asio::ip::tcp::socket(io_service_);
      tcp_handle_.reset(socket);

      try {
        socket->connect(*endpoint);
      } catch (std::runtime_error& e) {
        throw std::runtime_error("U-Blox: Could not connect to " + 
                                 endpoint->host_name() + ":" + 
                                 endpoint->service_name() + ": " + e.what());
      }

      ROS_INFO("U-Blox: Connected to %s:%s.", endpoint->host_name().c_str(),
               endpoint->service_name().c_str());
      initializeTcp();
    } else {
      throw std::runtime_error("Protocol '" + proto + "' is unsupported");
    }
  } else {
    boost::asio::serial_port* serial = new boost::asio::serial_port(io_service_);
    serial_handle_.reset(serial);

    // open serial port
    try {
      serial->open(device);
    } catch (std::runtime_error& e) {
      throw std::runtime_error("U-Blox: Could not open serial port :" 
                               + device + " " + e.what());
    }

    ROS_INFO("U-Blox: Opened serial port %s", device.c_str());
    initializeSerial(baudrate, uart_in, uart_out);
  }
}

void Gps::close() {
  worker_.reset();
  configured_ = false;
}

bool Gps::reset(uint16_t nav_bbr_mask, uint16_t reset_mode) {
  ROS_WARN("Resetting Device. Node must be relaunched. %s", 
           "Device address may change.");

  CfgRST rst;
  rst.navBbrMask = nav_bbr_mask;
  rst.resetMode = reset_mode;

  // Don't wait for ACK, return if it fails
  if (!configure(rst, false)) 
    return false;
  return true;
}

bool Gps::configUart1(unsigned int baudrate, uint16_t in_proto_mask, 
                      uint16_t out_proto_mask) {
  if (!worker_) return true;

  ROS_DEBUG("Configuring UART1 baud rate: %u, In/Out Protocol: %u / %u", 
            baudrate, in_proto_mask, out_proto_mask);

  CfgPRT port;
  port.portID = CfgPRT::PORT_ID_UART1;
  port.baudRate = baudrate;
  port.mode = CfgPRT::MODE_RESERVED1 | CfgPRT::MODE_CHAR_LEN_8BIT |
              CfgPRT::MODE_PARITY_NO | CfgPRT::MODE_STOP_BITS_1;
  port.inProtoMask = in_proto_mask;
  port.outProtoMask = out_proto_mask;
  return configure(port);
}

bool Gps::disableUart1(CfgPRT& prev_config) {
  ROS_DEBUG("Disabling UART1");

  // Poll UART PRT Config
  std::vector<uint8_t> payload;
  payload.push_back(CfgPRT::PORT_ID_UART1);
  if (!poll(CfgPRT::CLASS_ID, CfgPRT::MESSAGE_ID, payload)) {
    ROS_ERROR("disableUart: Could not poll UART1 CfgPRT");
    return false;
  }
  if(!read(prev_config, default_timeout_)) {
    ROS_ERROR("disableUart: Could not read polled UART1 CfgPRT message");
    return false;
  }
  // Keep original settings, but disable in/out
  CfgPRT port;
  port.portID = CfgPRT::PORT_ID_UART1;
  port.mode = prev_config.mode;
  port.baudRate = prev_config.baudRate;
  port.inProtoMask = 0;
  port.outProtoMask = 0;
  port.txReady = prev_config.txReady;
  port.flags = prev_config.flags;
  return configure(port);
}

bool Gps::configRate(uint16_t meas_rate, uint16_t nav_rate) {
  ROS_DEBUG("Configuring measurement rate to %u and nav rate to %u", meas_rate, 
           nav_rate);

  CfgRATE rate;
  rate.measRate = meas_rate;
  rate.navRate = nav_rate;  //  must be fixed at 1 for ublox 5 and 6
  rate.timeRef = CfgRATE::TIME_REF_GPS;
  return configure(rate);
}

bool Gps::configRtcm(std::vector<uint8_t> ids, std::vector<uint8_t> rates) {
  for(size_t i = 0; i < ids.size(); ++i) {
    ROS_DEBUG("Setting RTCM %d Rate %u", ids[i], rates[i]);
    if(!setRate(ublox_msgs::Class::RTCM, (uint8_t)ids[i], rates[i])) {
      ROS_ERROR("Could not set RTCM %d to rate %u", ids[i], rates[i]);
      return false;
    }
  }
  return true;
}

bool Gps::configSbas(bool enable, uint8_t usage, uint8_t max_sbas) {
  ROS_DEBUG("Configuring SBAS: usage %u, max_sbas %u", usage, max_sbas);

  ublox_msgs::CfgSBAS msg;
  msg.mode = (enable ? CfgSBAS::MODE_ENABLED : 0);
  msg.usage = usage;
  msg.maxSBAS = max_sbas;
  return configure(msg);
}

bool Gps::configTmode3Fixed(bool lla_flag,
                            std::vector<float> arp_position, 
                            std::vector<float> arp_position_hp,
                            float fixed_pos_acc) {
  if(arp_position.size() != 3 || arp_position_hp.size() != 3) {
    ROS_ERROR("Configuring TMODE3 to Fixed: size of position %s",
              "& arp_position_hp args must be 3");
    return false;
  }

  ROS_DEBUG("Configuring TMODE3 to Fixed");

  CfgTMODE3 tmode3;
  tmode3.flags = tmode3.FLAGS_MODE_FIXED & tmode3.FLAGS_MODE_MASK;
  tmode3.flags |= lla_flag ? tmode3.FLAGS_LLA : 0;

  // Set position
  if(lla_flag) {
    // Convert from deg to deg / 1e-7
    tmode3.ecefXOrLat = (int)round(arp_position[0] * 1e7);
    tmode3.ecefYOrLon = (int)round(arp_position[1] * 1e7);
    tmode3.ecefZOrAlt = (int)round(arp_position[2] * 1e7);
    tmode3.ecefXOrLatHP = (int)round(arp_position_hp[0] * 1e7);
    tmode3.ecefYOrLonHP = (int)round(arp_position_hp[1] * 1e7);
    tmode3.ecefZOrAltHP = (int)round(arp_position_hp[2] * 1e7);
  } else {
    // Convert from m to cm
    tmode3.ecefXOrLat = (int)round(arp_position[0] * 1e2);
    tmode3.ecefYOrLon = (int)round(arp_position[1] * 1e2);
    tmode3.ecefZOrAlt = (int)round(arp_position[2] * 1e2);
    tmode3.ecefXOrLatHP = (int)round(arp_position_hp[0] * 1e2);
    tmode3.ecefYOrLonHP = (int)round(arp_position_hp[1] * 1e2);
    tmode3.ecefZOrAltHP = (int)round(arp_position_hp[2] * 1e2);
  }
  // Convert from m to [0.1 mm]
  tmode3.fixedPosAcc = (int)round(fixed_pos_acc * 1e4);
  return configure(tmode3);
}

bool Gps::configTmode3SurveyIn(unsigned int svin_min_dur, 
                               float svin_acc_limit) {
  CfgTMODE3 tmode3;
  ROS_DEBUG("Setting TMODE3 to Survey In");
  tmode3.flags = tmode3.FLAGS_MODE_SURVEY_IN & tmode3.FLAGS_MODE_MASK;
  tmode3.svinMinDur = svin_min_dur;
  // Convert from m to [0.1 mm]
  tmode3.svinAccLimit = (int)round(svin_acc_limit * 1e4);
  return configure(tmode3);
}

bool Gps::disableTmode3() {
  ROS_DEBUG("Disabling TMODE3");

  CfgTMODE3 tmode3;
  tmode3.flags = tmode3.FLAGS_MODE_DISABLED & tmode3.FLAGS_MODE_MASK;
  return configure(tmode3);
}

bool Gps::setRate(uint8_t class_id, uint8_t message_id, uint8_t rate) {
  ROS_DEBUG_COND(debug >= 2, "Setting rate 0x%02x, 0x%02x, %u", class_id, 
                 message_id, rate);
  ublox_msgs::CfgMSG msg;
  msg.msgClass = class_id;
  msg.msgID = message_id;
  msg.rate = rate;
  return configure(msg);
}

bool Gps::setDynamicModel(uint8_t model) {
  ROS_DEBUG("Setting dynamic model to %u", model);

  ublox_msgs::CfgNAV5 msg;
  msg.dynModel = model;
  msg.mask = ublox_msgs::CfgNAV5::MASK_DYN;
  return configure(msg);
}

bool Gps::setFixMode(uint8_t mode) {
  ROS_DEBUG("Setting fix mode to %u", mode);

  ublox_msgs::CfgNAV5 msg;
  msg.fixMode = mode;
  msg.mask = ublox_msgs::CfgNAV5::MASK_FIX_MODE;
  return configure(msg);
}

bool Gps::setDeadReckonLimit(uint8_t limit) {
  ROS_DEBUG("Setting DR Limit to %u", limit);
  
  ublox_msgs::CfgNAV5 msg;
  msg.drLimit = limit;
  msg.mask = ublox_msgs::CfgNAV5::MASK_DR_LIM;
  return configure(msg);
}

bool Gps::setPpp(bool enable) {
  ROS_DEBUG("%s PPP", (enable ? "Enabling" : "Disabling"));

  ublox_msgs::CfgNAVX5 msg;
  msg.usePPP = enable;
  msg.mask1 = ublox_msgs::CfgNAVX5::MASK1_PPP;
  return configure(msg);
}

bool Gps::setDgnss(uint8_t mode) {
  CfgDGNSS cfg;
  ROS_DEBUG("Setting DGNSS mode to %u", mode);
  cfg.dgnssMode = mode;
  return configure(cfg);
}

bool Gps::setUseAdr(bool enable) {
  ROS_DEBUG("%s ADR/UDR", (enable ? "Enabling" : "Disabling"));
  
  ublox_msgs::CfgNAVX5 msg;
  msg.useAdr = enable;
  msg.mask2 = ublox_msgs::CfgNAVX5::MASK2_ADR;
  return configure(msg);
}

bool Gps::poll(uint8_t class_id, uint8_t message_id,
               const std::vector<uint8_t>& payload) {
  if (!worker_) return false;

  std::vector<unsigned char> out(kWriterSize);
  ublox::Writer writer(out.data(), out.size());
  if (!writer.write(payload.data(), payload.size(), class_id, message_id))
    return false;
  worker_->send(out.data(), writer.end() - out.data());

  return true;
}

bool Gps::waitForAcknowledge(const boost::posix_time::time_duration& timeout, 
                             uint8_t class_id, uint8_t msg_id) {
  ROS_DEBUG_COND(debug >= 2, "Waiting for ACK 0x%02x / 0x%02x", 
                 class_id, msg_id);
  boost::posix_time::ptime wait_until(
      boost::posix_time::second_clock::local_time() + timeout);

  Ack ack = ack_.load(boost::memory_order_seq_cst);
  while (boost::posix_time::second_clock::local_time() < wait_until 
         && (ack.class_id != class_id 
             || ack.msg_id != msg_id 
             || ack.type == WAIT)) {
    worker_->wait(timeout);
    ack = ack_.load(boost::memory_order_seq_cst);
  }
  bool result = ack.type == ACK 
                && ack.class_id == class_id 
                && ack.msg_id == msg_id;
  return result;
}

void Gps::readCallback(unsigned char* data, std::size_t& size) {
  ublox::Reader reader(data, size);
  // Read all U-Blox messages in buffer
  while (reader.search() != reader.end() && reader.found()) {
    if (debug >= 3) {
      // Print the received bytes
      std::ostringstream oss;
      for (ublox::Reader::iterator it = reader.pos();
           it != reader.pos() + reader.length() + 8; ++it)
        oss << boost::format("%02x") % static_cast<unsigned int>(*it) << " ";
      ROS_DEBUG("U-blox: reading %d bytes\n%s", reader.length() + 8, 
               oss.str().c_str());
    }

    // Find the callback handlers for the message & decode it
    callback_mutex_.lock();
    Callbacks::key_type key =
        std::make_pair(reader.classId(), reader.messageId());
    for (Callbacks::iterator callback = callbacks_.lower_bound(key);
         callback != callbacks_.upper_bound(key); ++callback)
      callback->second->handle(reader);
    callback_mutex_.unlock();

    if (reader.classId() == ublox_msgs::Class::ACK) {
      // Process ACK/NACK messages
      const uint8_t * data = reader.data();
      Ack ack;
      ack.type = (reader.messageId() == ublox_msgs::Message::ACK::ACK) 
                  ? ACK : NACK;
      ack.class_id = data[0];
      ack.msg_id = data[1];
      // store the ack atomically
      ack_.store(ack, boost::memory_order_seq_cst);
      ROS_DEBUG_COND(ack.type == ACK && debug >= 2, 
                     "U-blox: received ACK: 0x%02x / 0x%02x", data[0], data[1]);
      if(ack.type == NACK)
        ROS_ERROR("U-blox: received NACK: 0x%02x / 0x%02x", data[0], data[1]);
    }
  }

  // delete read bytes from ASIO input buffer
  std::copy(reader.pos(), reader.end(), data);
  size -= reader.pos() - data;
}

}  // namespace ublox_gps
