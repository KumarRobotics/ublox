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

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include <asio/io_service.hpp>
#include <asio/serial_port.hpp>
#include <asio/serial_port_base.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/ip/udp.hpp>

#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/gps.hpp>
#include <ublox_msgs/ublox_msgs.hpp>

namespace ublox_gps {

const std::chrono::milliseconds Gps::default_timeout_ =
    std::chrono::milliseconds(
        static_cast<int>(Gps::kDefaultAckTimeout * 1000));

Gps::Gps(int debug, const rclcpp::Logger& logger) : configured_(false), save_on_shutdown_(false), config_on_startup_flag_(true), debug_(debug), callbacks_(debug), logger_(logger) {
  subscribeAcks();
}

Gps::~Gps() {
  close();
}

void Gps::setWorker(const std::shared_ptr<Worker>& worker) {
  if (worker_) {
    return;
  }
  worker_ = worker;
  worker_->setCallback(std::bind(&CallbackHandlers::readCallback,
                                 &callbacks_, std::placeholders::_1,
                                 std::placeholders::_2));
  configured_ = static_cast<bool>(worker);
}

void Gps::subscribeAcks() {
  // Set NACK handler
  subscribeId<ublox_msgs::msg::Ack>(std::bind(&Gps::processNack, this,
                                              std::placeholders::_1),
                                    ublox_msgs::Message::ACK::NACK);
  // Set ACK handler
  subscribeId<ublox_msgs::msg::Ack>(std::bind(&Gps::processAck, this,
                                              std::placeholders::_1),
                                    ublox_msgs::Message::ACK::ACK);
  // Set UPD-SOS-ACK handler
  subscribe<ublox_msgs::msg::UpdSOSAck>(std::bind(&Gps::processUpdSosAck, this,
                                                  std::placeholders::_1));
}

void Gps::processAck(const ublox_msgs::msg::Ack &m) {
  // Process ACK/NACK messages
  Ack ack{};
  ack.type = ACK;
  ack.class_id = m.cls_id;
  ack.msg_id = m.msg_id;
  // store the ack atomically
  ack_.store(ack, std::memory_order_seq_cst);
  RCLCPP_DEBUG_EXPRESSION(logger_, debug_ >= 2, "U-blox: received ACK: 0x%02x / 0x%02x",
                          m.cls_id, m.msg_id);
}

void Gps::processNack(const ublox_msgs::msg::Ack &m) {
  // Process ACK/NACK messages
  Ack ack{};
  ack.type = NACK;
  ack.class_id = m.cls_id;
  ack.msg_id = m.msg_id;
  // store the ack atomically
  ack_.store(ack, std::memory_order_seq_cst);
  RCLCPP_ERROR(logger_, "U-blox: received NACK: 0x%02x / 0x%02x", m.cls_id, m.msg_id);
}

void Gps::processUpdSosAck(const ublox_msgs::msg::UpdSOSAck &m) {
  if (m.cmd == ublox_msgs::msg::UpdSOSAck::CMD_BACKUP_CREATE_ACK) {
    Ack ack{};
    ack.type = (m.response == ublox_msgs::msg::UpdSOSAck::BACKUP_CREATE_ACK) ? ACK : NACK;
    ack.class_id = ublox_msgs::msg::UpdSOSAck::CLASS_ID;
    ack.msg_id = ublox_msgs::msg::UpdSOSAck::MESSAGE_ID;
    // store the ack atomically
    ack_.store(ack, std::memory_order_seq_cst);
    RCLCPP_DEBUG_EXPRESSION(logger_, ack.type == ACK && debug_ >= 2,
                            "U-blox: received UPD SOS Backup ACK");
    if (ack.type == NACK) {
      RCLCPP_ERROR(logger_, "U-blox: received UPD SOS Backup NACK");
    }
  }
}

void Gps::initializeSerial(const std::string & port, unsigned int baudrate,
                           uint16_t uart_in, uint16_t uart_out) {
  port_ = port;
  auto io_service = std::make_shared<asio::io_service>();
  auto serial = std::make_shared<asio::serial_port>(*io_service);

  // open serial port
  try {
    serial->open(port);
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not open serial port :"
                             + port + " " + e.what());
  }

  RCLCPP_INFO(logger_, "U-Blox: Opened serial port %s", port.c_str());

  int fd = serial->native_handle();
  termios tio{};
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  tcsetattr(fd, TCSANOW, &tio);

  // Set the I/O worker
  if (worker_) {
    return;
  }
  setWorker(std::make_shared<AsyncWorker<asio::serial_port>>(serial, io_service, 8192, debug_, logger_));

  configured_ = false;

  // Set the baudrate
  asio::serial_port_base::baud_rate current_baudrate;
  serial->get_option(current_baudrate);
  // Incrementally increase the baudrate to the desired value
  for (unsigned int fixed_baudrate : kBaudrates) {
    if (current_baudrate.value() == baudrate) {
      break;
    }
    // Don't step down, unless the desired baudrate is lower
    if (current_baudrate.value() > fixed_baudrate && baudrate > fixed_baudrate) {
      continue;
    }
    serial->set_option(asio::serial_port_base::baud_rate(fixed_baudrate));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kSetBaudrateSleepMs));
    serial->get_option(current_baudrate);
    RCLCPP_DEBUG(logger_, "U-Blox: Set ASIO baudrate to %u", current_baudrate.value());
  }
  if (config_on_startup_flag_) {
    configured_ = configUart1(baudrate, uart_in, uart_out);
    if (!configured_ || current_baudrate.value() != baudrate) {
      throw std::runtime_error("Could not configure serial baud rate");
    }
  } else {
    configured_ = true;
  }
}

void Gps::resetSerial(const std::string & port) {
  auto io_service = std::make_shared<asio::io_service>();
  auto serial = std::make_shared<asio::serial_port>(*io_service);

  // open serial port
  try {
    serial->open(port);
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not open serial port :"
                             + port + " " + e.what());
  }

  RCLCPP_INFO(logger_, "U-Blox: Reset serial port %s", port.c_str());

  // Set the I/O worker
  if (worker_) {
    return;
  }
  setWorker(std::make_shared<AsyncWorker<asio::serial_port>>(serial, io_service, 8192, debug_, logger_));
  configured_ = false;

  // Poll UART PRT Config
  std::vector<uint8_t> payload;
  payload.push_back(ublox_msgs::msg::CfgPRT::PORT_ID_UART1);
  if (!poll(ublox_msgs::msg::CfgPRT::CLASS_ID, ublox_msgs::msg::CfgPRT::MESSAGE_ID, payload)) {
    RCLCPP_ERROR(logger_, "Resetting Serial Port: Could not poll UART1 CfgPRT");
    return;
  }
  ublox_msgs::msg::CfgPRT prt;
  if (!read(prt, default_timeout_)) {
    RCLCPP_ERROR(logger_, "Resetting Serial Port: Could not read polled UART1 CfgPRT %s",
                 "message");
    return;
  }

  // Set the baudrate
  serial->set_option(asio::serial_port_base::baud_rate(prt.baud_rate));
  configured_ = true;
}

void Gps::initializeTcp(const std::string & host, const std::string & port) {
  host_ = host;
  port_ = port;
  auto io_service = std::make_shared<asio::io_service>();
  asio::ip::tcp::resolver::iterator endpoint;

  try {
    asio::ip::tcp::resolver resolver(*io_service);
    endpoint =
        resolver.resolve(asio::ip::tcp::resolver::query(host, port));
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not resolve" + host + " " +
                             port + " " + e.what());
  }

  auto socket = std::make_shared<asio::ip::tcp::socket>(*io_service);

  try {
    socket->connect(*endpoint);
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not connect to " +
                             endpoint->host_name() + ":" +
                             endpoint->service_name() + ": " + e.what());
  }

  RCLCPP_INFO(logger_, "U-Blox: Connected to %s:%s.", endpoint->host_name().c_str(),
              endpoint->service_name().c_str());

  if (worker_) {
    return;
  }
  setWorker(std::make_shared<AsyncWorker<asio::ip::tcp::socket>>(socket, io_service, 8192, debug_, logger_));
}

void Gps::initializeUdp(const std::string & host, const std::string & port) {
  host_ = host;
  port_ = port;
  auto io_service = std::make_shared<asio::io_service>();
  asio::ip::udp::resolver::iterator endpoint;

  try {
    asio::ip::udp::resolver resolver(*io_service);
    endpoint =
        resolver.resolve(asio::ip::udp::resolver::query(host, port));
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not resolve" + host + " " +
                             port + " " + e.what());
  }

  auto socket = std::make_shared<asio::ip::udp::socket>(*io_service);

  try {
    socket->connect(*endpoint);
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("U-Blox: Could not connect to " +
                             endpoint->host_name() + ":" +
                             endpoint->service_name() + ": " + e.what());
  }

  RCLCPP_INFO(logger_, "U-Blox: Connected to %s:%s.", endpoint->host_name().c_str(),
              endpoint->service_name().c_str());

  if (worker_) {
    return;
  }
  setWorker(std::make_shared<AsyncWorker<asio::ip::udp::socket>>(socket, io_service, 8192, debug_, logger_));
}

void Gps::close() {
  if (save_on_shutdown_) {
    if (saveOnShutdown()) {
      RCLCPP_INFO(logger_, "U-Blox Flash BBR saved");
    } else {
      RCLCPP_INFO(logger_, "U-Blox Flash BBR failed to save");
    }
  }
  worker_.reset();
  configured_ = false;
}

void Gps::reset(const std::chrono::milliseconds& wait) {
  worker_.reset();
  configured_ = false;
  // sleep because of undefined behavior after I/O reset
  std::this_thread::sleep_for(wait);
  if (host_ == "") {
    resetSerial(port_);
  } else {
    initializeTcp(host_, port_);
  }
}

bool Gps::configReset(uint16_t nav_bbr_mask, uint16_t reset_mode) {
  RCLCPP_WARN(logger_, "Resetting u-blox. If device address changes, %s",
              "node must be relaunched.");

  ublox_msgs::msg::CfgRST rst;
  rst.nav_bbr_mask = nav_bbr_mask;
  rst.reset_mode = reset_mode;

  // Don't wait for ACK, return if it fails
  return configure(rst, false);
}

bool Gps::configGnss(ublox_msgs::msg::CfgGNSS gnss,
                     const std::chrono::milliseconds& wait) {
  // Configure the GNSS settingshttps://mail.google.com/mail/u/0/#inbox
  RCLCPP_DEBUG(logger_, "Re-configuring GNSS.");
  if (!configure(gnss)) {
    return false;
  }
  // Cold reset the GNSS
  RCLCPP_WARN(logger_, "GNSS re-configured, cold resetting device.");
  if (!configReset(ublox_msgs::msg::CfgRST::NAV_BBR_COLD_START, ublox_msgs::msg::CfgRST::RESET_MODE_GNSS)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Reset the I/O
  reset(wait);
  return isConfigured();
}

bool Gps::saveOnShutdown() {
  // Command the receiver to stop
  ublox_msgs::msg::CfgRST rst;
  rst.nav_bbr_mask = ublox_msgs::msg::CfgRST::NAV_BBR_HOT_START;
  rst.reset_mode = ublox_msgs::msg::CfgRST::RESET_MODE_GNSS_STOP;
  if (!configure(rst)) {
    return false;
  }
  // Command saving the contents of BBR to flash memory
  // And wait for UBX-UPD-SOS-ACK
  ublox_msgs::msg::UpdSOS backup;
  return configure(backup);
}

bool Gps::clearBbr() {
  // Command saving the contents of BBR to flash memory
  // And wait for UBX-UPD-SOS-ACK
  ublox_msgs::msg::UpdSOS sos;
  sos.cmd = ublox_msgs::msg::UpdSOS::CMD_FLASH_BACKUP_CLEAR;
  return configure(sos);
}

bool Gps::configUart1(unsigned int baudrate, uint16_t in_proto_mask,
                      uint16_t out_proto_mask) {
  if (!worker_) {
    return true;
  }

  RCLCPP_DEBUG(logger_, "Configuring UART1 baud rate: %u, In/Out Protocol: %u / %u",
               baudrate, in_proto_mask, out_proto_mask);

  ublox_msgs::msg::CfgPRT port;
  port.port_id = ublox_msgs::msg::CfgPRT::PORT_ID_UART1;
  port.baud_rate = baudrate;
  port.mode = ublox_msgs::msg::CfgPRT::MODE_RESERVED1 | ublox_msgs::msg::CfgPRT::MODE_CHAR_LEN_8BIT |
              ublox_msgs::msg::CfgPRT::MODE_PARITY_NO | ublox_msgs::msg::CfgPRT::MODE_STOP_BITS_1;
  port.in_proto_mask = in_proto_mask;
  port.out_proto_mask = out_proto_mask;
  return configure(port);
}

bool Gps::disableUart1(ublox_msgs::msg::CfgPRT& prev_config) {
  RCLCPP_DEBUG(logger_, "Disabling UART1");

  // Poll UART PRT Config
  std::vector<uint8_t> payload;
  payload.push_back(ublox_msgs::msg::CfgPRT::PORT_ID_UART1);
  if (!poll(ublox_msgs::msg::CfgPRT::CLASS_ID, ublox_msgs::msg::CfgPRT::MESSAGE_ID, payload)) {
    RCLCPP_ERROR(logger_, "disableUart: Could not poll UART1 CfgPRT");
    return false;
  }
  if (!read(prev_config, default_timeout_)) {
    RCLCPP_ERROR(logger_, "disableUart: Could not read polled UART1 CfgPRT message");
    return false;
  }
  // Keep original settings, but disable in/out
  ublox_msgs::msg::CfgPRT port;
  port.port_id = ublox_msgs::msg::CfgPRT::PORT_ID_UART1;
  port.mode = prev_config.mode;
  port.baud_rate = prev_config.baud_rate;
  port.in_proto_mask = 0;
  port.out_proto_mask = 0;
  port.tx_ready = prev_config.tx_ready;
  port.flags = prev_config.flags;
  return configure(port);
}

bool Gps::configUsb(uint16_t tx_ready,
                    uint16_t in_proto_mask,
                    uint16_t out_proto_mask) {
  if (!worker_) {
    return true;
  }

  RCLCPP_DEBUG(logger_, "Configuring USB tx_ready: %u, In/Out Protocol: %u / %u",
               tx_ready, in_proto_mask, out_proto_mask);

  ublox_msgs::msg::CfgPRT port;
  port.port_id = ublox_msgs::msg::CfgPRT::PORT_ID_USB;
  port.tx_ready = tx_ready;
  port.in_proto_mask = in_proto_mask;
  port.out_proto_mask = out_proto_mask;
  return configure(port);
}

bool Gps::configRate(uint16_t meas_rate, uint16_t nav_rate) {
  RCLCPP_DEBUG(logger_, "Configuring measurement rate to %u ms and nav rate to %u cycles",
               meas_rate, nav_rate);

  ublox_msgs::msg::CfgRATE rate;
  rate.meas_rate = meas_rate;
  rate.nav_rate = nav_rate;  //  must be fixed at 1 for ublox 5 and 6
  rate.time_ref = ublox_msgs::msg::CfgRATE::TIME_REF_GPS;
  return configure(rate);
}

bool Gps::configRtcm(const std::vector<Rtcm> & rtcms) {
  for (const Rtcm & rtcm : rtcms) {
    RCLCPP_DEBUG(logger_, "Setting RTCM %d Rate %u", rtcm.id, rtcm.rate);
    if (!setRate(ublox_msgs::Class::RTCM, rtcm.id, rtcm.rate)) {
      RCLCPP_ERROR(logger_, "Could not set RTCM %d to rate %u", rtcm.id, rtcm.rate);
      return false;
    }
  }
  return true;
}

bool Gps::configSbas(bool enable, uint8_t usage, uint8_t max_sbas) {
  RCLCPP_DEBUG(logger_, "Configuring SBAS: usage %u, max_sbas %u", usage, max_sbas);

  ublox_msgs::msg::CfgSBAS msg;
  msg.mode = (enable ? ublox_msgs::msg::CfgSBAS::MODE_ENABLED : 0);
  msg.usage = usage;
  msg.max_sbas = max_sbas;
  return configure(msg);
}

bool Gps::configTmode3Fixed(bool lla_flag,
                            std::vector<double> arp_position,
                            std::vector<int8_t> arp_position_hp,
                            float fixed_pos_acc) {
  if (arp_position.size() != 3 || arp_position_hp.size() != 3) {
    RCLCPP_ERROR(logger_, "Configuring TMODE3 to Fixed: size of position %s",
                 "& arp_position_hp args must be 3");
    return false;
  }

  RCLCPP_DEBUG(logger_, "Configuring TMODE3 to Fixed");

  ublox_msgs::msg::CfgTMODE3 tmode3;
  tmode3.flags = ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_FIXED & ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_MASK;
  tmode3.flags |= lla_flag ? ublox_msgs::msg::CfgTMODE3::FLAGS_LLA : 0;

  // Set position
  if (lla_flag) {
    // Convert from [deg] to [deg * 1e-7]
    tmode3.ecef_x_or_lat = static_cast<int>(round(arp_position[0] * 1e7));
    tmode3.ecef_y_or_lon = static_cast<int>(round(arp_position[1] * 1e7));
    tmode3.ecef_z_or_alt = static_cast<int>(round(arp_position[2] * 1e7));
  } else {
    // Convert from m to cm
    tmode3.ecef_x_or_lat = static_cast<int>(round(arp_position[0] * 1e2));
    tmode3.ecef_y_or_lon = static_cast<int>(round(arp_position[1] * 1e2));
    tmode3.ecef_z_or_alt = static_cast<int>(round(arp_position[2] * 1e2));
  }
  tmode3.ecef_x_or_lat_hp = arp_position_hp[0];
  tmode3.ecef_y_or_lon_hp = arp_position_hp[1];
  tmode3.ecef_z_or_alt_hp = arp_position_hp[2];
  // Convert from m to [0.1 mm]
  tmode3.fixed_pos_acc = static_cast<uint32_t>(round(fixed_pos_acc * 1e4));
  return configure(tmode3);
}

bool Gps::configTmode3SurveyIn(unsigned int svin_min_dur,
                               float svin_acc_limit) {
  ublox_msgs::msg::CfgTMODE3 tmode3;
  RCLCPP_DEBUG(logger_, "Setting TMODE3 to Survey In");
  tmode3.flags = ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_SURVEY_IN & ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_MASK;
  tmode3.svin_min_dur = svin_min_dur;
  // Convert from m to [0.1 mm]
  tmode3.svin_acc_limit = static_cast<int>(round(svin_acc_limit * 1e4));
  return configure(tmode3);
}

bool Gps::disableTmode3() {
  RCLCPP_DEBUG(logger_, "Disabling TMODE3");

  ublox_msgs::msg::CfgTMODE3 tmode3;
  tmode3.flags = ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_DISABLED & ublox_msgs::msg::CfgTMODE3::FLAGS_MODE_MASK;
  return configure(tmode3);
}

bool Gps::setRate(uint8_t class_id, uint8_t message_id, uint8_t rate) {
  RCLCPP_DEBUG_EXPRESSION(logger_, debug_ >= 2, "Setting rate 0x%02x, 0x%02x, %u", class_id,
                          message_id, rate);
  ublox_msgs::msg::CfgMSG msg;
  msg.msg_class = class_id;
  msg.msg_id = message_id;
  msg.rate = rate;
  return configure(msg);
}

bool Gps::setDynamicModel(uint8_t model) {
  RCLCPP_DEBUG(logger_, "Setting dynamic model to %u", model);

  ublox_msgs::msg::CfgNAV5 msg;
  msg.dyn_model = model;
  msg.mask = ublox_msgs::msg::CfgNAV5::MASK_DYN;
  return configure(msg);
}

bool Gps::setFixMode(uint8_t mode) {
  RCLCPP_DEBUG(logger_, "Setting fix mode to %u", mode);

  ublox_msgs::msg::CfgNAV5 msg;
  msg.fix_mode = mode;
  msg.mask = ublox_msgs::msg::CfgNAV5::MASK_FIX_MODE;
  return configure(msg);
}

bool Gps::setDeadReckonLimit(uint8_t limit) {
  RCLCPP_DEBUG(logger_, "Setting DR Limit to %u", limit);

  ublox_msgs::msg::CfgNAV5 msg;
  msg.dr_limit = limit;
  msg.mask = ublox_msgs::msg::CfgNAV5::MASK_DR_LIM;
  return configure(msg);
}

bool Gps::setPpp(bool enable) {
  RCLCPP_DEBUG(logger_,"%s PPP", (enable ? "Enabling" : "Disabling"));

  ublox_msgs::msg::CfgNAVX5 msg;
  msg.use_ppp = enable;
  msg.mask1 = ublox_msgs::msg::CfgNAVX5::MASK1_PPP;
  return configure(msg);
}

bool Gps::setDgnss(uint8_t mode) {
  ublox_msgs::msg::CfgDGNSS cfg;
  RCLCPP_DEBUG(logger_, "Setting DGNSS mode to %u", mode);
  cfg.dgnss_mode = mode;
  return configure(cfg);
}

bool Gps::setUseAdr(bool enable) {
  RCLCPP_DEBUG(logger_, "%s ADR/UDR", (enable ? "Enabling" : "Disabling"));

  ublox_msgs::msg::CfgNAVX5 msg;
  msg.use_adr = enable;
  msg.mask2 = ublox_msgs::msg::CfgNAVX5::MASK2_ADR;
  return configure(msg);
}

bool Gps::poll(uint8_t class_id, uint8_t message_id,
               const std::vector<uint8_t>& payload) {
  if (!worker_) {
    return false;
  }

  std::vector<unsigned char> out(kWriterSize);
  ublox::Writer writer(out.data(), out.size());
  if (!writer.write(payload.data(), payload.size(), class_id, message_id)) {
    return false;
  }
  worker_->send(out.data(), writer.end() - out.data());

  return true;
}

bool Gps::waitForAcknowledge(const std::chrono::milliseconds& timeout,
                             uint8_t class_id, uint8_t msg_id) {
  RCLCPP_DEBUG_EXPRESSION(logger_, debug_ >= 2, "Waiting for ACK 0x%02x / 0x%02x",
                          class_id, msg_id);
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point wait_until = now + timeout;

  Ack ack = ack_.load(std::memory_order_seq_cst);
  while (std::chrono::system_clock::now() < wait_until
         && (ack.class_id != class_id
             || ack.msg_id != msg_id
             || ack.type == WAIT)) {
    worker_->wait(timeout);
    ack = ack_.load(std::memory_order_seq_cst);
  }
  bool result = ack.type == ACK
                && ack.class_id == class_id
                && ack.msg_id == msg_id;
  return result;
}

void Gps::setRawDataCallback(const Worker::WorkerRawCallback& callback) {
  if (!worker_) {
    return;
  }
  worker_->setRawDataCallback(callback);
}

bool Gps::setUTCtime() {
  RCLCPP_DEBUG(logger_, "Setting time to UTC time");

  ublox_msgs::msg::CfgNAV5 msg;
  msg.utc_standard = 3;
  return configure(msg);
}

bool Gps::setTimtm2(uint8_t rate) {
  RCLCPP_DEBUG(logger_, "TIM-TM2 send rate on current port set to %u", rate);
  ublox_msgs::msg::CfgMSG msg;
  msg.msg_class = ublox_msgs::msg::TimTM2::CLASS_ID;
  msg.msg_id = ublox_msgs::msg::TimTM2::MESSAGE_ID;
  msg.rate  = rate;
  return configure(msg);
}

}  // namespace ublox_gps
