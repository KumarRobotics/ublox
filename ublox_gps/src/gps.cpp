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

uint8_t modelFromString(const std::string& model) {
  std::string lower = model;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if(lower == "portable") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_PORTABLE;
  } else if(lower == "stationary") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_STATIONARY;
  } else if(lower == "pedestrian") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_PEDESTRIAN;
  } else if(lower == "automotive") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AUTOMOTIVE;
  } else if(lower == "sea") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_SEA;
  } else if(lower == "airborne1") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_1G;
  } else if(lower == "airborne2") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_2G;
  } else if(lower == "airborne4") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_4G;
  } else if(lower == "wristwatch") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_WRIST_WATCH;
  }

  throw std::runtime_error(lower + " is not a valid dynamic model.");
}

uint8_t fixModeFromString(const std::string& mode) {
  std::string lower = mode;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "2d") {
    return ublox_msgs::CfgNAV5::FIX_MODE_2D_ONLY;
  } else if (lower == "3d") {
    return ublox_msgs::CfgNAV5::FIX_MODE_3D_ONLY;
  } else if (lower == "auto") {
    return ublox_msgs::CfgNAV5::FIX_MODE_AUTO;
  }

  throw std::runtime_error(mode + " is not a valid fix mode.");
}

boost::posix_time::time_duration Gps::default_timeout_(
    boost::posix_time::seconds(Gps::kDefaultAckTimeout));

Gps::Gps() : configured_(false) {}

Gps::~Gps() { close(); }

bool Gps::configUart1(unsigned int baudrate, int16_t inProtoMask, 
                      int16_t outProtoMask) {
  baudrate_ = baudrate;
  if (!worker_) return true;

  CfgPRT port;
  port.portID = CfgPRT::PORT_ID_UART1;
  port.baudRate = baudrate_;
  port.mode = CfgPRT::MODE_RESERVED1 | CfgPRT::MODE_CHAR_LEN_8BIT |
              CfgPRT::MODE_PARITY_NO | CfgPRT::MODE_STOP_BITS_1;
  port.inProtoMask = inProtoMask;
  port.outProtoMask = outProtoMask;

  if (debug) {
    ROS_INFO("Setting In/Out Protocol: %i / %i", inProtoMask, outProtoMask);
    ROS_INFO("Changing baudrate to %u", baudrate);
  }
  return configure(port);
}

void Gps::initialize(const boost::shared_ptr<Worker>& worker) {
  if (worker_) return;
  worker_ = worker;
  worker_->setCallback(boost::bind(&Gps::readCallback, this, _1, _2));
  configured_ = static_cast<bool>(worker);
}

template void Gps::initialize(boost::asio::ip::tcp::socket& stream,
                              boost::asio::io_service& io_service,
                              unsigned int baudrate,
                              uint16_t uart_in,
                              uint16_t uart_out);

template <>
void Gps::initialize(boost::asio::serial_port& serial_port,
                     boost::asio::io_service& io_service,
                     unsigned int baudrate,
                     uint16_t uart_in,
                     uint16_t uart_out) {
  if (worker_) return;
  initialize(boost::shared_ptr<Worker>(
      new AsyncWorker<boost::asio::serial_port>(serial_port, io_service)));

  configured_ = false;

  boost::asio::serial_port_base::baud_rate current_baudrate;
  // TODO
  serial_port.set_option(boost::asio::serial_port_base::baud_rate(4800));
  boost::this_thread::sleep(
      boost::posix_time::milliseconds(kSetBaudrateSleepMs));
  if (debug) {
    serial_port.get_option(current_baudrate);
    ROS_INFO("Set baudrate %u", current_baudrate.value());
  }
  configured_ = configUart1(baudrate, uart_in, uart_out);
  if (configured_) return;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(9600));
  boost::this_thread::sleep(
      boost::posix_time::milliseconds(kSetBaudrateSleepMs));
  if (debug) {
    serial_port.get_option(current_baudrate);
    ROS_INFO("Set baudrate %u", current_baudrate.value());
  }
  configured_ = configUart1(baudrate, uart_in, uart_out);
  if (configured_) return;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(19200));
  boost::this_thread::sleep(
      boost::posix_time::milliseconds(kSetBaudrateSleepMs));
  if (debug) {
    serial_port.get_option(current_baudrate);
    ROS_INFO("Set baudrate %u", current_baudrate.value());
  }
  configured_ = configUart1(baudrate, uart_in, uart_out);
  if (configured_) return;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(38400));
  boost::this_thread::sleep(
      boost::posix_time::milliseconds(kSetBaudrateSleepMs));
  if (debug) {
    serial_port.get_option(current_baudrate);
    ROS_INFO("Set baudrate %u", current_baudrate.value());
  }
  configured_ = configUart1(baudrate, uart_in, uart_out);
  if (configured_) return;

  serial_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
  boost::this_thread::sleep(
      boost::posix_time::milliseconds(kSetBaudrateSleepMs));
  if (debug) {
    serial_port.get_option(current_baudrate);
    ROS_INFO("Set baudrate %u", current_baudrate.value());
  }
  configured_ = configUart1(baudrate, uart_in, uart_out);
  if (configured_) return;
}

void Gps::close() {
  worker_.reset();
  configured_ = false;
}

bool Gps::setRate(uint8_t class_id, uint8_t message_id, uint8_t rate) {
  ublox_msgs::CfgMSG msg;
  msg.msgClass = class_id;
  msg.msgID = message_id;
  msg.rate = rate;
  return configure(msg);
}

bool Gps::setDynamicModel(uint8_t model) {
  ublox_msgs::CfgNAV5 msg;
  msg.dynModel = model;
  msg.mask = ublox_msgs::CfgNAV5::MASK_DYN;
  return configure(msg);
}

bool Gps::setFixMode(uint8_t mode) {
  ublox_msgs::CfgNAV5 msg;
  msg.fixMode = mode;
  msg.mask = ublox_msgs::CfgNAV5::MASK_FIX_MODE;
  return configure(msg);
}

bool Gps::setDeadReckonLimit(uint8_t limit) {
  ublox_msgs::CfgNAV5 msg;
  msg.drLimit = limit;
  msg.mask = ublox_msgs::CfgNAV5::MASK_DR_LIM;
  return configure(msg);
}

bool Gps::setPPPEnabled(bool enabled) {
  ublox_msgs::CfgNAVX5 msg;
  msg.usePPP = enabled;
  msg.mask1 = ublox_msgs::CfgNAVX5::MASK1_PPP;
  return configure(msg);
}

bool Gps::enableSBAS(bool enabled, uint8_t usage, uint8_t max_sbas) {
  ublox_msgs::CfgSBAS msg;
  msg.mode = (enabled ? CfgSBAS::MODE_ENABLED : 0);
  msg.usage = usage;
  msg.maxSBAS = max_sbas;
  return configure(msg);
}

bool Gps::configRate(uint16_t meas_rate, uint16_t nav_rate) {
  if(debug) {
    ROS_INFO("Configuring measurement rate to %u and nav rate to %u", meas_rate, 
             nav_rate);
  }
  CfgRATE rate;
  rate.measRate = meas_rate;
  rate.navRate = nav_rate;  //  must be fixed at 1 for ublox 5 and 6
  rate.timeRef = CfgRATE::TIME_REF_GPS;
  return configure(rate);
}

bool Gps::disableUart(CfgPRT initialCfg) {
  // Poll UART PRT Config
  std::vector<uint8_t> payload;
  payload.push_back(CfgPRT::PORT_ID_UART1);
  if (!poll(CfgPRT::CLASS_ID, CfgPRT::MESSAGE_ID, payload)) {
    ROS_ERROR("disableUart: Could not poll UART1 CfgPRT");
    return false;
  }
  if(!read(initialCfg, default_timeout_)) {
    ROS_ERROR("disableUart: Could not read polled UART1 CfgPRT message");
    return false;
  }
  // Keep original settings, but disable in/out
  CfgPRT port;
  port.portID = CfgPRT::PORT_ID_UART1;
  port.mode = initialCfg.mode;
  port.baudRate = initialCfg.baudRate;
  port.inProtoMask = 0;
  port.outProtoMask = 0;
  port.txReady = initialCfg.txReady;
  port.flags = initialCfg.flags;
  return configure(port);
}

bool Gps::configRtcm(std::vector<int> ids, uint8_t rate) {
  for(size_t i = 0; i < ids.size(); ++i) {
    if(debug) {
      ROS_INFO("Setting RTCM %d Rate %u", ids[i], rate);
    }
    if(!setRate(ublox_msgs::Class::RTCM, (uint8_t)ids[i], rate)) {
      ROS_ERROR("Could not set RTCM %d to rate %u", ids[i], rate);
      return false;
    }
  }
  return true;
}

bool Gps::disableTmode3() {
  if(debug) {
    ROS_INFO("Disabling TMODE3");
  }

  CfgTMODE3 tmode3;
  tmode3.flags = tmode3.FLAGS_MODE_DISABLED & tmode3.FLAGS_MODE_MASK;
  return configure(tmode3);
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

  if(debug) {
    ROS_INFO("Configuring TMODE3 to Fixed");
  }

  CfgTMODE3 tmode3;
  tmode3.flags = tmode3.FLAGS_MODE_FIXED & tmode3.FLAGS_MODE_MASK;
  tmode3.flags |= lla_flag & tmode3.FLAGS_LLA;
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

bool Gps::configTmode3SurveyIn(unsigned int svinMinDur, 
                               float svinAccLimit) {
  CfgTMODE3 tmode3;
  if(debug) {
    ROS_INFO("Setting TMODE3 to Survey In");
  }
  tmode3.flags = tmode3.FLAGS_MODE_SURVEY_IN & tmode3.FLAGS_MODE_MASK;
  tmode3.svinMinDur = svinMinDur;
  // Convert from m to [0.1 mm]
  tmode3.svinAccLimit = (int)round(svinAccLimit * 1e4);
  return configure(tmode3);
}

bool Gps::configDgnss(uint8_t mode) {
  CfgDGNSS cfg;
  if(debug) {
    ROS_INFO("Setting DGNSS mode to %u", mode);
  }
  cfg.dgnssMode = mode;
  return configure(cfg);
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
  boost::posix_time::ptime wait_until(
      boost::posix_time::second_clock::local_time() + timeout);

  while (acknowledge_ == WAIT || acknowledge_class_id_ != class_id 
         || acknowledge_msg_id_ != msg_id
         && boost::posix_time::second_clock::local_time() < wait_until ) {
    worker_->wait(timeout);
  }
  return acknowledge_ == ACK 
         && acknowledge_class_id_ == class_id 
         && acknowledge_msg_id_ == msg_id;
}

void Gps::readCallback(unsigned char* data, std::size_t& size) {
  ublox::Reader reader(data, size);
  while (reader.search() != reader.end() && reader.found()) {
    if (debug >= 3) {
      std::ostringstream oss;
      for (ublox::Reader::iterator it = reader.pos();
           it != reader.pos() + reader.length() + 8; ++it)
        oss << std::hex << static_cast<unsigned int>(*it) << " ";
      ROS_INFO("U-blox: received %d bytes\n%s", reader.length() + 8, 
               oss.str().c_str());
    }

    callback_mutex_.lock();
    Callbacks::key_type key =
        std::make_pair(reader.classId(), reader.messageId());
    for (Callbacks::iterator callback = callbacks_.lower_bound(key);
         callback != callbacks_.upper_bound(key); ++callback)
      callback->second->handle(reader);
    callback_mutex_.unlock();

    if (reader.classId() == ublox_msgs::Class::ACK) {
      const uint8_t * data = reader.data();
      acknowledge_ = (reader.messageId() == ublox_msgs::Message::ACK::NACK) 
                     ? NACK : ACK;
      acknowledge_class_id_ = data[0];
      acknowledge_msg_id_ = data[1];
      if (acknowledge_ == ACK && debug >= 2)
        ROS_INFO("U-blox: received ACK: %x / %x", data[0], data[1]);
      else if(acknowledge_ == NACK) {
        ROS_ERROR("U-blox: received NACK: %x / %x", data[0], data[1]);
      }
    }
  }

  // delete read bytes from input buffer
  std::copy(reader.pos(), reader.end(), data);
  size -= reader.pos() - data;
}

}  // namespace ublox_gps
