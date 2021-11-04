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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ublox_msgs/msg/aid_alm.hpp>
#include <ublox_msgs/msg/aid_eph.hpp>
#include <ublox_msgs/msg/aid_hui.hpp>
#include <ublox_msgs/msg/cfg_inf.hpp>
#include <ublox_msgs/msg/cfg_inf_block.hpp>
#include <ublox_msgs/msg/cfg_nav5.hpp>
#include <ublox_msgs/msg/cfg_prt.hpp>
#include <ublox_msgs/msg/inf.hpp>
#include <ublox_msgs/msg/mon_ver.hpp>
#include <ublox_msgs/msg/nav_clock.hpp>
#include <ublox_msgs/msg/nav_posecef.hpp>
#include <ublox_msgs/msg/nav_status.hpp>

#include <ublox_gps/adr_udr_product.hpp>
#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/fts_product.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/hp_pos_rec_product.hpp>
#include <ublox_gps/hpg_ref_product.hpp>
#include <ublox_gps/hpg_rov_product.hpp>
#include <ublox_gps/node.hpp>
#include <ublox_gps/raw_data_product.hpp>
#include <ublox_gps/tim_product.hpp>
#include <ublox_gps/ublox_firmware6.hpp>
#include <ublox_gps/ublox_firmware7.hpp>
#include <ublox_gps/ublox_firmware8.hpp>
#include <ublox_gps/ublox_firmware9.hpp>

namespace ublox_node {

/**
 * @brief Determine dynamic model from human-readable string.
 * @param model One of the following (case-insensitive):
 *  - portable
 *  - stationary
 *  - pedestrian
 *  - automotive
 *  - sea
 *  - airborne1
 *  - airborne2
 *  - airborne4
 *  - wristwatch
 * @return DynamicModel
 * @throws std::runtime_error on invalid argument.
 */
uint8_t modelFromString(const std::string& model) {
  std::string lower = model;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "portable") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_PORTABLE;
  }
  if (lower == "stationary") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_STATIONARY;
  }
  if (lower == "pedestrian") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_PEDESTRIAN;
  }
  if (lower == "automotive") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AUTOMOTIVE;
  }
  if (lower == "sea") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_SEA;
  }
  if (lower == "airborne1") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_1G;
  }
  if (lower == "airborne2") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_2G;
  }
  if (lower == "airborne4") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_AIRBORNE_4G;
  }
  if (lower == "wristwatch") {
    return ublox_msgs::msg::CfgNAV5::DYN_MODEL_WRIST_WATCH;
  }

  throw std::runtime_error("Invalid settings: " + lower +
                           " is not a valid dynamic model.");
}

/**
 * @brief Determine fix mode from human-readable string.
 * @param mode One of the following (case-insensitive):
 *  - 2d
 *  - 3d
 *  - auto
 * @return FixMode
 * @throws std::runtime_error on invalid argument.
 */
uint8_t fixModeFromString(const std::string& mode) {
  std::string lower = mode;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "2d") {
    return ublox_msgs::msg::CfgNAV5::FIX_MODE_2D_ONLY;
  }
  if (lower == "3d") {
    return ublox_msgs::msg::CfgNAV5::FIX_MODE_3D_ONLY;
  }
  if (lower == "auto") {
    return ublox_msgs::msg::CfgNAV5::FIX_MODE_AUTO;
  }

  throw std::runtime_error("Invalid settings: " + mode +
                           " is not a valid fix mode.");
}

std::vector<std::string> stringSplit(const std::string &str,
                                     const std::string &splitter) {
  std::vector<std::string> ret;
  size_t next = 0;
  size_t current = next;

  if (splitter.empty()) {
    // If the splitter is blank, just return the original
    ret.push_back(str);
    return ret;
  }

  while (next != std::string::npos) {
    next = str.find(splitter, current);
    ret.push_back(str.substr(current, next - current));
    current = next + splitter.length();
  }

  return ret;
}

//
// u-blox ROS Node
//
UbloxNode::UbloxNode(const rclcpp::NodeOptions & options) : rclcpp::Node("ublox_gps_node", options) {
  int debug = this->declare_parameter("debug", 1);
  if (debug) {
    if (rcutils_logging_set_logger_level("ublox_gps_node", RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set the debugging level");
    }
  }

  gps_ = std::make_shared<ublox_gps::Gps>(debug, this->get_logger());

  gnss_ = std::make_shared<Gnss>();

  updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  updater_->setHardwareID("ublox");

  initialize();
}

void UbloxNode::addFirmwareInterface() {
  int ublox_version;
  if (protocol_version_ < 14.0) {
    components_.push_back(std::make_shared<UbloxFirmware6>(frame_id_, updater_, freq_diag_, gnss_, this));
    ublox_version = 6;
  } else if (protocol_version_ >= 14.0 && protocol_version_ <= 15.0) {
    components_.push_back(std::make_shared<UbloxFirmware7>(frame_id_, updater_, freq_diag_, gnss_, this));
    ublox_version = 7;
  } else if (protocol_version_ > 15.0 && protocol_version_ <= 23.0) {
    components_.push_back(std::make_shared<UbloxFirmware8>(frame_id_, updater_, freq_diag_, gnss_, this));
    ublox_version = 8;
  } else {
    components_.push_back(std::make_shared<UbloxFirmware9>(frame_id_, updater_, freq_diag_, gnss_, this));
    ublox_version = 9;
  }

  RCLCPP_INFO(this->get_logger(), "U-Blox Firmware Version: %d", ublox_version);
}


void UbloxNode::addProductInterface(const std::string & product_category,
                                    const std::string & ref_rov) {
  if (product_category == "HPG" && ref_rov == "REF") {
    components_.push_back(std::make_shared<HpgRefProduct>(nav_rate_, meas_rate_, updater_, rtcms_, this));
  } else if (product_category == "HPG" && ref_rov == "ROV") {
    components_.push_back(std::make_shared<HpgRovProduct>(nav_rate_, updater_, this));
  } else if (product_category == "HPG") {
    components_.push_back(std::make_shared<HpPosRecProduct>(nav_rate_, meas_rate_, frame_id_, updater_, rtcms_, this));
  } else if (product_category == "TIM") {
    components_.push_back(std::make_shared<TimProduct>(frame_id_, updater_, this));
  } else if (product_category == "ADR" ||
             product_category == "UDR") {
    components_.push_back(std::make_shared<AdrUdrProduct>(nav_rate_, meas_rate_, frame_id_, updater_, this));
  } else if (product_category == "FTS") {
    components_.push_back(std::make_shared<FtsProduct>());
  } else if (product_category == "SPG") {
    RCLCPP_WARN(this->get_logger(), "Product category %s %s from MonVER message not recognized %s",
                product_category.c_str(), ref_rov.c_str(),
                "options are HPG REF, HPG ROV, HPG #.#, TIM, ADR, UDR, FTS, SPG");
  }
}

void UbloxNode::getRosParams() {
  device_ = this->declare_parameter("device", std::string("/dev/ttyACM0"));
  frame_id_ = this->declare_parameter("frame_id", std::string("gps"));

  // Save configuration parameters
  load_.load_mask = declareRosIntParameter<uint32_t>(this, "load.mask", 0);
  load_.device_mask = declareRosIntParameter<uint8_t>(this, "load.device", 0);
  save_.save_mask = declareRosIntParameter<uint32_t>(this, "save.mask", 0);
  save_.device_mask = declareRosIntParameter<uint8_t>(this, "save.device", 0);

  // UART 1 params
  baudrate_ = declareRosIntParameter<uint32_t>(this, "uart1.baudrate", 9600);
  uart_in_ = declareRosIntParameter<uint16_t>(this, "uart1.in", ublox_msgs::msg::CfgPRT::PROTO_UBX
                                              | ublox_msgs::msg::CfgPRT::PROTO_NMEA
                                              | ublox_msgs::msg::CfgPRT::PROTO_RTCM);
  uart_out_ = declareRosIntParameter<uint16_t>(this, "uart1.out", ublox_msgs::msg::CfgPRT::PROTO_UBX);
  // USB params
  set_usb_ = false;
  this->declare_parameter("usb.in", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("usb.out", rclcpp::PARAMETER_INTEGER);
  usb_tx_ = declareRosIntParameter<uint16_t>(this, "usb.tx_ready", 0);
  if (isRosParameterSet(this, "usb.in") || isRosParameterSet(this, "usb.out")) {
    set_usb_ = true;
    if (!getRosUint(this, "usb.in", usb_in_)) {
      throw std::runtime_error(std::string("usb.out is set, therefore ") +
                               "usb.in must be set");
    }
    if (!getRosUint(this, "usb.out", usb_out_)) {
      throw std::runtime_error(std::string("usb.in is set, therefore ") +
                               "usb.out must be set");
    }
  }
  // Measurement rate params
  rate_ = this->declare_parameter("rate", 4.0);  // in Hz
  checkMin(rate_, 0.0, "rate");

  nav_rate_ = declareRosIntParameter<uint16_t>(this, "nav_rate", 1);  // # of measurement rate cycles

  // RTCM params
  this->declare_parameter("rtcm.ids", rclcpp::PARAMETER_INTEGER_ARRAY);
  this->declare_parameter("rtcm.rates", rclcpp::PARAMETER_INTEGER_ARRAY);
  std::vector<int64_t> rtcm_ids;
  std::vector<int64_t> rtcm_rates;
  this->get_parameter("rtcm.ids", rtcm_ids);
  this->get_parameter("rtcm.rates", rtcm_rates);

  if (rtcm_ids.size() != rtcm_rates.size()) {
    throw std::runtime_error(std::string("Invalid settings: size of rtcm_ids") +
                             " must match size of rtcm_rates");
  }

  rtcms_.resize(rtcm_ids.size());
  for (size_t i = 0; i < rtcm_ids.size(); ++i) {
    if (rtcm_ids[i] < 0 || rtcm_ids[i] > 255) {
      throw std::runtime_error("RTCM IDs must be between 0 and 255");
    }
    if (rtcm_rates[i] < 0 || rtcm_rates[i] > 255) {
      throw std::runtime_error("RTCM rates must be between 0 and 255");
    }
    rtcms_[i].id = rtcm_ids[i];
    rtcms_[i].rate = rtcm_rates[i];
  }

  // PPP: Advanced Setting
  this->declare_parameter("enable_ppp", false);
  if (getRosBoolean(this, "enable_ppp")) {
    RCLCPP_WARN(this->get_logger(), "Warning: PPP is enabled - this is an expert setting.");
  }

  // SBAS params, only for some devices
  this->declare_parameter("gnss.sbas", false);
  this->declare_parameter("gnss.gps", true);
  this->declare_parameter("gnss.glonass", false);
  this->declare_parameter("gnss.qzss", false);
  this->declare_parameter("gnss.galileo", false);
  this->declare_parameter("gnss.beidou", false);
  this->declare_parameter("gnss.imes", false);
  max_sbas_ = declareRosIntParameter<uint8_t>(this, "sbas.max", 0); // Maximum number of SBAS channels
  sbas_usage_ = declareRosIntParameter<uint8_t>(this, "sbas.usage", 0);
  dynamic_model_ = this->declare_parameter("dynamic_model", std::string("portable"));
  dmodel_ = modelFromString(dynamic_model_);
  fix_mode_ = this->declare_parameter("fix_mode", std::string("auto"));
  fmode_ = fixModeFromString(fix_mode_);
  dr_limit_ = declareRosIntParameter<uint8_t>(this, "dr_limit", 0); // Dead reckoning limit


  this->declare_parameter("dat.set", false);
  this->declare_parameter("dat.majA", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dat.flat", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dat.shift", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("dat.rot", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("dat.scale", rclcpp::PARAMETER_DOUBLE);
  if (getRosBoolean(this, "dat.set")) {
    std::vector<double> shift, rot;
    if (!this->get_parameter("dat.majA", cfg_dat_.maj_a)
        || !this->get_parameter("dat.flat", cfg_dat_.flat)
        || !this->get_parameter("dat.shift", shift)
        || !this->get_parameter("dat.rot", rot)
        || !this->get_parameter("dat.scale", cfg_dat_.scale)) {
      throw std::runtime_error(std::string("dat.set is true, therefore ") +
         "dat.majA, dat.flat, dat.shift, dat.rot, & dat.scale must be set");
    }
    if (shift.size() != 3 || rot.size() != 3) {
      throw std::runtime_error(std::string("size of dat.shift & dat.rot ") +
                               "must be 3");
    }
    checkRange(cfg_dat_.maj_a, 6300000.0, 6500000.0, "dat.majA");
    checkRange(cfg_dat_.flat, 0.0, 500.0, "dat.flat");

    checkRange(shift, 0.0, 500.0, "dat.shift");
    cfg_dat_.d_x = shift[0];
    cfg_dat_.d_y = shift[1];
    cfg_dat_.d_z = shift[2];

    checkRange(rot, -5000.0, 5000.0, "dat.rot");
    cfg_dat_.rot_x = rot[0];
    cfg_dat_.rot_y = rot[1];
    cfg_dat_.rot_z = rot[2];

    checkRange(cfg_dat_.scale, 0.0, 50.0, "scale");
  }

  // measurement period [ms]
  meas_rate_ = 1000 / rate_;

  // activate/deactivate any config
  this->declare_parameter("config_on_startup", true);
  this->declare_parameter("raw_data", false);
  this->declare_parameter("clear_bbr", false);
  this->declare_parameter("save_on_shutdown", false);
  this->declare_parameter("use_adr", true);

  this->declare_parameter("sv_in.reset", true);
  this->declare_parameter("sv_in.min_dur", 0);
  this->declare_parameter("sv_in.acc_lim", 0.0);

  this->declare_parameter("dgnss_mode", rclcpp::PARAMETER_INTEGER);

  // raw data stream logging
  this->declare_parameter("raw_data_stream.enable", false);
  if (getRosBoolean(this, "raw_data_stream.enable")) {
    raw_data_stream_pa_ = std::make_shared<ublox_node::RawDataStreamPa>(
      getRosBoolean(this, "raw_data_stream.enable"));
    raw_data_stream_pa_->getRosParams();
  }

  // NMEA parameters
  this->declare_parameter("nmea.set", false);
  this->declare_parameter("nmea.compat", false);
  this->declare_parameter("nmea.consider", false);
  this->declare_parameter("nmea.limit82", false);
  this->declare_parameter("nmea.high_prec", false);
  this->declare_parameter("nmea.filter.pos", false);
  this->declare_parameter("nmea.filter.msk_pos", false);
  this->declare_parameter("nmea.filter.time", false);
  this->declare_parameter("nmea.filter.date", false);
  this->declare_parameter("nmea.filter.sbas", false);
  this->declare_parameter("nmea.filter.track", false);
  this->declare_parameter("nmea.filter.gps_only", false);
  this->declare_parameter("nmea.gnssToFilter.gps", false);
  this->declare_parameter("nmea.gnssToFilter.sbas", false);
  this->declare_parameter("nmea.gnssToFilter.qzss", false);
  this->declare_parameter("nmea.gnssToFilter.glonass", false);
  this->declare_parameter("nmea.gnssToFilter.beidou", false);

  // Publish parameters
  this->declare_parameter("publish.all", false);

  this->declare_parameter("publish.nav.all", getRosBoolean(this, "publish.all"));
  this->declare_parameter("publish.nav.att", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.clock", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.heading", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.posecef", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.posllh", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.pvt", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.relposned", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.sat", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.sol", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.svin", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.svinfo", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.status", getRosBoolean(this, "publish.nav.all"));
  this->declare_parameter("publish.nav.velned", getRosBoolean(this, "publish.nav.all"));

  this->declare_parameter("publish.rxm.all", getRosBoolean(this, "publish.all"));
  this->declare_parameter("publish.rxm.almRaw", getRosBoolean(this, "publish.rxm.all"));
  this->declare_parameter("publish.rxm.eph", getRosBoolean(this, "publish.rxm.all"));
  this->declare_parameter("publish.rxm.rtcm", getRosBoolean(this, "publish.rxm.all"));
  this->declare_parameter("publish.rxm.raw", getRosBoolean(this, "publish.rxm.all"));
  this->declare_parameter("publish.rxm.sfrb", getRosBoolean(this, "publish.rxm.all"));

  this->declare_parameter("publish.aid.all", getRosBoolean(this, "publish.all"));
  this->declare_parameter("publish.aid.alm", getRosBoolean(this, "publish.aid.all"));
  this->declare_parameter("publish.aid.eph", getRosBoolean(this, "publish.aid.all"));
  this->declare_parameter("publish.aid.hui", getRosBoolean(this, "publish.aid.all"));

  this->declare_parameter("publish.mon.all", getRosBoolean(this, "publish.all"));
  this->declare_parameter("publish.mon.hw", getRosBoolean(this, "publish.mon.all"));

  this->declare_parameter("publish.tim.tm2", false);

  // INF parameters
  this->declare_parameter("inf.all", true);
  this->declare_parameter("inf.debug", false);
  this->declare_parameter("inf.error", getRosBoolean(this, "inf.all"));
  this->declare_parameter("inf.notice", getRosBoolean(this, "inf.all"));
  this->declare_parameter("inf.test", getRosBoolean(this, "inf.all"));
  this->declare_parameter("inf.warning", getRosBoolean(this, "inf.all"));

  // ESF parameters
  this->declare_parameter("publish.esf.all", true);
  this->declare_parameter("publish.esf.ins", getRosBoolean(this, "publish.esf.all"));
  this->declare_parameter("publish.esf.meas", getRosBoolean(this, "publish.esf.all"));
  this->declare_parameter("publish.esf.raw", getRosBoolean(this, "publish.esf.all"));
  this->declare_parameter("publish.esf.status", getRosBoolean(this, "publish.esf.all"));

  // HNR parameters
  this->declare_parameter("publish.hnr.pvt", true);

  this->declare_parameter("tmode3", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("arp.position", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("arp.position_hp", rclcpp::PARAMETER_INTEGER_ARRAY);
  this->declare_parameter("arp.acc", 0.0);
  this->declare_parameter("arp.lla_flag", false);

  this->declare_parameter("diagnostic_period", kDiagnosticPeriod);

  // Create publishers based on parameters
  if (getRosBoolean(this, "publish.nav.status")) {
    nav_status_pub_ = this->create_publisher<ublox_msgs::msg::NavSTATUS>("navstatus", 1);
  }
  if (getRosBoolean(this, "publish.nav.posecef")) {
    nav_posecef_pub_ = this->create_publisher<ublox_msgs::msg::NavPOSECEF>("navposecef", 1);
  }
  if (getRosBoolean(this, "publish.nav.clock")) {
    nav_clock_pub_ = this->create_publisher<ublox_msgs::msg::NavCLOCK>("navclock", 1);
  }
  if (getRosBoolean(this, "publish.nav.clock")) {
    nav_clock_pub_ = this->create_publisher<ublox_msgs::msg::NavCLOCK>("navclock", 1);
  }
  if (getRosBoolean(this, "publish.aid.alm")) {
    aid_alm_pub_ = this->create_publisher<ublox_msgs::msg::AidALM>("aidalm", 1);
  }
  if (getRosBoolean(this, "publish.aid.eph")) {
    aid_eph_pub_ = this->create_publisher<ublox_msgs::msg::AidEPH>("aideph", 1);
  }
  if (getRosBoolean(this, "publish.aid.hui")) {
    aid_hui_pub_ = this->create_publisher<ublox_msgs::msg::AidHUI>("aidhui", 1);
  }
}

void UbloxNode::keepAlive() {
  // Poll version message to keep UDP socket active
  gps_->poll(ublox_msgs::Class::MON, ublox_msgs::Message::MON::VER);
}

void UbloxNode::pollMessages() {
  static std::vector<uint8_t> payload(1, 1);
  if (getRosBoolean(this, "publish.aid.alm")) {
    gps_->poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM, payload);
  }
  if (getRosBoolean(this, "publish.aid.eph")) {
    gps_->poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH, payload);
  }
  if (getRosBoolean(this, "publish.aid.hui")) {
    gps_->poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI);
  }

  payload[0]++;
  if (payload[0] > 32) {
    payload[0] = 1;
  }
}

void UbloxNode::printInf(const ublox_msgs::msg::Inf &m, uint8_t id) {
  if (id == ublox_msgs::Message::INF::ERROR) {
    RCLCPP_ERROR(this->get_logger(), "INF: %s", std::string(m.str.begin(), m.str.end()).c_str());
  } else if (id == ublox_msgs::Message::INF::WARNING) {
    RCLCPP_WARN(this->get_logger(), "INF: %s", std::string(m.str.begin(), m.str.end()).c_str());
  } else if (id == ublox_msgs::Message::INF::DEBUG) {
    RCLCPP_DEBUG(this->get_logger(), "INF: %s", std::string(m.str.begin(), m.str.end()).c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "INF: %s", std::string(m.str.begin(), m.str.end()).c_str());
  }
}

void UbloxNode::subscribe() {
  RCLCPP_DEBUG(this->get_logger(), "Subscribing to U-Blox messages");
  // subscribe messages

  // Nav Messages
  if (getRosBoolean(this, "publish.nav.status")) {
    gps_->subscribe<ublox_msgs::msg::NavSTATUS>([this](const ublox_msgs::msg::NavSTATUS &m) { nav_status_pub_->publish(m); },
                                           1);
  }

  if (getRosBoolean(this, "publish.nav.posecef")) {
    gps_->subscribe<ublox_msgs::msg::NavPOSECEF>([this](const ublox_msgs::msg::NavPOSECEF &m) { nav_posecef_pub_->publish(m); },
                                            1);
  }

  if (getRosBoolean(this, "publish.nav.clock")) {
    gps_->subscribe<ublox_msgs::msg::NavCLOCK>([this](const ublox_msgs::msg::NavCLOCK &m) { nav_clock_pub_->publish(m); },
                                          1);
  }

  // INF messages
  if (getRosBoolean(this, "inf.debug")) {
    gps_->subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::DEBUG),
        ublox_msgs::Message::INF::DEBUG);
  }

  if (getRosBoolean(this, "inf.error")) {
    gps_->subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::ERROR),
        ublox_msgs::Message::INF::ERROR);
  }

  if (getRosBoolean(this, "inf.notice")) {
    gps_->subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::NOTICE),
        ublox_msgs::Message::INF::NOTICE);
  }

  if (getRosBoolean(this, "inf.test")) {
    gps_->subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::TEST),
        ublox_msgs::Message::INF::TEST);
  }

  if (getRosBoolean(this, "inf.warning")) {
    gps_->subscribeId<ublox_msgs::msg::Inf>(
        std::bind(&UbloxNode::printInf, this, std::placeholders::_1,
                    ublox_msgs::Message::INF::WARNING),
        ublox_msgs::Message::INF::WARNING);
  }

  // AID messages
  if (getRosBoolean(this, "publish.aid.alm")) {
    gps_->subscribe<ublox_msgs::msg::AidALM>([this](const ublox_msgs::msg::AidALM &m) { aid_alm_pub_->publish(m); },
                                        1);
  }

  if (getRosBoolean(this, "publish.aid.eph")) {
    gps_->subscribe<ublox_msgs::msg::AidEPH>([this](const ublox_msgs::msg::AidEPH &m) { aid_eph_pub_->publish(m); },
                                        1);
  }

  if (getRosBoolean(this, "publish.aid.hui")) {
    gps_->subscribe<ublox_msgs::msg::AidHUI>([this](const ublox_msgs::msg::AidHUI &m) { aid_hui_pub_->publish(m); },
                                        1);
  }

  for (const std::shared_ptr<ComponentInterface> & component : components_) {
    component->subscribe(gps_);
  }
}

void UbloxNode::initializeRosDiagnostics() {
  for (const std::shared_ptr<ComponentInterface> & component : components_) {
    component->initializeRosDiagnostics();
  }
}

void UbloxNode::processMonVer() {
  ublox_msgs::msg::MonVER monVer;
  if (!gps_->poll(monVer)) {
    throw std::runtime_error("Failed to poll MonVER & set relevant settings");
  }

  RCLCPP_INFO(this->get_logger(), "%s, HW VER: %s",
              std::string(monVer.sw_version.begin(), monVer.sw_version.end()).c_str(),
              std::string(monVer.hw_version.begin(), monVer.hw_version.end()).c_str());
  // Convert extension to vector of strings
  std::vector<std::string> extensions;
  extensions.reserve(monVer.extension.size());
  for (std::size_t i = 0; i < monVer.extension.size(); ++i) {  // NOLINT(modernize-loop-convert)
    RCLCPP_DEBUG(this->get_logger(), "%s",
                 std::string(monVer.extension[i].field.begin(), monVer.extension[i].field.end()).c_str());
    // Find the end of the string (null character)
    unsigned char* end = std::find(monVer.extension[i].field.begin(),
                                   monVer.extension[i].field.end(), '\0');
    extensions.emplace_back(std::string(monVer.extension[i].field.begin(), end));
  }

  // Get the protocol version
  for (const std::string & ext : extensions) {
    std::size_t found = ext.find("PROTVER");
    if (found != std::string::npos) {
      const char * sub = ext.substr(8, ext.size()-8).c_str();
      char * end{nullptr};
      protocol_version_ = std::strtof(sub, &end);
      if (protocol_version_ == HUGE_VALF || (protocol_version_ == 0 && end == sub)) {
        // strtof failed to convert either via overflow or no conversion possible.
        // Throw an error.
        throw std::runtime_error("Failed to parse protocol version from extensions");
      }
      break;
    }
  }
  if (protocol_version_ == 0.0) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse MonVER and determine protocol version. %s",
                "Defaulting to firmware version 6.");
  }
  addFirmwareInterface();

  if (protocol_version_ < 18.0) {
    // Final line contains supported GNSS delimited by ;
    std::vector<std::string> strs;
    if (extensions.size() > 0) {
      strs = stringSplit(extensions[extensions.size() - 1], ";");
    }
    for (const std::string & str : strs) {
      gnss_->add(str);
    }
  } else {
    for (std::size_t i = 0; i < extensions.size(); ++i) {
      std::vector<std::string> strs;
      // Up to 2nd to last line
      if (i <= extensions.size() - 2) {
        strs = stringSplit(extensions[i], "=");
        if (strs.size() > 1) {
          if (strs[0] == "FWVER") {
            if (strs[1].length() > 8) {
              addProductInterface(strs[1].substr(0, 3), strs[1].substr(8, 10));
            } else {
              addProductInterface(strs[1].substr(0, 3));
            }
            continue;
          }
        }
      }
      // Last 1-2 lines contain supported GNSS
      if (i >= extensions.size() - 2) {
        strs = stringSplit(extensions[i], ";");
        for (const std::string & str : strs) {
          gnss_->add(str);
        }
      }
    }
  }
}

bool UbloxNode::configureUblox() {
  try {
    if (!gps_->isInitialized()) {
      throw std::runtime_error("Failed to initialize.");
    }
    if (load_.load_mask != 0) {
      RCLCPP_DEBUG(this->get_logger(), "Loading u-blox configuration from memory. %u", load_.load_mask);
      if (!gps_->configure(load_)) {
        throw std::runtime_error(std::string("Failed to load configuration ") +
                                 "from memory");
      }
      if (load_.load_mask & ublox_msgs::msg::CfgCFG::MASK_IO_PORT) {
        RCLCPP_DEBUG(this->get_logger(), "Loaded I/O configuration from memory, resetting serial %s",
          "communications.");
        std::chrono::seconds wait(kResetWait);
        gps_->reset(wait);
        if (!gps_->isConfigured()) {
          throw std::runtime_error(std::string("Failed to reset serial I/O") +
            "after loading I/O configurations from device memory.");
        }
      }
    }

    if (getRosBoolean(this, "config_on_startup")) {
      if (set_usb_) {
        gps_->configUsb(usb_tx_, usb_in_, usb_out_);
      }
      if (!gps_->configRate(meas_rate_, nav_rate_)) {
        std::stringstream ss;
        ss << "Failed to set measurement rate to " << meas_rate_
          << "ms and navigation rate to " << nav_rate_;
        throw std::runtime_error(ss.str());
      }
      // If device doesn't have SBAS, will receive NACK (causes exception)
      if (gnss_->isSupported("SBAS")) {
        if (!gps_->configSbas(getRosBoolean(this, "gnss.sbas"), sbas_usage_, max_sbas_)) {
          throw std::runtime_error(std::string("Failed to ") +
                                  (getRosBoolean(this, "gnss.sbas") ? "enable" : "disable") +
                                  " SBAS.");
        }
      }
      if (!gps_->setPpp(getRosBoolean(this, "enable_ppp"))) {
        throw std::runtime_error(std::string("Failed to ") +
                                (getRosBoolean(this, "enable_ppp") ? "enable" : "disable")
                                + " PPP.");
      }
      if (!gps_->setDynamicModel(dmodel_)) {
        throw std::runtime_error("Failed to set model: " + dynamic_model_ + ".");
      }
      if (!gps_->setFixMode(fmode_)) {
        throw std::runtime_error("Failed to set fix mode: " + fix_mode_ + ".");
      }
      if (!gps_->setDeadReckonLimit(dr_limit_)) {
        std::stringstream ss;
        ss << "Failed to set dead reckoning limit: " << dr_limit_ << ".";
        throw std::runtime_error(ss.str());
      }
      if (getRosBoolean(this, "dat.set") && !gps_->configure(cfg_dat_)) {
        throw std::runtime_error("Failed to set user-defined datum.");
      }
      // Configure each component
      for (const std::shared_ptr<ComponentInterface> & component : components_) {
        if (!component->configureUblox(gps_)) {
          return false;
        }
      }
    }
    if (save_.save_mask != 0) {
      RCLCPP_DEBUG(this->get_logger(), "Saving the u-blox configuration, mask %u, device %u",
                   save_.save_mask, save_.device_mask);
      if (!gps_->configure(save_)) {
        RCLCPP_ERROR(this->get_logger(), "u-blox unable to save configuration to non-volatile memory");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Error configuring u-blox: %s", e.what());
    return false;
  }
  return true;
}

void UbloxNode::configureInf() {
  ublox_msgs::msg::CfgINF msg;
  // Subscribe to UBX INF messages
  ublox_msgs::msg::CfgINFBlock block;
  block.protocol_id = ublox_msgs::msg::CfgINFBlock::PROTOCOL_ID_UBX;
  // Enable desired INF messages on each UBX port
  uint8_t mask = (getRosBoolean(this, "inf.error") ? ublox_msgs::msg::CfgINFBlock::INF_MSG_ERROR : 0) |
                 (getRosBoolean(this, "inf.warning") ? ublox_msgs::msg::CfgINFBlock::INF_MSG_WARNING : 0) |
                 (getRosBoolean(this, "inf.notice") ? ublox_msgs::msg::CfgINFBlock::INF_MSG_NOTICE : 0) |
                 (getRosBoolean(this, "inf.test") ? ublox_msgs::msg::CfgINFBlock::INF_MSG_TEST : 0) |
                 (getRosBoolean(this, "inf.debug") ? ublox_msgs::msg::CfgINFBlock::INF_MSG_DEBUG : 0);
  for (size_t i = 0; i < block.inf_msg_mask.size(); i++) {  // NOLINT(modernize-loop-convert)
    block.inf_msg_mask[i] = mask;
  }

  msg.blocks.push_back(block);

  // IF NMEA is enabled
  if (uart_in_ & ublox_msgs::msg::CfgPRT::PROTO_NMEA) {
    ublox_msgs::msg::CfgINFBlock block;
    block.protocol_id = ublox_msgs::msg::CfgINFBlock::PROTOCOL_ID_NMEA;
    // Enable desired INF messages on each NMEA port
    for (size_t i = 0; i < block.inf_msg_mask.size(); i++) {  // NOLINT(modernize-loop-convert)
      block.inf_msg_mask[i] = mask;
    }
    msg.blocks.push_back(block);
  }

  RCLCPP_DEBUG(this->get_logger(), "Configuring INF messages");
  if (!gps_->configure(msg)) {
    RCLCPP_WARN(this->get_logger(), "Failed to configure INF messages");
  }
}

void UbloxNode::initializeIo() {
  gps_->setConfigOnStartup(getRosBoolean(this, "config_on_startup"));

  std::smatch match;
  if (std::regex_match(device_, match,
                       std::regex("(tcp|udp)://(.+):(\\d+)"))) {
    std::string proto(match[1]);
    if (proto == "tcp") {
      std::string host(match[2]);
      std::string port(match[3]);
      RCLCPP_INFO(this->get_logger(), "Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(),
               port.c_str());
      gps_->initializeTcp(host, port);
    } else if (proto == "udp") {
      std::string host(match[2]);
      std::string port(match[3]);
      RCLCPP_INFO(this->get_logger(), "Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(),
               port.c_str());
      gps_->initializeUdp(host, port);
    } else {
      throw std::runtime_error("Protocol '" + proto + "' is unsupported");
    }
  } else {
    gps_->initializeSerial(device_, baudrate_, uart_in_, uart_out_);
  }

  // raw data stream logging
  if (getRosBoolean(this, "raw_data_stream.enable")) {
    if (raw_data_stream_pa_->isEnabled()) {
      gps_->setRawDataCallback(
        std::bind(&RawDataStreamPa::ubloxCallback, raw_data_stream_pa_.get(),
        std::placeholders::_1, std::placeholders::_2));
      raw_data_stream_pa_->initialize();
    }
  }
}

void UbloxNode::initialize() {
  // Params must be set before initializing IO
  getRosParams();

  // configure diagnostic updater for frequency
  freq_diag_ = std::make_shared<FixDiagnostic>(std::string("fix"), kFixFreqTol,
                                               kFixFreqWindow, kTimeStampStatusMin, nav_rate_, meas_rate_, updater_);


  initializeIo();
  // Must process Mon VER before setting firmware/hardware params
  processMonVer();
  if (protocol_version_ <= 14.0) {
    if (getRosBoolean(this, "raw_data")) {
      components_.push_back(std::make_shared<RawDataProduct>(nav_rate_, meas_rate_, updater_, this));
    }
  }
  // Must set firmware & hardware params before initializing diagnostics
  for (const std::shared_ptr<ComponentInterface> & component : components_) {
    component->getRosParams();
  }
  // Do this last
  initializeRosDiagnostics();

  if (configureUblox()) {
    RCLCPP_INFO(this->get_logger(), "U-Blox configured successfully.");
    // Subscribe to all U-Blox messages
    subscribe();
    // Configure INF messages (needs INF params, call after subscribing)
    configureInf();

    if (device_.substr(0, 6) == "udp://") {
      // Setup timer to poll version message to keep UDP socket active
      keep_alive_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(kKeepAlivePeriod * 1000.0)),
                                            std::bind(&UbloxNode::keepAlive, this));
    }

    poller_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(kPollDuration * 1000.0)),
                                      std::bind(&UbloxNode::pollMessages, this));
  }
}

void UbloxNode::shutdown() {
  if (gps_->isInitialized()) {
    gps_->close();
    RCLCPP_INFO(this->get_logger(), "Closed connection to %s.", device_.c_str());
  }
}

UbloxNode::~UbloxNode() {
  shutdown();
}

}  // namespace ublox_node

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_node::UbloxNode)
