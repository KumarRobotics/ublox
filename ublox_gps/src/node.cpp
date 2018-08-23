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
#include <ublox_gps/utils.h>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/serial_port.hpp>

#include <boost/regex.hpp>

#include <ros/ros.h>
#include <ros/serialization.h>
#include <ublox_msgs/CfgGNSS.h>
#include <ublox_msgs/CfgPRT.h>
#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavSOL.h>
#include <ublox_msgs/NavSTATUS.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavSAT.h>
#include <ublox_msgs/ublox_msgs.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "ublox_gps/node.h"
const static uint32_t kROSQueueSize = 1;

using namespace ublox_gps;
using namespace ublox_node;


int UbloxNode::setParams() {
  int qzss_sig_cfg_default =
      ublox_msgs::CfgGNSS_Block::SIG_CFG_QZSS_L1CA;
  int uart_in_default = ublox_msgs::CfgPRT::PROTO_UBX
                        | ublox_msgs::CfgPRT::PROTO_NMEA
                        | ublox_msgs::CfgPRT::PROTO_RTCM;
  int uart_out_default = ublox_msgs::CfgPRT::PROTO_UBX;
  ros::NodeHandle param_nh("~");
  param_nh.param("device", device_, std::string("/dev/ttyACM0"));
  param_nh.param("frame_id", frame_id_, std::string("gps"));
  param_nh.param("baudrate", baudrate_, 9600); // UART 1 baudrate
  param_nh.param("uart_in", uart_in_, uart_in_default); // UART 1 in protocol
  param_nh.param("uart_out", uart_out_, uart_out_default); // UART 1 out protocol
  param_nh.param("rate", rate_, 4);  // in Hz
  param_nh.param("nav_rate", nav_rate_, 1);  // # of measurement rate cycles
  param_nh.param("rtcm_ids", rtcm_ids_, rtcm_ids_);  // RTCM IDs to configure
  param_nh.param("rtcm_rate", rtcm_rate_, 1);  // in Hz, same for all RTCM IDs
  param_nh.param("enable_gps", enable_gps_, true);
  param_nh.param("enable_galileo", enable_galileo_, false);
  param_nh.param("enable_beidou", enable_beidou_, false);
  param_nh.param("enable_imes", enable_imes_, false);
  param_nh.param("enable_glonass", enable_glonass_, false);
  param_nh.param("enable_ppp", enable_ppp_, false); // Advanced setting
  param_nh.param("enable_qzss", enable_qzss_, false);
  param_nh.param("qzss_sig_cfg", qzss_sig_cfg_,
                 qzss_sig_cfg_default);
  param_nh.param("enable_sbas", enable_sbas_, false);
  param_nh.param("max_sbas", max_sbas_, 0); // Maximum number of SBAS channels
  param_nh.param("sbas_usage", sbas_usage_, 0);
  param_nh.param("dynamic_model", dynamic_model_, std::string("portable"));
  param_nh.param("fix_mode", fix_mode_, std::string("auto"));
  param_nh.param("dr_limit", dr_limit_, 0); // Dead reckoning limit

  if (enable_ppp_) {
    ROS_WARN("Warning: PPP is enabled - this is an expert setting.");
  }

  if (rate_ <= 0) {
    throw std::runtime_error("Invalid settings: rate must be > 0");
  }

  if (dr_limit_ < 0 || dr_limit_ > 255) {
    throw std::runtime_error("Invalid settings: dr_limit must be between 0 and 255");
  }

  try {
    dmodel = ublox_gps::modelFromString(dynamic_model_);
    fmode = ublox_gps::fixModeFromString(fix_mode_);
  } catch (std::exception& e) {
    throw std::runtime_error("Invalid dynamic model or fix mode settings");
  }

  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::NavSatStatus::SERVICE_GPS
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GLONASS
      + (enable_beidou_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_COMPASS
      + (enable_galileo_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GALILEO;

  //  measurement rate param for ublox, units of ms
  meas_rate_ = 1000 / rate_;

  return 0;
}

template <typename MessageT>
void UbloxNode::publish(const MessageT& m, const std::string& topic) {
  static ros::Publisher publisher =
      nh->advertise<MessageT>(topic, kROSQueueSize);
  publisher.publish(m);
}

void UbloxNode::initializeIo() {
  boost::smatch match;

  if (boost::regex_match(device_, match,
                         boost::regex("(tcp|udp)://(.+):(\\d+)"))) {
    std::string proto(match[1]);
    std::string host(match[2]);
    std::string port(match[3]);
    ROS_INFO("Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(),
             port.c_str());

    if (proto == "tcp") {
      boost::asio::ip::tcp::resolver::iterator endpoint;

      try {
        boost::asio::ip::tcp::resolver resolver(io_service);
        endpoint =
            resolver.resolve(boost::asio::ip::tcp::resolver::query(host, port));
      } catch (std::runtime_error& e) {
        throw std::runtime_error("Could not resolve" + host + " " +
                                 port + " " + e.what());
      }

      boost::asio::ip::tcp::socket* socket =
          new boost::asio::ip::tcp::socket(io_service);
      tcp_handle.reset(socket);

      try {
        socket->connect(*endpoint);
      } catch (std::runtime_error& e) {
        throw std::runtime_error("Could not connect to " +
                                 endpoint->host_name() + ":" +
                                 endpoint->service_name() + ": " + e.what());
      }

      ROS_INFO("Connected to %s:%s.", endpoint->host_name().c_str(),
               endpoint->service_name().c_str());
      gps.initialize(*socket, io_service, baudrate_, uart_in_, uart_out_);
    } else {
      throw std::runtime_error("Protocol '" + proto + "' is unsupported");
    }
  } else {
    boost::asio::serial_port* serial = new boost::asio::serial_port(io_service);
    serial_handle.reset(serial);

    // open serial port
    try {
      serial->open(device_);
    } catch (std::runtime_error& e) {
      throw std::runtime_error("Could not open serial port :"
                               + device_ + " " + e.what());
    }

    ROS_INFO("Opened serial port %s", device_.c_str());
    gps.configUart1(baudrate_, uart_in_, uart_out_);
    gps.initialize(*serial, io_service, baudrate_, uart_in_, uart_out_);
  }
}

void UbloxNode::pollMessages(const ros::TimerEvent& event) {
  static std::vector<uint8_t> payload(1, 1);
  if (enabled["aid_alm"]) {
    gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM, payload);
  }
  if (enabled["aid_eph"]) {
    gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH, payload);
  }
  if (enabled["aid_hui"]) {
    gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI);
  }
  payload[0]++;
  if (payload[0] > 32) {
    payload[0] = 1;
  }
}

void UbloxNode::subscribeAll() {
  ros::NodeHandle param_nh("~");
  // subscribe messages
  param_nh.param("all", enabled["all"], false);
  param_nh.param("rxm", enabled["rxm"], false);
  param_nh.param("aid", enabled["aid"], false);

  param_nh.param("nav_sol", enabled["nav_sol"], true);

  if (enabled["nav_sol"])
    gps.subscribe<ublox_msgs::NavSOL>(boost::bind(
        &UbloxNode::publish<ublox_msgs::NavSOL>, this, _1, "navsol"), 1);

  param_nh.param("nav_status", enabled["nav_status"], true);
  if (enabled["nav_status"])
    gps.subscribe<ublox_msgs::NavSTATUS>(boost::bind(
        &UbloxNode::publish<ublox_msgs::NavSTATUS>, this, _1, "navstatus"), 1);

  param_nh.param("nav_svinfo", enabled["nav_svinfo"], enabled["all"]);
  if (enabled["nav_svinfo"])
    gps.subscribe<ublox_msgs::NavSVINFO>(boost::bind(
        &UbloxNode::publish<ublox_msgs::NavSVINFO>, this, _1, "navsvinfo"), 1);

  param_nh.param("nav_clk", enabled["nav_clk"], enabled["all"]);
  if (enabled["nav_clk"])
    gps.subscribe<ublox_msgs::NavCLOCK>(boost::bind(
        &UbloxNode::publish<ublox_msgs::NavCLOCK>, this, _1, "navclock"), 1);

  param_nh.param("rxm_eph", enabled["rxm_eph"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_eph"])
    gps.subscribe<ublox_msgs::RxmEPH>(boost::bind(
        &UbloxNode::publish<ublox_msgs::RxmEPH>, this, _1, "rxmeph"), 1);

  param_nh.param("rxm_alm", enabled["rxm_alm"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_alm"])
    gps.subscribe<ublox_msgs::RxmALM>(boost::bind(
        &UbloxNode::publish<ublox_msgs::RxmALM>, this, _1, "rxmalm"), 1);

  param_nh.param("nav_posllh", enabled["nav_posllh"], true);
  if (enabled["nav_posllh"])
    gps.subscribe<ublox_msgs::NavPOSLLH>(boost::bind(
        &UbloxNode::publish<ublox_msgs::NavPOSLLH>, this, _1, "navposllh"), 1);

  param_nh.param("nav_posecef", enabled["nav_posecef"], true);
  if (enabled["nav_posecef"])
    gps.subscribe<ublox_msgs::NavPOSECEF>(boost::bind(
        &UbloxNode::publish<ublox_msgs::NavPOSECEF>, this, _1, "navposecef"), 1);

  param_nh.param("nav_velned", enabled["nav_velned"], true);
  if (enabled["nav_velned"])
    gps.subscribe<ublox_msgs::NavVELNED>(boost::bind(
        &UbloxNode::publish<ublox_msgs::NavVELNED>, this, _1, "navvelned"), 1);

  param_nh.param("aid_alm", enabled["aid_alm"],
                 enabled["all"] || enabled["aid"]);
  if (enabled["aid_alm"])
    gps.subscribe<ublox_msgs::AidALM>(boost::bind(
        &UbloxNode::publish<ublox_msgs::AidALM>, this, _1, "aidalm"), 1);

  param_nh.param("aid_eph", enabled["aid_eph"],
                 enabled["all"] || enabled["aid"]);
  if (enabled["aid_eph"])
    gps.subscribe<ublox_msgs::AidEPH>(boost::bind(
        &UbloxNode::publish<ublox_msgs::AidEPH>, this, _1, "aideph"), 1);

  param_nh.param("aid_hui", enabled["aid_hui"],
                 enabled["all"] || enabled["aid"]);
  if (enabled["aid_hui"])
    gps.subscribe<ublox_msgs::AidHUI>(boost::bind(
        &UbloxNode::publish<ublox_msgs::AidHUI>, this, _1, "aidhui"), 1);

  // Subscribe to Nav SAT
  param_nh.param("nav_sat", enabled["nav_sat"], enabled["all"]);
  if (enabled["nav_sat"])
    gps.subscribe<ublox_msgs::NavSAT>(boost::bind(
        &UbloxNode::publish<ublox_msgs::NavSAT>, this, _1, "navsat"), 1);

  // Subscribe to MEASX messages
  param_nh.param("rxm_meas", enabled["rxm_meas"],
               enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_meas"])
    gps.subscribe<ublox_msgs::RxmMEASX>(boost::bind(
        &UbloxNode::publish<ublox_msgs::RxmMEASX>, this, _1, "rxmmeas"), 1);

  subscribeVersion();
}

void UbloxNode::initDiagnostics() {
  if (!nh->hasParam("diagnostic_period")) {
    nh->setParam("diagnostic_period", 0.2);  //  5Hz diagnostic period
  }

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("ublox");

  const double target_freq = rate_;  //  actual update frequency
  double min_freq = target_freq;
  double max_freq = target_freq;
  double timeStampStatusMax = meas_rate_ * 1e-3 * 0.05;
  diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq,
                                                      kTolerance, kWindow);
  diagnostic_updater::TimeStampStatusParam time_param(kTimeStampStatusMin,
                                                      timeStampStatusMax);
  freq_diag.reset(new diagnostic_updater::TopicDiagnostic(
      std::string("fix"), *updater, freq_param, time_param));

  // configure diagnostic updater for frequency
  updater->add("fix", this, &UbloxNode::fixDiagnostic);
  updater->force_update();
}

bool UbloxNode::configureUblox() {
  try {
    if (!gps.isInitialized()) {
      throw std::runtime_error("Failed to initialize.");
    }
    ublox_msgs::MonVER monVer;
    if (gps.poll(monVer)) {
      ROS_INFO("%s, HW VER: %s", monVer.swVersion.c_array(),
               monVer.hwVersion.c_array());
      for(std::size_t i = 0; i < monVer.extension.size(); ++i) {
        ROS_INFO("%s", monVer.extension[i].field.c_array());
      }
    } else {
      ROS_WARN("Failed to poll MonVER");
    }
    if (!gps.configRate(meas_rate_, nav_rate_)) {
      std::stringstream ss;
      ss << "Failed to set measurement rate to " << meas_rate_
         << "ms and navigation rate to " << nav_rate_;
      throw std::runtime_error(ss.str());
    }
    if(!gps.configRtcm(rtcm_ids_, rtcm_rate_)) {
      throw std::runtime_error("Failed to set RTCM rates");
    }
    // If device doesn't have SBAS, will receive NACK (causes exception)
    if(enable_sbas_) {
      if (!gps.enableSBAS(enable_sbas_, sbas_usage_, max_sbas_)) {
        throw std::runtime_error(std::string("Failed to ") +
                                 ((enable_sbas_) ? "enable" : "disable") +
                                 " SBAS.");
      }
    }
    if (!gps.setPPPEnabled(enable_ppp_)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_ppp_) ? "enable" : "disable")
                               + " PPP.");
    }
    if (!gps.setDynamicModel(dmodel)) {
      throw std::runtime_error("Failed to set model: " + dynamic_model_ + ".");
    }
    if (!gps.setFixMode(fmode)) {
      throw std::runtime_error("Failed to set fix mode: " + fix_mode_ + ".");
    }
    if (!gps.setDeadReckonLimit(dr_limit_)) {
      std::stringstream ss;
      ss << "Failed to set dead reckoning limit: " << dr_limit_ << ".";
      throw std::runtime_error(ss.str());
    }

    configureGnss();
  } catch (std::exception& e) {
    ROS_ERROR("Error configuring device: %s", e.what());
    return false;
  }
  return true;
}


void UbloxNode::initialize() {
  setParams();
  initDiagnostics();
  initializeIo();

  if (configureUblox()) {
    ROS_INFO("U-Blox configured successfully.");
    // Subscribe to all U-Blox messages
    subscribeAll();

    ros::Timer poller;
    poller = nh->createTimer(ros::Duration(kPollDuration),
                             &UbloxNode::pollMessages,
                             this);
    poller.start();
    ros::spin();
  }

  if (gps.isInitialized()) {
    gps.close();
    ROS_INFO("Closed connection to %s.", device_.c_str());
  }
}

/** Ublox Version 6 **/

UbloxNode6::UbloxNode6(boost::shared_ptr<ros::NodeHandle> _nh) {
  setNh(_nh);
  initialize();
}

void UbloxNode6::configureGnss() {
  ROS_WARN("ublox_version < 7, ignoring GNSS settings");
}

void UbloxNode6::subscribeVersion() {
  ros::NodeHandle param_nh("~");
  // Subscribe to RXM Raw
  param_nh.param("rxm_raw", enabled["rxm_raw"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_raw"])
    gps.subscribe<ublox_msgs::RxmRAW>(boost::bind(
      &UbloxNode::publish<ublox_msgs::RxmRAW>, this, _1, "rxmraw"), 1);

  // Subscribe to RXM SFRB
  param_nh.param("rxm_sfrb", enabled["rxm_sfrb"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_sfrb"])
    gps.subscribe<ublox_msgs::RxmSFRB>(boost::bind(
      &UbloxNode::publish<ublox_msgs::RxmSFRB>, this, _1, "rxmsfrb"), 1);
}

void UbloxNode6::fixDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  //  check the last pos message, convert to diagnostic
  if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "No fix";
  } else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  } else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "2D fix";
  } else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "3D fix";
  } else if (status.gpsFix ==
             ublox_msgs::NavSTATUS::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  } else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Time fix only";
  }

  //  append last fix position
  stat.add("iTOW", last_nav_pos_.iTOW);
  stat.add("lon", last_nav_pos_.lon);
  stat.add("lat", last_nav_pos_.lat);
  stat.add("height", last_nav_pos_.height);
  stat.add("hMSL", last_nav_pos_.hMSL);
  stat.add("hAcc", last_nav_pos_.hAcc);
  stat.add("vAcc", last_nav_pos_.vAcc);
  stat.add("numSV", num_svs_used_);
}

void UbloxNode6::publishNavPOSLLH(const ublox_msgs::NavPOSLLH& m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavPOSLLH>("navposllh", kROSQueueSize);
  publisher.publish(m);

  // Position message
  static ros::Publisher fixPublisher =
      nh->advertise<sensor_msgs::NavSatFix>("fix", kROSQueueSize);
  if (m.iTOW == last_nav_vel_.iTOW) {
    //  use last timestamp
    fix.header.stamp = velocity.header.stamp;
  } else {
    //  new timestamp
    fix.header.stamp = ros::Time::now();
  }
  fix.header.frame_id = frame_id_;
  fix.latitude = m.lat * 1e-7;
  fix.longitude = m.lon * 1e-7;
  fix.altitude = m.height * 1e-3;
  if (status.gpsFix >= status.GPS_2D_FIX)
    fix.status.status = fix.status.STATUS_FIX;
  else
    fix.status.status = fix.status.STATUS_NO_FIX;

  //  calculate covariance (convert from mm to m too)
  const double stdH = (m.hAcc / 1000.0) * 3.0;
  const double stdV = (m.vAcc / 1000.0) * 3.0;

  fix.position_covariance[0] = stdH * stdH;
  fix.position_covariance[4] = stdH * stdH;
  fix.position_covariance[8] = stdV * stdV;
  fix.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix.status.service = fix.status.SERVICE_GPS;
  fixPublisher.publish(fix);
  last_nav_pos_ = m;
  //  update diagnostics
  freq_diag->tick(fix.header.stamp);
  updater->update();
}

void UbloxNode6::publishNavVELNED(const ublox_msgs::NavVELNED& m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavVELNED>("navvelned", kROSQueueSize);
  publisher.publish(m);

  // Example geometry message
  static ros::Publisher velocityPublisher =
      nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity",
                                                               kROSQueueSize);
  if (m.iTOW == last_nav_pos_.iTOW) {
    //  use same time as las navposllh message
    velocity.header.stamp = fix.header.stamp;
  } else {
    //  create a new timestamp
    velocity.header.stamp = ros::Time::now();
  }
  velocity.header.frame_id = frame_id_;

  //  convert to XYZ linear velocity
  velocity.twist.twist.linear.x = m.velE / 100.0;
  velocity.twist.twist.linear.y = m.velN / 100.0;
  velocity.twist.twist.linear.z = -m.velD / 100.0;

  const double stdSpeed = (m.sAcc / 100.0) * 3;

  const int cols = 6;
  velocity.twist.covariance[cols * 0 + 0] = stdSpeed * stdSpeed;
  velocity.twist.covariance[cols * 1 + 1] = stdSpeed * stdSpeed;
  velocity.twist.covariance[cols * 2 + 2] = stdSpeed * stdSpeed;
  velocity.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

  velocityPublisher.publish(velocity);
  last_nav_vel_ = m;
}

void UbloxNode6::publishNavSOL(const ublox_msgs::NavSOL& m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavSOL>("navsol", kROSQueueSize);
  num_svs_used_ = m.numSV;  //  number of satellites used
  publisher.publish(m);
}

/** Ublox Firmware Version >=7 **/
void UbloxNode7Plus::fixDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  //  check the last message, convert to diagnostic
  if (last_nav_pvt_.fixType == ublox_msgs::NavSTATUS::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "No fix";
  } else if (last_nav_pvt_.fixType ==
             ublox_msgs::NavSTATUS::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  } else if (last_nav_pvt_.fixType == ublox_msgs::NavSTATUS::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "2D fix";
  } else if (last_nav_pvt_.fixType == ublox_msgs::NavSTATUS::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "3D fix";
  } else if (last_nav_pvt_.fixType ==
             ublox_msgs::NavSTATUS::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  } else if (last_nav_pvt_.fixType == ublox_msgs::NavSTATUS::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Time fix only";
  }

  //  append last fix position
  stat.add("iTOW", last_nav_pvt_.iTOW);
  stat.add("lon", last_nav_pvt_.lon);
  stat.add("lat", last_nav_pvt_.lat);
  stat.add("height", last_nav_pvt_.height);
  stat.add("hMSL", last_nav_pvt_.hMSL);
  stat.add("hAcc", last_nav_pvt_.hAcc);
  stat.add("vAcc", last_nav_pvt_.vAcc);
  stat.add("numSV", last_nav_pvt_.numSV);
}

void UbloxNode7Plus::publishNavPVT(const ublox_msgs::NavPVT& m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavPVT>("navpvt", kROSQueueSize);
  publisher.publish(m);

  /** Fix message */
  static ros::Publisher fixPublisher =
      nh->advertise<sensor_msgs::NavSatFix>("fix", kROSQueueSize);
  // timestamp
  sensor_msgs::NavSatFix fix;
  if(m.nano>0){
      fix.header.stamp.sec = toUtcSeconds(m);
      fix.header.stamp.nsec = m.nano;
  }else {
      fix.header.stamp.sec = toUtcSeconds(m)-1;
      fix.header.stamp.nsec = 1000000000+m.nano;
  }


  bool fixOk = m.flags & m.FLAGS_GNSS_FIX_OK;
  uint8_t cpSoln = m.flags & m.CARRIER_PHASE_FIXED;

  fix.header.frame_id = frame_id_;
  fix.latitude = m.lat * 1e-7; // to deg
  fix.longitude = m.lon * 1e-7; // to deg
  fix.altitude = m.height * 1e-3; // to [m]
  if (fixOk && m.fixType >= m.FIX_TYPE_2D) {
      fix.status.status = fix.status.STATUS_FIX;
      if(cpSoln == m.CARRIER_PHASE_FIXED)
        fix.status.status = fix.status.STATUS_GBAS_FIX;
  }
  else {
      fix.status.status = fix.status.STATUS_NO_FIX;
  }

  const double stdH = (m.hAcc / 1000.0); // to [m]
  const double stdV = (m.vAcc / 1000.0); // to [m]
  fix.position_covariance[0] = stdH * stdH;
  fix.position_covariance[4] = stdH * stdH;
  fix.position_covariance[8] = stdV * stdV;
  fix.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix.status.service = fix_status_service_;
  fixPublisher.publish(fix);

  /** Fix Velocity */
  static ros::Publisher velocityPublisher =
      nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity",
                                                               kROSQueueSize);
  geometry_msgs::TwistWithCovarianceStamped velocity;
  velocity.header.stamp = fix.header.stamp;
  velocity.header.frame_id = frame_id_;

  // convert to XYZ linear velocity in ENU
  velocity.twist.twist.linear.x = m.velE * 1e-3;
  velocity.twist.twist.linear.y = m.velN * 1e-3;
  velocity.twist.twist.linear.z = -m.velD * 1e-3;

  const double covSpeed = pow(m.sAcc * 1e-3, 2);

  const int cols = 6;
  velocity.twist.covariance[cols * 0 + 0] = covSpeed;
  velocity.twist.covariance[cols * 1 + 1] = covSpeed;
  velocity.twist.covariance[cols * 2 + 2] = covSpeed;
  velocity.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

  velocityPublisher.publish(velocity);

  /** Update diagnostics **/
  last_nav_pvt_ = m;
  freq_diag->tick(fix.header.stamp);
  updater->update();
}

/** Ublox Firmware Version 7 **/
UbloxNode7::UbloxNode7(boost::shared_ptr<ros::NodeHandle> _nh) {
  setNh(_nh);
  initialize();
}

void UbloxNode7::configureGnss() {
  ublox_msgs::CfgGNSS cfgGNSSRead;
  if (gps.poll(cfgGNSSRead)) {
    ROS_INFO("Read GNSS config.");
    ROS_INFO("Num. tracking channels in hardware: %i", cfgGNSSRead.numTrkChHw);
    ROS_INFO("Num. tracking channels to use: %i", cfgGNSSRead.numTrkChUse);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  ublox_msgs::CfgGNSS cfgGNSSWrite;
  cfgGNSSWrite.numConfigBlocks = 1;  // do services one by one
  cfgGNSSWrite.numTrkChHw = cfgGNSSRead.numTrkChHw;
  cfgGNSSWrite.numTrkChUse = cfgGNSSRead.numTrkChUse;
  cfgGNSSWrite.msgVer = 0;
  // configure glonass
  ublox_msgs::CfgGNSS_Block block;
  block.gnssId = block.GNSS_ID_GLONASS;
  block.resTrkCh = block.RES_TRK_CH_GLONASS;
  block.maxTrkCh = block.MAX_TRK_CH_GLONASS;
  block.flags = enable_glonass_ | block.SIG_CFG_GLONASS_L1OF;
  cfgGNSSWrite.blocks.push_back(block);
  if (!gps.configure(cfgGNSSWrite)) {
    throw std::runtime_error(std::string("Failed to ") +
                             ((enable_glonass_) ? "enable" : "disable") +
                             " GLONASS.");
  }
  ROS_WARN("ublox_version < 8, ignoring BeiDou Settings");
}

void UbloxNode7::subscribeVersion() {
  ros::NodeHandle param_nh("~");
  // Subscribe to RXM Raw
  param_nh.param("rxm_raw", enabled["rxm_raw"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_raw"])
    gps.subscribe<ublox_msgs::RxmRAW>(boost::bind(
        &UbloxNode::publish<ublox_msgs::RxmRAW>, this, _1, "rxmraw"), 1);

  // Subscribe to RXM SFRB
  param_nh.param("rxm_sfrb", enabled["rxm_sfrb"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_sfrb"])
    gps.subscribe<ublox_msgs::RxmSFRB>(boost::bind(
        &UbloxNode::publish<ublox_msgs::RxmSFRB>, this, _1, "rxmsfrb"), 1);

  // Subscribe to Nav PVT (version 7 & above only)
  param_nh.param("nav_pvt", enabled["nav_pvt"], true);
  if (enabled["nav_pvt"])
    gps.subscribe<ublox_msgs::NavPVT>(boost::bind(
        &UbloxNode7::publishNavPVT, this, _1), 1);
}

/** Ublox Version 8 **/
UbloxNode8::UbloxNode8(boost::shared_ptr<ros::NodeHandle> _nh) {
  setNh(_nh);
  initialize();
}

void UbloxNode8::configureGnss() {
  ublox_msgs::CfgGNSS cfgGNSSRead;
  if (gps.poll(cfgGNSSRead)) {
    ROS_INFO("Read GNSS config.");
    ROS_INFO("Num. tracking channels in hardware: %i", cfgGNSSRead.numTrkChHw);
    ROS_INFO("Num. tracking channels to use: %i", cfgGNSSRead.numTrkChUse);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  ublox_msgs::CfgGNSS cfgGNSSWrite;
  cfgGNSSWrite.numTrkChHw = cfgGNSSRead.numTrkChHw;
  cfgGNSSWrite.numTrkChUse = cfgGNSSRead.numTrkChUse;
  cfgGNSSWrite.msgVer = 0;
  cfgGNSSWrite.numTrkChUse = 28;
  // Configure GPS
  ublox_msgs::CfgGNSS_Block gps_block;
  gps_block.gnssId = gps_block.GNSS_ID_GPS;
  gps_block.resTrkCh = gps_block.RES_TRK_CH_GPS;
  gps_block.maxTrkCh = gps_block.MAX_TRK_CH_GPS;
  gps_block.flags = enable_gps_ | gps_block.SIG_CFG_GPS_L1CA;
  cfgGNSSWrite.blocks.push_back(gps_block);
  // Configure SBAS
  ublox_msgs::CfgGNSS_Block sbas_block;
  sbas_block.gnssId = sbas_block.GNSS_ID_SBAS;
  sbas_block.maxTrkCh = sbas_block.MAX_TRK_CH_MAJOR_MIN;
  sbas_block.flags = enable_sbas_ | sbas_block.SIG_CFG_SBAS_L1CA;
  cfgGNSSWrite.blocks.push_back(sbas_block);
  // Configure Galileo
  ublox_msgs::CfgGNSS_Block galileo_block;
  galileo_block.gnssId = galileo_block.GNSS_ID_GALILEO;
  galileo_block.maxTrkCh = galileo_block.MAX_TRK_CH_MAJOR_MIN;
  galileo_block.flags = enable_galileo_ | galileo_block.SIG_CFG_GALILEO_E1OS;
  cfgGNSSWrite.blocks.push_back(galileo_block);
  // Configure Beidou
  ublox_msgs::CfgGNSS_Block beidou_block;
  beidou_block.gnssId = beidou_block.GNSS_ID_BEIDOU;
  beidou_block.maxTrkCh = beidou_block.MAX_TRK_CH_MAJOR_MIN;
  beidou_block.flags = enable_beidou_ | beidou_block.SIG_CFG_BEIDOU_B1I;
  cfgGNSSWrite.blocks.push_back(beidou_block);
  // Configure IMES
  ublox_msgs::CfgGNSS_Block imes_block;
  imes_block.gnssId = imes_block.GNSS_ID_IMES;
  imes_block.maxTrkCh = imes_block.MAX_TRK_CH_MAJOR_MIN;
  imes_block.flags = enable_imes_ | imes_block.SIG_CFG_IMES_L1;
  cfgGNSSWrite.blocks.push_back(imes_block);
  // Configure QZSS
  ublox_msgs::CfgGNSS_Block qzss_block;
  qzss_block.gnssId = qzss_block.GNSS_ID_QZSS;
  qzss_block.resTrkCh = qzss_block.RES_TRK_CH_QZSS;
  qzss_block.maxTrkCh = qzss_block.MAX_TRK_CH_QZSS;
  qzss_block.flags = enable_qzss_ | qzss_sig_cfg_;
  cfgGNSSWrite.blocks.push_back(qzss_block);
  // Configure GLONASS
  ublox_msgs::CfgGNSS_Block glonass_block;
  glonass_block.gnssId = glonass_block.GNSS_ID_GLONASS;
  glonass_block.resTrkCh = glonass_block.RES_TRK_CH_GLONASS;
  glonass_block.maxTrkCh = glonass_block.MAX_TRK_CH_GLONASS;
  glonass_block.flags = enable_glonass_ | glonass_block.SIG_CFG_GLONASS_L1OF;
  cfgGNSSWrite.blocks.push_back(glonass_block);
  cfgGNSSWrite.numConfigBlocks = cfgGNSSWrite.blocks.size();
  if (!gps.configure(cfgGNSSWrite)) {
    throw std::runtime_error(std::string("Failed to Configure GNSS"));
  }
}
void UbloxNode8::subscribeVersion() {
  ros::NodeHandle param_nh("~");

  // Subscribe to RawX messages
  param_nh.param("rxm_raw", enabled["rxm_raw"],
               enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_raw"])
    gps.subscribe<ublox_msgs::RxmRAWX>(boost::bind(
        &UbloxNode::publish<ublox_msgs::RxmRAWX>, this, _1, "rxmraw"), 1);

  // Subscribe to SFRB messages
  param_nh.param("rxm_sfrb", enabled["rxm_sfrb"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_sfrb"])
    gps.subscribe<ublox_msgs::RxmSFRBX>(boost::bind(
        &UbloxNode::publish<ublox_msgs::RxmSFRBX>, this, _1, "rxmsfrb"), 1);

  // Subscribe to Nav PVT (version 7 & above only)
  param_nh.param("nav_pvt", enabled["nav_pvt"], true);
  if (enabled["nav_pvt"])
    gps.subscribe<ublox_msgs::NavPVT>(boost::bind(
        &UbloxNode7::publishNavPVT, this, _1), 1);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ublox_gps");
  boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle("~"));

  // Create Ublox Node based on firmware version
  ros::NodeHandle param_nh("~");
  int ublox_version;
  param_nh.param("ublox_version", ublox_version, 6);
  ROS_INFO("U-Blox Firmware Version: %d", ublox_version);
  if(ublox_version <= 6) {
    UbloxNode6 node(nh);
  } else if(ublox_version == 7){
    UbloxNode7 node(nh);
  } else {
    ROS_INFO("U-Blox Version: %d", ublox_version);
    UbloxNode8 node(nh);
  }

  return 0;
}
