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

#include "ublox_gps/node.h"

using namespace ublox_gps;
using namespace ublox_node;

//
// U-Blox Node
//
UbloxNode::UbloxNode() {
  // Create Ublox Node based on firmware version
  int ublox_version;
  nh->param("ublox_version", ublox_version, 6);
  ROS_INFO("U-Blox Firmware Version: %d", ublox_version);
  boost::shared_ptr<UbloxInterface> firmware;
  if(ublox_version <= 6) {
    firmware.reset(new UbloxFirmware6);
  } else if(ublox_version == 7){
    firmware.reset(new UbloxFirmware7);
  } else {
    firmware.reset(new UbloxFirmware8);
  }
  xware_.push_back(firmware);
  // Initialize the node
  initialize();
}

void UbloxNode::setHardware(std::string product_category, std::string ref_rov) {
  if(product_category.compare("HPG") == 0 && ref_rov.compare("REF") == 0) {
    xware_.push_back(boost::shared_ptr<UbloxInterface>(new UbloxHpgRef));
  } else if(product_category.compare("HPG") == 0 
            && ref_rov.compare("ROV") == 0) {
    xware_.push_back(boost::shared_ptr<UbloxInterface>(new UbloxHpgRov));
  } else if(product_category.compare("TIM") == 0) {
    xware_.push_back(boost::shared_ptr<UbloxInterface>(new UbloxTim));
  }
}

void UbloxNode::getRosParams() {
  int uart_in_default = ublox_msgs::CfgPRT::PROTO_UBX 
                        | ublox_msgs::CfgPRT::PROTO_NMEA 
                        | ublox_msgs::CfgPRT::PROTO_RTCM;
  int uart_out_default = ublox_msgs::CfgPRT::PROTO_UBX;
  nh->param("device", device_, std::string("/dev/ttyACM0"));
  nh->param("frame_id", frame_id, std::string("gps"));
  // UART 1 params
  nh->param("baudrate", baudrate_, 9600);
  nh->param("uart_in", uart_in_, uart_in_default);
  nh->param("uart_out", uart_out_, uart_out_default);
  // Measurement rate params
  nh->param("rate", rate_, 4);  // in Hz
  nh->param("nav_rate", nav_rate, 1);  // # of measurement rate cycles
  // RTCM params
  nh->param("rtcm_ids", rtcm_ids, rtcm_ids);  // RTCM IDs to configure
  nh->param("rtcm_rate", rtcm_rate, 1);  // in Hz, same for all RTCM IDs
  // PPP: Advanced Setting
  nh->param("enable_ppp", enable_ppp_, false);
  // SBAS params, only for some devices
  nh->param("enable_sbas", enable_sbas_, false);
  nh->param("max_sbas", max_sbas_, 0); // Maximum number of SBAS channels
  nh->param("sbas_usage", sbas_usage_, 0);
  nh->param("dynamic_model", dynamic_model_, std::string("portable"));
  nh->param("fix_mode", fix_mode_, std::string("auto"));
  nh->param("dr_limit", dr_limit_, 0); // Dead reckoning limit
  
  if (enable_ppp_) {
    ROS_WARN("Warning: PPP is enabled - this is an expert setting.");
  }

  if (rate_ <= 0) {
    throw std::runtime_error("Invalid settings: rate must be > 0");
  }

  if (dr_limit_ < 0 || dr_limit_ > 255) {
    throw std::runtime_error(std::string("Invalid settings: dr_limit must ") +
                                         "be between 0 and 255");
  }

  try {
    dmodel_ = ublox_gps::modelFromString(dynamic_model_);
    fmode_ = ublox_gps::fixModeFromString(fix_mode_);
  } catch (std::exception& e) {
    throw std::runtime_error("Invalid dynamic model or fix mode settings");
  }
  
  //  measurement rate param for ublox, units of ms
  meas_rate = 1000 / rate_;
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

void UbloxNode::printInf(const ublox_msgs::Inf &m, uint8_t id) {
  if(id == ublox_msgs::Message::INF::ERROR) {
    ROS_ERROR("%s", m.str.data());
  } else if(id == ublox_msgs::Message::INF::WARNING) {
    ROS_WARN("%s", m.str.data());
  } else {
    ROS_INFO("%s", m.str.data());
  }
}

void UbloxNode::subscribe() {
  ROS_INFO("Subscribing to U-Blox messages");
  // subscribe messages
  nh->param("all", enabled["all"], false);
  nh->param("inf", enabled["inf"], true);
  nh->param("rxm", enabled["rxm"], enabled["all"]);
  nh->param("aid", enabled["aid"], enabled["all"]);
  nh->param("mon", enabled["mon"], enabled["all"]);
  
  // Nav Messages
  nh->param("nav_status", enabled["nav_status"], true);
  if (enabled["nav_status"])
    gps.subscribe<ublox_msgs::NavSTATUS>(boost::bind(
        publish<ublox_msgs::NavSTATUS>, _1, "navstatus"), kSubscribeRate);
  
  nh->param("nav_posecef", enabled["nav_posecef"], false);
  if (enabled["nav_posecef"])
    gps.subscribe<ublox_msgs::NavPOSECEF>(boost::bind(
        publish<ublox_msgs::NavPOSECEF>, _1, "navposecef"), kSubscribeRate);

  nh->param("nav_clock", enabled["nav_clock"], enabled["all"]);
  if (enabled["nav_clock"])
    gps.subscribe<ublox_msgs::NavCLOCK>(boost::bind(
        publish<ublox_msgs::NavCLOCK>, _1, "navclock"), kSubscribeRate);  
  
  // INF messages
  nh->param("inf_debug", enabled["inf_debug"], false);
  if (enabled["inf_debug"])
    gps.subscribeId<ublox_msgs::Inf>(
        boost::bind(&UbloxNode::printInf, this, _1, 
                    ublox_msgs::Message::INF::DEBUG), 
        ublox_msgs::Message::INF::DEBUG);

  nh->param("inf_error", enabled["inf_error"], enabled["inf"]);
  if (enabled["inf_error"])
    gps.subscribeId<ublox_msgs::Inf>(
        boost::bind(&UbloxNode::printInf, this, _1, 
                    ublox_msgs::Message::INF::ERROR), 
        ublox_msgs::Message::INF::ERROR);

  nh->param("inf_notice", enabled["inf_notice"], enabled["inf"]);
  if (enabled["inf_notice"])
    gps.subscribeId<ublox_msgs::Inf>(
        boost::bind(&UbloxNode::printInf, this, _1, 
                    ublox_msgs::Message::INF::NOTICE), 
        ublox_msgs::Message::INF::NOTICE);

  nh->param("inf_test", enabled["inf_test"], enabled["inf"]);
  if (enabled["inf_test"])
    gps.subscribeId<ublox_msgs::Inf>(
        boost::bind(&UbloxNode::printInf, this, _1, 
                    ublox_msgs::Message::INF::TEST), 
        ublox_msgs::Message::INF::TEST);

  nh->param("inf_warning", enabled["inf_warning"], enabled["inf"]);
  if (enabled["inf_warning"])
    gps.subscribeId<ublox_msgs::Inf>(
        boost::bind(&UbloxNode::printInf, this, _1, 
                    ublox_msgs::Message::INF::WARNING), 
        ublox_msgs::Message::INF::WARNING);

  // AID messages
  nh->param("aid_alm", enabled["aid_alm"], enabled["aid"]);
  if (enabled["aid_alm"]) 
    gps.subscribe<ublox_msgs::AidALM>(boost::bind(
        publish<ublox_msgs::AidALM>, _1, "aidalm"), kSubscribeRate);
  
  nh->param("aid_eph", enabled["aid_eph"], enabled["aid"]);
  if (enabled["aid_eph"]) 
    gps.subscribe<ublox_msgs::AidEPH>(boost::bind(
        publish<ublox_msgs::AidEPH>, _1, "aideph"), kSubscribeRate);
  
  nh->param("aid_hui", enabled["aid_hui"], enabled["aid"]);
  if (enabled["aid_hui"]) 
    gps.subscribe<ublox_msgs::AidHUI>(boost::bind(
        publish<ublox_msgs::AidHUI>, _1, "aidhui"), kSubscribeRate);

  for(int i = 0; i < xware_.size(); i++) {
    xware_[i]->subscribe();
  }
}

void UbloxNode::initializeRosDiagnostics() {
  if (!nh->hasParam("diagnostic_period"))
    nh->setParam("diagnostic_period", kDiagnosticPeriod);
  
  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("ublox");

  const double target_freq = rate_;  //  actual update frequency
  double min_freq = target_freq;
  double max_freq = target_freq;
  double timeStampStatusMax = meas_rate * 1e-3 * 0.05;
  diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq,
                                                      kTolerance, kWindow);
  diagnostic_updater::TimeStampStatusParam time_param(kTimeStampStatusMin,
                                                      timeStampStatusMax);
  // configure diagnostic updater for frequency
  freq_diag.reset(new diagnostic_updater::TopicDiagnostic(
      std::string("fix"), *updater, freq_param, time_param));

  for(int i = 0; i < xware_.size(); i++) {
    xware_[i]->initializeRosDiagnostics();
  }
}


void UbloxNode::processMonVer() {
  ublox_msgs::MonVER monVer;
  if (!gps.poll(monVer))
    throw std::runtime_error("Failed to poll MonVER & set relevant settings");

  ROS_INFO("%s, HW VER: %s", monVer.swVersion.c_array(), 
               monVer.hwVersion.c_array());
  // Convert extension to vector of strings
  std::vector<std::string> extension;
  extension.reserve(monVer.extension.size());
  for(std::size_t i = 0; i < monVer.extension.size(); ++i) {
    ROS_INFO("%s", monVer.extension[i].field.c_array());
    // Find the end of the string (null character)
    unsigned char* end = std::find(monVer.extension[i].field.begin(), 
          monVer.extension[i].field.end(), '\0');
    extension.push_back(std::string(monVer.extension[i].field.begin(), end));
  }

  // Get the protocol version
  for(std::size_t i = 0; i < extension.size(); ++i) {
    std::size_t found = extension[i].find("PROTVER");
    if (found!=std::string::npos) {
      protocol_version_ = ::atof(
          extension[i].substr(8, extension[i].size()-8).c_str());
      break;
    }
  }

  if(protocol_version_ < 18) {
    // Firmware is for Standard Precision GNSS product 
    setHardware(std::string("SPG"), "");
    // Final line contains supported GNSS delimited by ;
    std::vector<std::string> strs;
    boost::split(strs, extension[extension.size()-1], boost::is_any_of(";"));
    for(size_t i = 0; i < strs.size(); i++) {
      supported.insert(strs[i]);
    }
  } else {
    for(std::size_t i = 0; i < extension.size(); ++i) {
      std::vector<std::string> strs;
      // Up to 2nd to last line
      if(i <= extension.size() - 2) {
        boost::split(strs, extension[i], boost::is_any_of("="));
        if(strs.size() > 1) {
          if (strs[0].compare(std::string("FWVER")) == 0) {
            if(strs[1].length() > 8) {
              setHardware(strs[1].substr(0, 3), strs[1].substr(8, 10));
            } else {
              setHardware(strs[1].substr(0, 3), "");
            }
            continue;
          }
        }
      }
      // Last 1-2 lines contain supported GNSS
      if(i >= extension.size() - 2) {
        boost::split(strs, extension[i], boost::is_any_of(";"));
        for(size_t i = 0; i < strs.size(); i++) {
          supported.insert(strs[i]);
        }
      }
    }
  }
}

bool UbloxNode::configureUblox() {
  try {
    if (!gps.isInitialized()) {
      throw std::runtime_error("Failed to initialize.");
    }
    // if (!gps.configRate(meas_rate, nav_rate)) {
    //   std::stringstream ss;
    //   ss << "Failed to set measurement rate to " << meas_rate 
    //      << "ms and navigation rate to " << nav_rate;
    //   throw std::runtime_error(ss.str());
    // }
    // // If device doesn't have SBAS, will receive NACK (causes exception)
    // if(supportsGnss("SBAS")) {
    //   if (!gps.enableSBAS(enable_sbas_, sbas_usage_, max_sbas_)) {
    //     throw std::runtime_error(std::string("Failed to ") +
    //                              ((enable_sbas_) ? "enable" : "disable") +
    //                              " SBAS.");
    //   }
    // }
    // if (!gps.setPPPEnabled(enable_ppp_)) {
    //   throw std::runtime_error(std::string("Failed to ") +
    //                            ((enable_ppp_) ? "enable" : "disable") 
    //                            + " PPP.");
    // }
    // if (!gps.setDynamicModel(dmodel_)) {
    //   throw std::runtime_error("Failed to set model: " + dynamic_model_ + ".");
    // }
    // if (!gps.setFixMode(fmode_)) {
    //   throw std::runtime_error("Failed to set fix mode: " + fix_mode_ + ".");
    // }
    // if (!gps.setDeadReckonLimit(dr_limit_)) {
    //   std::stringstream ss;
    //   ss << "Failed to set dead reckoning limit: " << dr_limit_ << ".";
    //   throw std::runtime_error(ss.str());
    // }
    for(int i = 0; i < xware_.size(); i++) {
      xware_[i]->configureUblox();
    }
  } catch (std::exception& e) {
    ROS_ERROR("Error configuring device: %s", e.what());
    return false;
  }
  return true;
}

void UbloxNode::initialize() {
  // Params must be set before initializing IO
  getRosParams();
  gps.initializeIo(device_, baudrate_, uart_in_, uart_out_);
  // Must process Mon VER before setting firmware/hardware params
  processMonVer();
  // Must set firmware & hardware params before initializing diagnostics
  for(int i = 0; i < xware_.size(); i++) {
    xware_[i]->getRosParams();
  }
  // Do this last
  initializeRosDiagnostics();

  if (configureUblox()) {
    ROS_INFO("U-Blox configured successfully.");
    // Subscribe to all U-Blox messages
    subscribe();
    
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

//
// U-Blox Firmware (all versions) 
//
void UbloxFirmware::initializeRosDiagnostics() {
  updater->add("fix", this, &UbloxFirmware::fixDiagnostic);
  updater->force_update();
}

//
// U-Blox Firmware Version 6
//
UbloxFirmware6::UbloxFirmware6() {}

void UbloxFirmware6::getRosParams() {
  // Fix Service type, used when publishing fix status messages
  fix_status_service = sensor_msgs::NavSatStatus::SERVICE_GPS;  
}

bool UbloxFirmware6::configureUblox() {  
  ROS_WARN("ublox_version < 7, ignoring GNSS settings");
  return true;
}

void UbloxFirmware6::subscribe() {
  // Subscribe to RXM Raw, raw data product variants only
  nh->param("rxm_raw", enabled["rxm_raw"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_raw"])
    gps.subscribe<ublox_msgs::RxmRAW>(boost::bind(
        publish<ublox_msgs::RxmRAW>, _1, "rxmraw"), kSubscribeRate);

  // Subscribe to RXM SFRB, raw data product variants only
  nh->param("rxm_sfrb", enabled["rxm_sfrb"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_sfrb"])
    gps.subscribe<ublox_msgs::RxmSFRB>(boost::bind(
        publish<ublox_msgs::RxmSFRB>, _1, "rxmsfrb"), kSubscribeRate);

  // Subscribe to RXM EPH, raw data product variants only
  nh->param("rxm_eph", enabled["rxm_eph"], enabled["rxm"]);
  if (enabled["rxm_eph"])
    gps.subscribe<ublox_msgs::RxmEPH>(boost::bind(
        publish<ublox_msgs::RxmEPH>, _1, "rxmeph"), kSubscribeRate);

  // Subscribe to RXM ALM, raw data product variants only
  nh->param("rxm_alm", enabled["rxm_alm"], enabled["rxm"]);
  if (enabled["rxm_alm"])
    gps.subscribe<ublox_msgs::RxmALM>(boost::bind(
        publish<ublox_msgs::RxmALM>, _1, "rxmalm"), kSubscribeRate);

  // Subscribe to Nav POSLLH
  nh->param("nav_posllh", enabled["nav_posllh"], true);
  if (enabled["nav_posllh"])
    gps.subscribe<ublox_msgs::NavPOSLLH>(boost::bind(
        &UbloxFirmware6::publishNavPosLlh, this, _1), kSubscribeRate);

  // Subscribe to Nav SOL
  nh->param("nav_sol", enabled["nav_sol"], true);
  if (enabled["nav_sol"])
    gps.subscribe<ublox_msgs::NavSOL>(boost::bind(
        &UbloxFirmware6::publishNavSol, this, _1), kSubscribeRate);

  // Subscribe to Nav VELNED
  nh->param("nav_velned", enabled["nav_velned"], true);
  if (enabled["nav_velned"])
    gps.subscribe<ublox_msgs::NavVELNED>(boost::bind(
        &UbloxFirmware6::publishNavVelNed, this, _1), kSubscribeRate);
  
  // Subscribe to Nav SVINFO
  nh->param("nav_svinfo", enabled["nav_svinfo"], enabled["all"]);
  if (enabled["nav_svinfo"])
    gps.subscribe<ublox_msgs::NavSVINFO>(boost::bind(
        publish<ublox_msgs::NavSVINFO>, _1, "navsvinfo"), 
        kNavSvInfoSubscribeRate);
}

void UbloxFirmware6::fixDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  // Set the diagnostic level based on the fix status
  if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  } else if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "2D fix";
  } else if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "3D fix";
  } else if (last_nav_sol_.gpsFix ==
             ublox_msgs::NavSOL::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  } else if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Time fix only";
  } 
  // If fix is not ok (within DOP & Accuracy Masks), raise the diagnostic level
  if(!last_nav_sol_.flags & ublox_msgs::NavSOL::FLAGS_GPS_FIX_OK) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
  } 
  // Raise diagnostic level to error if no fix
  if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "No fix";
  }

  //  append last fix position
  stat.add("iTOW", last_nav_pos_.iTOW);
  stat.add("lon", last_nav_pos_.lon);
  stat.add("lat", last_nav_pos_.lat);
  stat.add("height", last_nav_pos_.height);
  stat.add("hMSL", last_nav_pos_.hMSL);
  stat.add("hAcc", last_nav_pos_.hAcc);
  stat.add("vAcc", last_nav_pos_.vAcc);
  stat.add("numSV", last_nav_sol_.numSV);
}

void UbloxFirmware6::publishNavPosLlh(const ublox_msgs::NavPOSLLH& m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavPOSLLH>("navposllh", kROSQueueSize);
  publisher.publish(m);

  // Position message
  static ros::Publisher fixPublisher =
      nh->advertise<sensor_msgs::NavSatFix>("fix", kROSQueueSize);
  if (m.iTOW == last_nav_vel_.iTOW) {
    //  use last timestamp
    fix_.header.stamp = velocity_.header.stamp;
  } else {
    //  new timestamp
    fix_.header.stamp = ros::Time::now();
  }
  fix_.header.frame_id = frame_id;
  fix_.latitude = m.lat * 1e-7;
  fix_.longitude = m.lon * 1e-7;
  fix_.altitude = m.height * 1e-3;
  if (last_nav_sol_.gpsFix >= last_nav_sol_.GPS_2D_FIX)
    fix_.status.status = fix_.status.STATUS_FIX;
  else
    fix_.status.status = fix_.status.STATUS_NO_FIX;

  // Convert from mm to m
  const double varH = pow(m.hAcc / 1000.0, 2);
  const double varV = pow(m.vAcc / 1000.0, 2);

  fix_.position_covariance[0] = varH;
  fix_.position_covariance[4] = varH;
  fix_.position_covariance[8] = varV;
  fix_.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix_.status.service = fix_.status.SERVICE_GPS;
  fixPublisher.publish(fix_);
  last_nav_pos_ = m;
  //  update diagnostics
  freq_diag->tick(fix_.header.stamp);
  updater->update();
}

void UbloxFirmware6::publishNavVelNed(const ublox_msgs::NavVELNED& m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavVELNED>("navvelned", kROSQueueSize);
  publisher.publish(m);

  // Example geometry message
  static ros::Publisher velocityPublisher =
      nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity",
                                                                kROSQueueSize);
  if (m.iTOW == last_nav_pos_.iTOW) {
    //  use same time as las navposllh message
    velocity_.header.stamp = fix_.header.stamp;
  } else {
    //  create a new timestamp
    velocity_.header.stamp = ros::Time::now();
  }
  velocity_.header.frame_id = frame_id;

  //  convert to XYZ linear velocity
  velocity_.twist.twist.linear.x = m.velE / 100.0;
  velocity_.twist.twist.linear.y = m.velN / 100.0;
  velocity_.twist.twist.linear.z = -m.velD / 100.0;

  const double varSpeed = pow(m.sAcc / 100.0, 2);

  const int cols = 6;
  velocity_.twist.covariance[cols * 0 + 0] = varSpeed;
  velocity_.twist.covariance[cols * 1 + 1] = varSpeed;
  velocity_.twist.covariance[cols * 2 + 2] = varSpeed;
  velocity_.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

  velocityPublisher.publish(velocity_);
  last_nav_vel_ = m;
}

void UbloxFirmware6::publishNavSol(const ublox_msgs::NavSOL& m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavSOL>("navsol", kROSQueueSize);
  publisher.publish(m);
  last_nav_sol_ = m;
}

/** Ublox Firmware Version >=7 **/
void UbloxFirmware7Plus::fixDiagnostic(
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
  } else if (last_nav_pvt_.fixType == 
             ublox_msgs::NavSTATUS::GPS_TIME_ONLY_FIX) {
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

void UbloxFirmware7Plus::publishNavPvt(const ublox_msgs::NavPVT& m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavPVT>("navpvt", kROSQueueSize);
  publisher.publish(m);

  /** Fix message */
  static ros::Publisher fixPublisher =
      nh->advertise<sensor_msgs::NavSatFix>("fix", kROSQueueSize);
  // timestamp
  sensor_msgs::NavSatFix fix;
  fix.header.stamp.sec = toUtcSeconds(m);
  fix.header.stamp.nsec = m.nano;

  bool fixOk = m.flags & m.FLAGS_GNSS_FIX_OK;
  uint8_t cpSoln = m.flags & m.CARRIER_PHASE_FIXED;

  fix.header.frame_id = frame_id;
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

  const double varH = pow(m.hAcc / 1000.0, 2); // to [m^2]
  const double varV = pow(m.vAcc / 1000.0, 2); // to [m^2]
  fix.position_covariance[0] = varH;
  fix.position_covariance[4] = varH;
  fix.position_covariance[8] = varV;
  fix.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix.status.service = fix_status_service;
  fixPublisher.publish(fix);

  /** Fix Velocity */
  static ros::Publisher velocityPublisher =
      nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity",
                                                                kROSQueueSize);
  geometry_msgs::TwistWithCovarianceStamped velocity;
  velocity.header.stamp = fix.header.stamp;
  velocity.header.frame_id = frame_id;

  // convert to XYZ linear velocity [m/s] in ENU
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

//
// Ublox Firmware Version 7
//
UbloxFirmware7::UbloxFirmware7() {}

void UbloxFirmware7::getRosParams() {
  int qzss_sig_cfg_default = 
      ublox_msgs::CfgGNSS_Block::SIG_CFG_QZSS_L1CA;
  nh->param("qzss_sig_cfg", qzss_sig_cfg_, qzss_sig_cfg_default);
  // GNSS enable/disable
  nh->param("enable_gps", enable_gps_, true);
  nh->param("enable_glonass", enable_glonass_, false);
  nh->param("enable_qzss", enable_qzss_, false);
  nh->param("enable_sbas", enable_sbas_, false);
  
  if(enable_gps_ && !supportsGnss("GPS")) 
    ROS_WARN("enable_gps is true, but GPS GNSS is not supported by this device");
  if(enable_glonass_ && !supportsGnss("GLO")) 
    ROS_WARN("enable_glonass is true, but GLONASS is not %s", 
             "supported by this device");
  if(enable_qzss_ && !supportsGnss("QZSS")) 
    ROS_WARN("enable_qzss is true, but QZSS is not supported by this device");
  if(enable_sbas_ && !supportsGnss("SBAS")) 
    ROS_WARN("enable_sbas is true, but SBAS is not supported by this device");
  
  bool enable_galileo, enable_beidou, enable_imes;
  nh->param("enable_imes", enable_imes, false);
  if(enable_galileo) 
    ROS_WARN("ublox_version < 8, ignoring Galileo GNSS Settings");
  nh->param("enable_galileo", enable_galileo, false);
  if(enable_beidou) 
    ROS_WARN("ublox_version < 8, ignoring BeiDou Settings");
  nh->param("enable_beidou", enable_beidou, false);
  if(enable_imes) 
    ROS_WARN("ublox_version < 8, ignoring IMES GNSS Settings");

  // Fix Service type, used when publishing fix status messages
  fix_status_service = sensor_msgs::NavSatStatus::SERVICE_GPS 
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GLONASS;
}

bool UbloxFirmware7::configureUblox() {
  /** Configure the GNSS **/
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
  
  // configure GLONASS
  if(supportsGnss("GLO")) {
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
  }
  
  if(supportsGnss("QZSS")) {
    // configure QZSS
    ublox_msgs::CfgGNSS_Block block;
    block.gnssId = block.GNSS_ID_QZSS;
    block.resTrkCh = block.RES_TRK_CH_QZSS;
    block.maxTrkCh = block.MAX_TRK_CH_QZSS;
    block.flags = enable_qzss_ | qzss_sig_cfg_; 
    cfgGNSSWrite.blocks[0] = block;
    if (!gps.configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_glonass_) ? "enable" : "disable") +
                               " QZSS.");
    }
  }

  if(supportsGnss("SBAS")) {
    // configure SBAS
    ublox_msgs::CfgGNSS_Block block;
    block.gnssId = block.GNSS_ID_SBAS;
    block.resTrkCh = block.RES_TRK_CH_SBAS;
    block.maxTrkCh = block.MAX_TRK_CH_SBAS;
    block.flags = enable_sbas_ | block.SIG_CFG_SBAS_L1CA;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps.configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_sbas_) ? "enable" : "disable") +
                               " SBAS.");
    }
  }
  return true;
}

void UbloxFirmware7::subscribe() {
  // Subscribe to Nav PVT
  nh->param("nav_pvt", enabled["nav_pvt"], true);
  if (enabled["nav_pvt"])
    gps.subscribe<ublox_msgs::NavPVT>(boost::bind(
        &UbloxFirmware7Plus::publishNavPvt, this, _1),
        kSubscribeRate);

  // Subscribe to Nav SVINFO
  nh->param("nav_svinfo", enabled["nav_svinfo"], enabled["all"]);
  if (enabled["nav_svinfo"])
    gps.subscribe<ublox_msgs::NavSVINFO>(boost::bind(
        publish<ublox_msgs::NavSVINFO>, _1, "navsvinfo"), 
        kNavSvInfoSubscribeRate);

  // Subscribe to Mon HW
  nh->param("mon_hw", enabled["mon_hw"], enabled["mon"]);
  if (enabled["mon_hw"])
    gps.subscribe<ublox_msgs::MonHW>(boost::bind(
        publish<ublox_msgs::MonHW>, _1, "monhw"), kSubscribeRate);

  // Subscribe to RXM Raw, raw data product variants only
  nh->param("rxm_raw", enabled["rxm_raw"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_raw"])
    gps.subscribe<ublox_msgs::RxmRAW>(boost::bind(
        publish<ublox_msgs::RxmRAW>, _1, "rxmraw"), kSubscribeRate);

  // Subscribe to RXM SFRB, raw data product variants only
  nh->param("rxm_sfrb", enabled["rxm_sfrb"],
                 enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_sfrb"])
    gps.subscribe<ublox_msgs::RxmSFRB>(boost::bind(
        publish<ublox_msgs::RxmSFRB>, _1, "rxmsfrb"), kSubscribeRate);

  // Subscribe to RXM EPH, raw data product variants only
  nh->param("rxm_eph", enabled["rxm_eph"], enabled["rxm"]);
  if (enabled["rxm_eph"])
    gps.subscribe<ublox_msgs::RxmEPH>(boost::bind(
        publish<ublox_msgs::RxmEPH>, _1, "rxmeph"), kSubscribeRate);

  // Subscribe to RXM ALM, raw data product variants only
  nh->param("rxm_alm", enabled["rxm_alm"], enabled["rxm"]);
  if (enabled["rxm_alm"])
    gps.subscribe<ublox_msgs::RxmALM>(boost::bind(
        publish<ublox_msgs::RxmALM>, _1, "rxmalm"), kSubscribeRate);
}

//
// Ublox Version 8 
//
UbloxFirmware8::UbloxFirmware8() {}

void UbloxFirmware8::getRosParams() {
  int qzss_sig_cfg_default = 
      ublox_msgs::CfgGNSS_Block::SIG_CFG_QZSS_L1CA;
  nh->param("qzss_sig_cfg", qzss_sig_cfg_, qzss_sig_cfg_default);
  // GNSS enable/disable
  nh->param("enable_gps", enable_gps_, true);
  nh->param("enable_galileo", enable_galileo_, false);
  nh->param("enable_beidou", enable_beidou_, false);
  nh->param("enable_imes", enable_imes_, false);
  nh->param("enable_glonass", enable_glonass_, false);
  nh->param("enable_qzss", enable_qzss_, false);
  nh->param("enable_sbas", enable_sbas_, false);

  if(enable_gps_ && !supportsGnss("GPS")) 
    ROS_WARN("enable_gps is true, but GPS GNSS is not supported by %s",
             "this device");
  if(enable_glonass_ && !supportsGnss("GLO")) 
    ROS_WARN("enable_glonass is true, but GLONASS is not supported by %s",
             "this device");
  if(enable_galileo_ && !supportsGnss("GAL")) 
    ROS_WARN("enable_galileo is true, but Galileo GNSS is not supported by %s",
             "this device");
  if(enable_beidou_ && !supportsGnss("BDS")) 
    ROS_WARN("enable_beidou is true, but Beidou GNSS is not supported by %s",
             "this device");
  if(enable_imes_ && !supportsGnss("IMES")) 
    ROS_WARN("enable_imes is true, but IMES GNSS is not supported by %s",
             "this device");
  if(enable_qzss_ && !supportsGnss("QZSS")) 
    ROS_WARN("enable_qzss is true, but QZSS is not supported by this device");
  if(enable_sbas_ && !supportsGnss("SBAS")) 
    ROS_WARN("enable_sbas is true, but SBAS is not supported by this device");

  // Fix Service type, used when publishing fix status messages
  fix_status_service = sensor_msgs::NavSatStatus::SERVICE_GPS 
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GLONASS
      + (enable_beidou_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_COMPASS
      + (enable_galileo_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GALILEO;
}

bool UbloxFirmware8::configureUblox() {
  //
  // Configure the GNSS, only if the configuration is different
  //
  // First, get the current GNSS configuration
  ublox_msgs::CfgGNSS cfgGNSSRead;
  if (gps.poll(cfgGNSSRead)) {
    ROS_INFO("Read GNSS config.");
    ROS_INFO("Num. tracking channels in hardware: %i", cfgGNSSRead.numTrkChHw);
    ROS_INFO("Num. tracking channels to use: %i", cfgGNSSRead.numTrkChUse);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  // Then, check if the GNSS configuration is correct
  bool correct = true;
  for(int i = 0; i < cfgGNSSRead.blocks.size(); i++) {
    ublox_msgs::CfgGNSS_Block block = cfgGNSSRead.blocks[i];
    if (block.gnssId == block.GNSS_ID_GPS
        && !enable_gps_ == block.flags & block.FLAGS_ENABLE) {
      correct = false;
      break;
    } else if (block.gnssId == block.GNSS_ID_SBAS
               && !enable_sbas_ == block.flags & block.FLAGS_ENABLE) {
      correct = false;
      break;
    } else if (block.gnssId == block.GNSS_ID_GALILEO
               && !enable_galileo_ == block.flags & block.FLAGS_ENABLE) {
      correct = false;
      break;
    } else if (block.gnssId == block.GNSS_ID_BEIDOU
               && !enable_beidou_ == block.flags & block.FLAGS_ENABLE) {
      correct = false;
      break;
    } else if (block.gnssId == block.GNSS_ID_IMES
               && !enable_imes_ == block.flags & block.FLAGS_ENABLE) {
      correct = false;
      break;
    } else if (block.gnssId == block.GNSS_ID_QZSS
               && !enable_qzss_ == block.flags & block.FLAGS_ENABLE 
               || (enable_qzss_ 
               && qzss_sig_cfg_ != block.flags & block.FLAGS_SIG_CFG_MASK)) {
      correct = false;
      break;
    } else if (block.gnssId == block.GNSS_ID_GLONASS
               && !enable_glonass_ == block.flags & block.FLAGS_ENABLE) {
      correct = false;
      break;
    }
  }

  // If the GNSS is already configured correctly, do not re-configure GNSS
  // since this requires a cold reset
  if(correct) {
    ROS_INFO("U-Blox GNSS settings are already as desired. %s",
             "Device not re-configured.");
    return true;
  }

  // Create configuration message
  ublox_msgs::CfgGNSS cfgGNSSWrite;
  cfgGNSSWrite.msgVer = 0;
  cfgGNSSWrite.numTrkChHw = cfgGNSSRead.numTrkChHw;
  cfgGNSSWrite.numTrkChUse = cfgGNSSRead.numTrkChUse;
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
  sbas_block.resTrkCh = sbas_block.RES_TRK_CH_SBAS;
  sbas_block.maxTrkCh = sbas_block.MAX_TRK_CH_SBAS;
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

  // Configure the GNSS
  if (!gps.configure(cfgGNSSWrite)) {
    throw std::runtime_error(std::string("Failed to Configure GNSS"));
  }
  ROS_WARN("GNSS re-configured, cold reset recommended.");
  return true;
}

void UbloxFirmware8::subscribe() {
  // Subscribe to Nav PVT (version 7 & above only)
  nh->param("nav_pvt", enabled["nav_pvt"], true);
  if (enabled["nav_pvt"])
    gps.subscribe<ublox_msgs::NavPVT>(boost::bind(
        &UbloxFirmware7Plus::publishNavPvt, this, _1), 
        kSubscribeRate);

  // Subscribe to Nav SAT messages
  nh->param("nav_sat", enabled["nav_sat"], enabled["all"]);
  if (enabled["nav_sat"])
    gps.subscribe<ublox_msgs::NavSAT>(boost::bind(
        publish<ublox_msgs::NavSAT>, _1, "navsat"), kNavSvInfoSubscribeRate);

  // Subscribe to Mon HW
  nh->param("mon_hw", enabled["mon_hw"], enabled["mon"]);
  if (enabled["mon_hw"])
    gps.subscribe<ublox_msgs::MonHW>(boost::bind(
        publish<ublox_msgs::MonHW>, _1, "monhw"), kSubscribeRate);

  // Subscribe to RTCM messages
  nh->param("rxm_rtcm", enabled["rxm_rtcm"], 
            enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_rtcm"])
    gps.subscribe<ublox_msgs::RxmRTCM>(boost::bind(
        publish<ublox_msgs::RxmRTCM>, _1, "rxmrtcm"), kSubscribeRate);
}

//
// U-Blox High Precision GNSS Reference Station
//
void UbloxHpgRef::getRosParams() {
  nh->param("tmode3", tmode3_, -1); // default to do not configure

  if(tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_FIXED) {
    if(!nh->getParam("arp_position", arp_position_))
      throw std::runtime_error(std::string("Invalid settings: arp_position ") 
                               + "must be set if TMODE3 is fixed");
    if(!nh->getParam("arp_position_hp", arp_position_hp_))
      throw std::runtime_error(std::string("Invalid settings: arp_position_hp ") 
                               + "must be set if TMODE3 is fixed");
    if(!nh->getParam("fixed_pos_acc", fixed_pos_acc_))
      throw std::runtime_error(std::string("Invalid settings: fixed_pos_acc ") 
                               + "must be set if TMODE3 is fixed");
    if(!nh->getParam("lla_flag", lla_flag_)) {
      ROS_WARN("lla_flag param not set, assuming ARP coordinates are in ECEF");
      lla_flag_ = false;
    }
  } else if(tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
    nh->param("svin_reset", svin_reset_, true);
    if(!nh->getParam("sv_in_min_dur", sv_in_min_dur_))
      throw std::runtime_error(std::string("Invalid settings: sv_in_min_dur ") 
                               + "must be set if TMODE3 is survey-in");
    if(!nh->getParam("sv_in_acc_lim", sv_in_acc_lim_))
      throw std::runtime_error(std::string("Invalid settings: sv_in_acc_lim ") 
                               + "must be set if TMODE3 is survey-in");
  }
}

bool UbloxHpgRef::configureUblox() {
  // Configure TMODE3
  if(tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_DISABLED) {
    if(!gps.disableTmode3())
      throw std::runtime_error("Failed to disable TMODE3.");
    mode_ = DISABLED;
  } else if(tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_FIXED) {
    if(!gps.configTmode3Fixed(lla_flag_, arp_position_, arp_position_hp_, 
                               fixed_pos_acc_))
      throw std::runtime_error("Failed to set TMODE3 to fixed.");
    if(!gps.configRtcm(rtcm_ids, rtcm_rate))
      throw std::runtime_error("Failed to set RTCM rates");
    mode_ = FIXED;
  } else if(tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
    if(!svin_reset_) {
      ublox_msgs::NavSVIN nav_svin;
      if(!gps.poll(nav_svin))
        throw std::runtime_error(std::string("Failed to poll NavSVIN while") +
                                 " configuring survey-in");
      // Don't reset survey-in if it's already active
      if(nav_svin.active) {
        mode_ = SURVEY_IN;
        return true;
      }
      // Don't reset survey-in if it already has a valid value
      if(nav_svin.valid) {
        mode_ = TIME;
        return true;
      }
      ublox_msgs::NavPVT nav_pvt;
      if(!gps.poll(nav_pvt))
        throw std::runtime_error(std::string("Failed to poll NavPVT while") + 
                                 " configuring survey-in");
      // Don't reset survey in if in time mode with a good fix
      if (nav_pvt.fixType == nav_pvt.FIX_TYPE_TIME_ONLY 
          && nav_pvt.flags & nav_pvt.FLAGS_GNSS_FIX_OK) {
        mode_ = TIME;
        return true;
      }
    }
    // Reset the Survey In    
    // For Survey in, meas rate must be at least 1 Hz
    uint16_t meas_rate_temp = std::min(meas_rate, 1000); // [ms]
    // If measurement period isn't a factor of 1000, set to default
    if(1000 % meas_rate_temp != 0)
      meas_rate_temp = kDefaultMeasPeriod;
    // Set nav rate to 1 Hz during survey in
    if(!gps.configRate(meas_rate_temp, (int) 1000 / meas_rate_temp))
      throw std::runtime_error(std::string("Failed to set nav rate to 1 Hz") +
                               "before setting TMODE3 to survey-in.");
    // As recommended in the documentation, first disable, then set to survey in
    if(!gps.disableTmode3())
      ROS_ERROR("Failed to disable TMODE3 before setting to survey-in.");
    else 
      mode_ = DISABLED;
    // Set to Survey in mode
    if(!gps.configTmode3SurveyIn(sv_in_min_dur_, sv_in_acc_lim_))
      throw std::runtime_error("Failed to set TMODE3 to survey-in.");
    mode_ = SURVEY_IN;
  }
  return true;
}

void UbloxHpgRef::subscribe() {
  // Subscribe to Nav Survey In, High Precision GNSS devices only
  nh->param("nav_svin", enabled["nav_svin"], true);
  if (enabled["nav_svin"])
     gps.subscribe<ublox_msgs::NavSVIN>(boost::bind(
        &UbloxHpgRef::publishNavSvIn, this, _1), kSubscribeRate);
}

void UbloxHpgRef::publishNavSvIn(ublox_msgs::NavSVIN m) {
  static ros::Publisher publisher =
      nh->advertise<ublox_msgs::NavSVIN>("navsvin", kROSQueueSize);
  publisher.publish(m);

  last_nav_svin_ = m;

  if(!m.active && m.valid && mode_ == SURVEY_IN) {
    mode_ = TIME; // switch to time mode

    // Set the Measurement & nav rate to user config
    if(!gps.configRate(meas_rate, nav_rate)) {
      ROS_ERROR("Failed to set measurement rate to %d ms %s %d", meas_rate, 
                "navigation rate to ", nav_rate);
    }
    // Enable the RTCM out messages
    if(!gps.configRtcm(rtcm_ids, rtcm_rate)) {
      ROS_ERROR("Failed to configure RTCM IDs");
    }
  }

  updater->update();
}

void UbloxHpgRef::initializeRosDiagnostics() {
  updater->add("HPG REF", this, &UbloxHpgRef::diagnosticUpdater);
  updater->force_update();
}

void UbloxHpgRef::diagnosticUpdater(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (mode_ == INIT) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "TMODE3 Not configured";
  } else if (mode_ == DISABLED){
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "TMODE3 Disabled";
  } else if (mode_ == SURVEY_IN) {
    if (!last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.message = "Survey-In inactive and invalid";
    } else if (last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.message = "Survey-In active but invalid";
    } else if (!last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Survey-In complete";
    } else if (last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Survey-In active and valid";
    }
    
    stat.add("iTOW [ms]", last_nav_svin_.iTOW);
    stat.add("Duration [s]", last_nav_svin_.dur);
    stat.add("# observations", last_nav_svin_.obs);
    stat.add("Mean X [m]", last_nav_svin_.meanX * 1e-2);
    stat.add("Mean Y [m]", last_nav_svin_.meanY * 1e-2);
    stat.add("Mean Z [m]", last_nav_svin_.meanZ * 1e-2);
    stat.add("Mean X HP [m]", last_nav_svin_.meanXHP * 1e-4);
    stat.add("Mean Y HP [m]", last_nav_svin_.meanYHP * 1e-4);
    stat.add("Mean Z HP [m]", last_nav_svin_.meanZHP * 1e-4);
    stat.add("Mean Accuracy [m]", last_nav_svin_.meanAcc * 1e-4);
  } else if(mode_ == FIXED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "TMODE3 Fixed Position";
  } else if(mode_ == TIME) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "TMODE3 Time";
  }
}

//
// U-Blox High Precision GNSS Rover
//
void UbloxHpgRov::getRosParams() {
  // default to float, see CfgDGNSS message for details
  nh->param("dgnss_mode", dgnss_mode_, 2); 
}

bool UbloxHpgRov::configureUblox() {
  // Configure the DGNSS
  if(!gps.configDgnss(dgnss_mode_)) {
    throw std::runtime_error(std::string("Failed to Configure DGNSS"));
  }
  return true;
}

void UbloxHpgRov::subscribe() {
  // Subscribe to Nav Relative Position NED, High Precision Rovers only
  nh->param("nav_relposned", enabled["nav_relposned"], true);
  if (enabled["nav_relposned"])
    gps.subscribe<ublox_msgs::NavRELPOSNED>(boost::bind(
        publish<ublox_msgs::NavRELPOSNED>, _1, "navrelposned"), kSubscribeRate);
}

void UbloxHpgRov::initializeRosDiagnostics() {
  updater->add("HPG ROV", this, &UbloxHpgRov::diagnosticUpdater);
  updater->force_update();
}

void UbloxHpgRov::diagnosticUpdater(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  std::string error_msg = "RTCM Message frequency too low for IDs: ";
  bool error = false;
  for(int i = 0; i < rtcm_ids.size(); i++) {
    ros::Duration dt = ros::Time::now() - last_received_rtcm_[rtcm_ids[i]];
    std::string id = boost::lexical_cast<std::string>(rtcm_ids[i]);
    stat.add("RTCM " + id, dt.toSec());
    if(dt.toSec() > 1.0) {
      error_msg += rtcm_ids[i] + ", ";
      error = true;
    }
  }
  if(error) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = error_msg.c_str();
  } else {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "RTCM Message frequencies ok";
  } 
}

//
// U-Blox Time Sync Products
//
void UbloxTim::subscribe() {
  // Subscribe to RawX messages
  nh->param("rxm_raw", enabled["rxm_raw"],
            enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_raw"])
    gps.subscribe<ublox_msgs::RxmRAWX>(boost::bind(
        publish<ublox_msgs::RxmRAWX>, _1, "rxmraw"), kSubscribeRate);

  // Subscribe to SFRBX messages
  nh->param("rxm_sfrb", enabled["rxm_sfrb"], 
            enabled["all"] || enabled["rxm"]);
  if (enabled["rxm_sfrb"])
    gps.subscribe<ublox_msgs::RxmSFRBX>(boost::bind(
        publish<ublox_msgs::RxmSFRBX>, _1, "rxmsfrb"), kSubscribeRate);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ublox_gps");
  nh.reset(new ros::NodeHandle("~"));
  nh->param("debug", debug, 1);
  if(debug) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                                       ros::console::levels::Debug)) {
     ros::console::notifyLoggerLevelsChanged();
    }
  }
  UbloxNode node;
  return 0;
}
