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
#include <boost/asio/serial_port.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <boost/regex.hpp>

#include <ros/ros.h>
#include <ros/serialization.h>
#include <ublox_msgs/ublox_msgs.h>
#include <ublox_msgs/NavSTATUS.h>
#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavSOL.h>
#include <ublox_msgs/CfgGNSS.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

const static uint32_t kROSQueueSize = 1;

using namespace ublox_gps;

boost::shared_ptr<ros::NodeHandle> nh;
boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> freq_diag;
Gps gps;
ublox_msgs::NavSTATUS status;
std::map<std::string,bool> enabled;
std::string frame_id;
int num_svs_used=0;

ublox_msgs::NavPOSLLH last_nav_pos;
ublox_msgs::NavVELNED last_nav_vel;

sensor_msgs::NavSatFix fix;
geometry_msgs::TwistWithCovarianceStamped velocity;

void publishNavStatus(const ublox_msgs::NavSTATUS& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavSTATUS>("navstatus", kROSQueueSize);
  publisher.publish(m);

  status = m;
}

void publishNavSOL(const ublox_msgs::NavSOL& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavSOL>("navsol", kROSQueueSize);
  num_svs_used = m.numSV; //  number of satellites used
  publisher.publish(m);
}

void publishNavVelNED(const ublox_msgs::NavVELNED& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavVELNED>("navvelned", kROSQueueSize);
  publisher.publish(m);

  // Example geometry message
  static ros::Publisher velocityPublisher = 
      nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity", kROSQueueSize);
  if (m.iTOW == last_nav_pos.iTOW) {
    //  use same time as las navposllh message
    velocity.header.stamp = fix.header.stamp;
  } else {
    //  create a new timestamp
    velocity.header.stamp = ros::Time::now();
  }
  velocity.header.frame_id = frame_id;
  
  //  convert to XYZ linear velocity
  velocity.twist.twist.linear.x = m.velE/100.0;
  velocity.twist.twist.linear.y = m.velN/100.0;
  velocity.twist.twist.linear.z = -m.velD/100.0;
  
  const double stdSpeed = (m.sAcc/100.0) * 3;
  
  const int cols = 6;
  velocity.twist.covariance[cols*0 + 0] = stdSpeed*stdSpeed;
  velocity.twist.covariance[cols*1 + 1] = stdSpeed*stdSpeed;
  velocity.twist.covariance[cols*2 + 2] = stdSpeed*stdSpeed;
  velocity.twist.covariance[cols*3 + 3] = -1; //  angular rate unsupported
  
  velocityPublisher.publish(velocity);
  last_nav_vel = m;
}

void publishNavPosLLH(const ublox_msgs::NavPOSLLH& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavPOSLLH>("navposllh", kROSQueueSize);
  publisher.publish(m);

  // Position message
  static ros::Publisher fixPublisher = nh->advertise<sensor_msgs::NavSatFix>("fix", kROSQueueSize);
  if (m.iTOW == last_nav_vel.iTOW) {
    //  use last timestamp
    fix.header.stamp = velocity.header.stamp;
  } else {
    //  new timestamp
    fix.header.stamp = ros::Time::now();
  }
  fix.header.frame_id = frame_id;
  fix.latitude  = m.lat*1e-7;
  fix.longitude = m.lon*1e-7;
  fix.altitude  = m.height*1e-3;
  if (status.gpsFix >= status.GPS_2D_FIX)
      fix.status.status = fix.status.STATUS_FIX;
  else
      fix.status.status = fix.status.STATUS_NO_FIX;

  //  calculate covariance (convert from mm to m too)
  const double stdH = (m.hAcc / 1000.0) * 3.0;
  const double stdV = (m.vAcc / 1000.0) * 3.0;
  
  fix.position_covariance[0] = stdH*stdH;
  fix.position_covariance[4] = stdH*stdH;
  fix.position_covariance[8] = stdV*stdV;
  fix.position_covariance_type = 
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  
  fix.status.service = fix.status.SERVICE_GPS;
  fixPublisher.publish(fix);
  last_nav_pos = m;
  //  update diagnostics
  freq_diag->tick(fix.header.stamp);
  updater->update();
}

void publishNavSVINFO(const ublox_msgs::NavSVINFO& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavSVINFO>("navsvinfo", kROSQueueSize);
  publisher.publish(m);
}

void publishNavCLK(const ublox_msgs::NavCLOCK& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavCLOCK>("navclock", kROSQueueSize);
  publisher.publish(m);
}

void publishRxmRAW(const ublox_msgs::RxmRAW& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::RxmRAW>("rxmraw", kROSQueueSize);
  publisher.publish(m);
}

void publishRxmSFRB(const ublox_msgs::RxmSFRB& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::RxmSFRB>("rxmsfrb", kROSQueueSize);
  publisher.publish(m);
}

void publishRxmALM(const ublox_msgs::RxmALM& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::RxmALM>("rxmalm", kROSQueueSize);
  publisher.publish(m);
}

void publishRxmEPH(const ublox_msgs::RxmEPH& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::RxmEPH>("rxmeph", kROSQueueSize);
  publisher.publish(m);
}

void publishAidALM(const ublox_msgs::AidALM& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::AidALM>("aidalm", kROSQueueSize);
  publisher.publish(m);
}

void publishAidEPH(const ublox_msgs::AidEPH& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::AidEPH>("aideph", kROSQueueSize);
  publisher.publish(m);
}

void publishAidHUI(const ublox_msgs::AidHUI& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::AidHUI>("aidhui", kROSQueueSize);
  publisher.publish(m);
}

template <typename MessageT>
void publish(const MessageT& m, const std::string& topic) {
  static ros::Publisher publisher = nh->advertise<MessageT>(topic, kROSQueueSize);
  publisher.publish(m);
}

void pollMessages(const ros::TimerEvent& event)
{
  static std::vector<uint8_t> payload(1,1);
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

void fix_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  //  check the last message, convert to diagnostic
  if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "No fix";
  }
  else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  }
  else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "2D fix";
  }
  else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "3D fix";
  }
  else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  }
  else if (status.gpsFix == ublox_msgs::NavSTATUS::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Time fix only";
  }
 
  //  append last fix position
  stat.add("iTOW", last_nav_pos.iTOW);
  stat.add("lon", last_nav_pos.lon);
  stat.add("lat", last_nav_pos.lat);
  stat.add("height", last_nav_pos.height);
  stat.add("hMSL", last_nav_pos.hMSL);
  stat.add("hAcc", last_nav_pos.hAcc);
  stat.add("vAcc", last_nav_pos.vAcc);
  stat.add("numSV", num_svs_used);
}

int main(int argc, char **argv) {
  boost::asio::io_service io_service;
  ros::Timer poller;
  boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_handle;
  boost::shared_ptr<boost::asio::serial_port> serial_handle;
  bool setup_ok = true;
  
  ros::init(argc, argv, "ublox_gps");
  nh.reset(new ros::NodeHandle("~"));
  if (!nh->hasParam("diagnostic_period")) {
    nh->setParam("diagnostic_period", 0.2); //  5Hz diagnostic period 
  }
  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("ublox");
  
  std::string device;
  int baudrate;
  int rate, meas_rate;
  bool enable_sbas, enable_glonass, enable_beidou, enable_ppp;
  std::string dynamic_model, fix_mode;
  int dr_limit;
  int ublox_version;
  ros::NodeHandle param("~");
  param.param("device", device, std::string("/dev/ttyUSB0"));
  param.param("frame_id", frame_id, std::string("gps"));
  param.param("baudrate", baudrate, 9600);
  param.param("rate", rate, 4); //  in Hz
  param.param("enable_sbas", enable_sbas, false);
  param.param("enable_glonass", enable_glonass, false);
  param.param("enable_beidou", enable_beidou, false);
  param.param("enable_ppp", enable_ppp, false);
  param.param("dynamic_model", dynamic_model, std::string("portable"));
  param.param("fix_mode", fix_mode, std::string("both"));
  param.param("dr_limit", dr_limit, 0);
  param.param("ublox_version", ublox_version, 6); /// @todo: temporary workaround
    
  if (enable_ppp) {
    ROS_WARN("Warning: PPP is enabled - this is an expert setting.");
  }
  
  if (rate <= 0) {
    ROS_ERROR("Invalid settings: rate must be > 0");
    return 1;
  }
  //  measurement rate param for ublox, units of ms
  meas_rate = 1000 / rate;
  
  if (dr_limit < 0 || dr_limit > 255) {
    ROS_ERROR("Invalid settings: dr_limit must be between 0 and 255");
    return 1;
  }
  
  DynamicModel dmodel;
  FixMode fmode;
  try {
    dmodel = ublox_gps::modelFromString(dynamic_model);
    fmode = ublox_gps::fixModeFromString(fix_mode);
  } catch (std::exception& e) {
    ROS_ERROR("Invalid settings: %s", e.what());
    return 1;
  }
  
  //  configure diagnostic updater for frequency
  updater->add("fix", &fix_diagnostic);
  updater->force_update();
  
  const double target_freq = 1000.0 / meas_rate;  //  actual update frequency
  double min_freq = target_freq;
  double max_freq = target_freq;
  diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq, 0.05, 10);
  diagnostic_updater::TimeStampStatusParam time_param(0,meas_rate * 1e-3 * 0.05);
  freq_diag.reset(new diagnostic_updater::TopicDiagnostic(std::string("fix"), 
                                                          *updater, 
                                                          freq_param,
                                                          time_param));
  
  boost::smatch match;
  if (boost::regex_match(device, match, boost::regex("(tcp|udp)://(.+):(\\d+)"))) {
    std::string proto(match[1]);
    std::string host(match[2]);
    std::string port(match[3]);
    ROS_INFO("Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(), port.c_str());

    if (proto == "tcp") {
      boost::asio::ip::tcp::resolver::iterator endpoint;

      try {
        boost::asio::ip::tcp::resolver resolver(io_service);
        endpoint = resolver.resolve(boost::asio::ip::tcp::resolver::query(host, port));
      } catch(std::runtime_error& e) {
        ROS_ERROR("Could not resolve %s:%s: %s", host.c_str(), port.c_str(), e.what());
        return 1; //  exit
      }

      boost::asio::ip::tcp::socket *socket = new boost::asio::ip::tcp::socket(io_service);
      tcp_handle.reset(socket);

      try {
        socket->connect(*endpoint);
      } catch(std::runtime_error& e) {
        ROS_ERROR("Could not connect to %s:%s: %s", endpoint->host_name().c_str(), endpoint->service_name().c_str(), e.what());
        return 1; //  exit
      }
    
      ROS_INFO("Connected to %s:%s.", endpoint->host_name().c_str(), endpoint->service_name().c_str());
      gps.initialize(*socket, io_service);
    } else {
      ROS_ERROR("Protocol '%s' is unsupported", proto.c_str());
      return 1; //  exit
    }
  } else {
    boost::asio::serial_port *serial = new boost::asio::serial_port(io_service);
    serial_handle.reset(serial);

    // open serial port
    try {
      serial->open(device);
    } catch (std::runtime_error& e) {
      ROS_ERROR("Could not open serial port %s: %s", device.c_str(), e.what());
      return 1; //  exit
    }

    ROS_INFO("Opened serial port %s", device.c_str());
    gps.setBaudrate(baudrate);
    gps.initialize(*serial, io_service);
  }

  //  apply all requested settings
  try {
    if (!gps.isInitialized()) {
      throw std::runtime_error("Failed to initialize.");
    }
    if (!gps.setMeasRate(meas_rate)) {
      std::stringstream ss;
      ss << "Failed to set measurement rate to " << meas_rate << "ms.";
      throw std::runtime_error(ss.str());
    }
    if (!gps.enableSBAS(enable_sbas)) {
      throw std::runtime_error(std::string("Failed to ") + 
                               ((enable_sbas) ? "enable" : "disable") + " SBAS.");
    }
    if (!gps.setPPPEnabled(enable_ppp)) {
      throw std::runtime_error(std::string("Failed to ") +
                               ((enable_ppp) ? "enable" : "disable") + " PPP.");
    }
    if (!gps.setDynamicModel(dmodel)) {
      throw std::runtime_error("Failed to set model: " + 
                               dynamic_model + ".");
    }
    if (!gps.setFixMode(fmode)) {
      throw std::runtime_error("Failed to set fix mode: " + fix_mode + ".");
    }
    if (!gps.setDeadReckonLimit(dr_limit)) {
      std::stringstream ss;
      ss << "Failed to set dead reckoning limit: " << dr_limit << ".";
      throw std::runtime_error(ss.str());
    }
    
    if (ublox_version >= 7) {
      ublox_msgs::CfgGNSS cfgGNSS;
      if (gps.poll(cfgGNSS)) {
        ROS_INFO("Read GNSS config.");
        ROS_INFO("Num. tracking channels in hardware: %i",cfgGNSS.numTrkChHw);
        ROS_INFO("Num. tracking channels to use: %i",cfgGNSS.numTrkChUse);
      } else {
        throw std::runtime_error("Failed to read the GNSS config.");
      }
      
      cfgGNSS.numConfigBlocks = 1;  //  do services one by one
      cfgGNSS.msgVer = 0;
      cfgGNSS.resTrkCh = 8;   //  taken as defaults from ublox manual
      cfgGNSS.maxTrkCh = 16;
      
      //  configure glonass
      cfgGNSS.gnssId = ublox_msgs::CfgGNSS::GNSS_ID_GLONASS;
      cfgGNSS.flags = enable_glonass;
      if (!gps.configure(cfgGNSS)) {
        throw std::runtime_error(std::string("Failed to ") +
                                 ((enable_glonass) ? "enable" : "disable") + " GLONASS.");
      }
      if (ublox_version >= 8) {
        //  configure beidou
        cfgGNSS.gnssId = ublox_msgs::CfgGNSS::GNSS_ID_BEIDOU;
        cfgGNSS.flags = enable_beidou;
        if (!gps.configure(cfgGNSS)) {
          throw std::runtime_error(std::string("Failed to ") +
                                   ((enable_beidou) ? "enable" : "disable") + " BeiDou.");
        }
      } else {
        ROS_WARN("ublox_version < 8, ignoring BeiDou Settings");
      }
    } else {
      ROS_WARN("ublox_version < 7, ignoring GNSS settings");
    }
  } catch (std::exception& e) {
    setup_ok = false;
    ROS_ERROR("Error configuring device: %s", e.what());
  }

  if (setup_ok) {
    ROS_INFO("U-Blox configured successfully.");
    
    // subscribe messages
    param.param("all", enabled["all"], false);
    param.param("rxm", enabled["rxm"], false);
    param.param("aid", enabled["aid"], false);
    
    param.param("nav_sol", enabled["nav_sol"], true);
    if (enabled["nav_sol"]) gps.subscribe<ublox_msgs::NavSOL>(boost::bind(&publish<ublox_msgs::NavSOL>, _1, "navsol"), 1);
    param.param("nav_status", enabled["nav_status"], true);
    if (enabled["nav_status"]) gps.subscribe<ublox_msgs::NavSTATUS>(&publishNavStatus, 1);
    param.param("nav_svinfo", enabled["nav_svinfo"], enabled["all"]);
    if (enabled["nav_svinfo"]) gps.subscribe<ublox_msgs::NavSVINFO>(&publishNavSVINFO, 20);
    param.param("nav_clk", enabled["nav_clk"], enabled["all"]);
    if (enabled["nav_clk"]) gps.subscribe<ublox_msgs::NavCLOCK>(&publishNavCLK, 1);
    param.param("rxm_raw", enabled["rxm_raw"], enabled["all"] || enabled["rxm"]);
    if (enabled["rxm_raw"]) gps.subscribe<ublox_msgs::RxmRAW>(&publishRxmRAW, 1);
    param.param("rxm_sfrb", enabled["rxm_sfrb"], enabled["all"] || enabled["rxm"]);
    if (enabled["rxm_sfrb"]) gps.subscribe<ublox_msgs::RxmSFRB>(&publishRxmSFRB, 1);
    param.param("nav_posllh", enabled["nav_posllh"], true);
    if (enabled["nav_posllh"]) gps.subscribe<ublox_msgs::NavPOSLLH>(&publishNavPosLLH, 1);
    param.param("nav_velned", enabled["nav_velned"], true);
    if (enabled["nav_velned"]) gps.subscribe<ublox_msgs::NavVELNED>(&publishNavVelNED, 1);
    param.param("aid_alm", enabled["aid_alm"], enabled["all"] || enabled["aid"]);
    if (enabled["aid_alm"]) gps.subscribe<ublox_msgs::AidALM>(&publishAidALM);
    param.param("aid_eph", enabled["aid_eph"], enabled["all"] || enabled["aid"]);
    if (enabled["aid_eph"]) gps.subscribe<ublox_msgs::AidEPH>(&publishAidEPH);
    param.param("aid_hui", enabled["aid_hui"], enabled["all"] || enabled["aid"]);
    if (enabled["aid_hui"]) gps.subscribe<ublox_msgs::AidHUI>(&publishAidHUI);
    
    poller = nh->createTimer(ros::Duration(1.0), &pollMessages);
    poller.start();
    ros::spin();
  }

  if (gps.isInitialized()) {
    gps.close();
    ROS_INFO("Closed connection to %s.", device.c_str());
  }
  return 0;
}
