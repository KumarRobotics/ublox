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
#include <ublox_msgs/ublox_msgs.h>

#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/Vector3Stamped.h>

using namespace ublox_gps;

ros::NodeHandle *nh;
Gps gps;
ublox_msgs::NavSTATUS status;
std::map<std::string,bool> enabled;
std::string frame_id;

sensor_msgs::NavSatFix fix;
geometry_msgs::Vector3Stamped velocity;

void publishNavStatus(const ublox_msgs::NavSTATUS& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavSTATUS>("ublox/navstatus", 10);
  publisher.publish(m);

  status = m;
}

void publishNavSOL(const ublox_msgs::NavSOL& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavSOL>("ublox/navsol", 10);
  publisher.publish(m);
}

void publishNavVelNED(const ublox_msgs::NavVELNED& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavVELNED>("ublox/navvelned", 10);
  publisher.publish(m);

  // Example geometry message
  static ros::Publisher velocityPublisher = nh->advertise<geometry_msgs::Vector3Stamped>("fix_velocity",10);
  velocity.header.stamp = fix.header.stamp.isZero() ? ros::Time::now() : fix.header.stamp;
  velocity.header.frame_id = frame_id;
  velocity.vector.x = m.velN/100.0;
  velocity.vector.y = -m.velE/100.0;
  velocity.vector.z = -m.velD/100.0;
  velocityPublisher.publish(velocity);

}

void publishNavPosLLH(const ublox_msgs::NavPOSLLH& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavPOSLLH>("ublox/navposllh", 10);
  publisher.publish(m);

  // Position message
  static ros::Publisher fixPublisher = nh->advertise<sensor_msgs::NavSatFix>("fix", 10);
  fix.header.stamp = ros::Time::now();
  fix.header.frame_id = frame_id;
  fix.latitude  = m.lat*1e-7;
  fix.longitude = m.lon*1e-7;
  fix.altitude  = m.height*1e-3;
  if (status.gpsFix >= status.GPS_3D_FIX)
      fix.status.status = fix.status.STATUS_FIX;
  else
      fix.status.status = fix.status.STATUS_NO_FIX;

  fix.status.service = fix.status.SERVICE_GPS;
  fixPublisher.publish(fix);

}

void publishNavSVINFO(const ublox_msgs::NavSVINFO& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavSVINFO>("ublox/navsvinfo", 10);
  publisher.publish(m);
}

void publishNavCLK(const ublox_msgs::NavCLOCK& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::NavCLOCK>("ublox/navclock", 10);
  publisher.publish(m);
}

void publishRxmRAW(const ublox_msgs::RxmRAW& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::RxmRAW>("ublox/rxmraw", 10);
  publisher.publish(m);
}

void publishRxmSFRB(const ublox_msgs::RxmSFRB& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::RxmSFRB>("ublox/rxmsfrb", 10);
  publisher.publish(m);
}

void publishRxmALM(const ublox_msgs::RxmALM& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::RxmALM>("ublox/rxmalm", 10);
  publisher.publish(m);
}

void publishRxmEPH(const ublox_msgs::RxmEPH& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::RxmEPH>("ublox/rxmeph", 10);
  publisher.publish(m);
}

void publishAidALM(const ublox_msgs::AidALM& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::AidALM>("ublox/aidalm", 10);
  publisher.publish(m);
}

void publishAidEPH(const ublox_msgs::AidEPH& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::AidEPH>("ublox/aideph", 10);
  publisher.publish(m);
}

void publishAidHUI(const ublox_msgs::AidHUI& m)
{
  static ros::Publisher publisher = nh->advertise<ublox_msgs::AidHUI>("ublox/aidhui", 10);
  publisher.publish(m);
}

template <typename MessageT>
void publish(const MessageT& m, const std::string& topic) {
  static ros::Publisher publisher = nh->advertise<MessageT>(topic, 10);
  publisher.publish(m);
}

void pollMessages(const ros::TimerEvent& event)
{
  static std::vector<uint8_t> payload(1,1);
  if (enabled["aid_alm"]) gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM, payload);
  if (enabled["aid_eph"]) gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH, payload);
  if (enabled["aid_hui"]) gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI);
  payload[0]++;
  if (payload[0] > 32) payload[0] = 1;
}

int main(int argc, char **argv) {
  boost::asio::io_service io_service;
  ros::Timer poller;
  boost::shared_ptr<void> handle;

  ros::init(argc, argv, "ublox_gps");
  nh = new ros::NodeHandle;

  std::string device;
  int baudrate;

  ros::NodeHandle param("~");
  param.param("device", device, std::string("/dev/ttyUSB0"));
  param.param("frame_id", frame_id, std::string("gps_frame"));
  if (param.getParam("baudrate", baudrate)) gps.setBaudrate(baudrate);

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
        if (endpoint == boost::asio::ip::tcp::resolver::iterator()) {
          ROS_ERROR("Could not resolve %s:%s", host.c_str(), port.c_str());
          return 1;
        }
      } catch(std::runtime_error& e) {
        ROS_ERROR("Could not resolve %s:%s: %s", host.c_str(), port.c_str(), e.what());
        return 1;
      }

      boost::asio::ip::tcp::socket *socket = new boost::asio::ip::tcp::socket(io_service);
      handle.reset(socket);

      try {
        socket->connect(*endpoint);
      } catch(std::runtime_error& e) {
        ROS_ERROR("Could not connect to %s:%s: %s", endpoint->host_name().c_str(), endpoint->service_name().c_str(), e.what());
        return 1;
      }

      ROS_INFO("Connected to %s:%s.", endpoint->host_name().c_str(), endpoint->service_name().c_str());
      gps.initialize(*socket, io_service);
    }

  } else {
    boost::asio::serial_port *serial = new boost::asio::serial_port(io_service);
    handle.reset(serial);

    // open serial port
    try {
      serial->open(device);
    } catch(std::runtime_error& e) {
      ROS_ERROR("Could not open serial port %s: %s", device.c_str(), e.what());
      delete serial;
      return 1;
    }

    ROS_INFO("Opened serial port %s", device.c_str());
    gps.initialize(*serial, io_service);
  }

  // configure the GPS
  ROS_INFO("Configuring GPS receiver...");
  if (!gps.configure()) {
    ROS_ERROR("Could not configure u-blox GPS");
    goto cleanup;
  }

  ROS_INFO("Done!");

  // subscribe messages
  param.param("all", enabled["all"], false);
  param.param("rxm", enabled["rxm"], false);
  param.param("aid", enabled["aid"], false);

  param.param("nav_sol", enabled["nav_sol"], enabled["all"]);
  if (enabled["nav_sol"]) gps.subscribe<ublox_msgs::NavSOL>(boost::bind(&publish<ublox_msgs::NavSOL>, _1, "/ublox/navsol"), 1);
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

  // wait...
  ros::spin();

  // cleanup
cleanup:
  gps.close();
  handle.reset();
  ROS_INFO("Closed connection to %s.", device.c_str());

  delete nh;
  return 0;
}
