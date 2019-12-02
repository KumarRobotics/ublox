#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>

#include <ublox_msgs/RxmRAWX.h>
#include <ublox_msgs/RxmSFRBX.h>
#include <ublox_msgs/TimTM2.h>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/tim_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// U-Blox Time Sync Products, partially implemented.
//
TimProduct::TimProduct(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, ros::NodeHandle* node) : frame_id_(frame_id), updater_(updater), node_(node)
{
  timtm2_pub_ =
    node_->advertise<ublox_msgs::TimTM2>("timtm2", 1);
  interrupt_time_pub_ =
    node_->advertise<sensor_msgs::TimeReference>("interrupt_time", 1);
  rxm_sfrb_pub_ = node_->advertise<ublox_msgs::RxmSFRBX>("rxmsfrb", 1);
  rxm_raw_pub_ = node_->advertise<ublox_msgs::RxmRAWX>("rxmraw", 1);
}

void TimProduct::getRosParams() {
}

bool TimProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  uint8_t r = 1;
  // Configure the reciever
  if (!gps->setUTCtime()) {
    throw std::runtime_error(std::string("Failed to Configure TIM Product to UTC Time"));
  }

  if (!gps->setTimtm2(r)) {
    throw std::runtime_error(std::string("Failed to Configure TIM Product"));
  }

  return true;
}

void TimProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  gps->subscribe<ublox_msgs::TimTM2>(std::bind(
    &TimProduct::callbackTimTM2, this, std::placeholders::_1), 1);

  ROS_INFO("Subscribed to TIM-TM2 messages on topic tim/tm2");

  // Subscribe to SFRBX messages
  if (getRosBoolean(node_, "publish/rxm/sfrb")) {
    gps->subscribe<ublox_msgs::RxmSFRBX>([this](const ublox_msgs::RxmSFRBX &m) { rxm_sfrb_pub_.publish(m); },
                                         1);
  }

   // Subscribe to RawX messages
   if (getRosBoolean(node_, "publish/rxm/raw")) {
     gps->subscribe<ublox_msgs::RxmRAWX>([this](const ublox_msgs::RxmRAWX &m) { rxm_raw_pub_.publish(m); },
                                         1);
   }
}

void TimProduct::callbackTimTM2(const ublox_msgs::TimTM2 &m) {
  if (getRosBoolean(node_, "publish/tim/tm2")) {
    // create time ref message and put in the data
    t_ref_.header.seq = m.rising_edge_count;
    t_ref_.header.stamp = ros::Time::now();
    t_ref_.header.frame_id = frame_id_;

    t_ref_.time_ref = ros::Time((m.wn_r * 604800 + m.tow_ms_r / 1000), (m.tow_ms_r % 1000) * 1000000 + m.tow_sub_ms_r);

    std::ostringstream src;
    src << "TIM" << int(m.ch);
    t_ref_.source = src.str();

    t_ref_.header.stamp = ros::Time::now(); // create a new timestamp
    t_ref_.header.frame_id = frame_id_;

    timtm2_pub_.publish(m);
    interrupt_time_pub_.publish(t_ref_);
  }

  updater_->force_update();
}

void TimProduct::initializeRosDiagnostics() {
  updater_->force_update();
}

}  // namespace ublox_node
