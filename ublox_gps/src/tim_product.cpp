#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/rxm_rawx.hpp>
#include <ublox_msgs/msg/rxm_sfrbx.hpp>
#include <ublox_msgs/msg/tim_tm2.hpp>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/tim_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// U-Blox Time Sync Products, partially implemented.
//
TimProduct::TimProduct(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, rclcpp::Node* node) : frame_id_(frame_id), updater_(updater), node_(node)
{
  timtm2_pub_ =
    node_->create_publisher<ublox_msgs::msg::TimTM2>("timtm2", 1);
  interrupt_time_pub_ =
    node_->create_publisher<sensor_msgs::msg::TimeReference>("interrupt_time", 1);

  if (getRosBoolean(node_, "publish.rxm.sfrb")) {
    rxm_sfrb_pub_ = node_->create_publisher<ublox_msgs::msg::RxmSFRBX>("rxmsfrb", 1);
  }
  if (getRosBoolean(node_, "publish.rxm.raw")) {
    rxm_raw_pub_ = node_->create_publisher<ublox_msgs::msg::RxmRAWX>("rxmraw", 1);
  }
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
  gps->subscribe<ublox_msgs::msg::TimTM2>(std::bind(
    &TimProduct::callbackTimTM2, this, std::placeholders::_1), 1);

  // RCLCPP_INFO("Subscribed to TIM-TM2 messages on topic tim/tm2");

  // Subscribe to SFRBX messages
  if (getRosBoolean(node_, "publish.rxm.sfrb")) {
    gps->subscribe<ublox_msgs::msg::RxmSFRBX>([this](const ublox_msgs::msg::RxmSFRBX &m) { rxm_sfrb_pub_->publish(m); },
                                         1);
  }

  // Subscribe to RawX messages
  if (getRosBoolean(node_, "publish.rxm.raw")) {
    gps->subscribe<ublox_msgs::msg::RxmRAWX>([this](const ublox_msgs::msg::RxmRAWX &m) { rxm_raw_pub_->publish(m); },
                                         1);
  }
}

void TimProduct::callbackTimTM2(const ublox_msgs::msg::TimTM2 &m) {
  if (getRosBoolean(node_, "publish.tim.tm2")) {
    // create time ref message and put in the data
    t_ref_.header.stamp = node_->now();
    t_ref_.header.frame_id = frame_id_;

    t_ref_.time_ref = rclcpp::Time((m.wn_r * 604800 + m.tow_ms_r / 1000), (m.tow_ms_r % 1000) * 1000000 + m.tow_sub_ms_r);

    std::ostringstream src;
    src << "TIM" << int(m.ch);
    t_ref_.source = src.str();

    t_ref_.header.stamp = node_->now(); // create a new timestamp
    t_ref_.header.frame_id = frame_id_;

    timtm2_pub_->publish(m);
    interrupt_time_pub_->publish(t_ref_);
  }

  updater_->force_update();
}

void TimProduct::initializeRosDiagnostics() {
  updater_->force_update();
}

}  // namespace ublox_node
