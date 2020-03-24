#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <ublox_msgs/msg/nav_relposned9.hpp>

#include <ublox_gps/hp_pos_rec_product.hpp>
#include <ublox_gps/hpg_ref_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// U-Blox High Precision Positioning Receiver
//
HpPosRecProduct::HpPosRecProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node)
  : HpgRefProduct(nav_rate, meas_rate, updater, rtcms, node), frame_id_(frame_id)
{
  if (getRosBoolean(node_, "publish.nav.relposned")) {
    nav_relposned_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavRELPOSNED9>("navrelposned", 1);
  }

  if (getRosBoolean(node_, "publish.nav.heading")) {
    imu_pub_ =
      node_->create_publisher<sensor_msgs::msg::Imu>("navheading", 1);
  }
}

void HpPosRecProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Whether to publish Nav Relative Position NED
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  gps->subscribe<ublox_msgs::msg::NavRELPOSNED9>(std::bind(
     &HpPosRecProduct::callbackNavRelPosNed, this, std::placeholders::_1), 1);
}

void HpPosRecProduct::callbackNavRelPosNed(const ublox_msgs::msg::NavRELPOSNED9 &m) {
  if (getRosBoolean(node_, "publish.nav.relposned")) {
    nav_relposned_pub_->publish(m);
  }

  if (getRosBoolean(node_, "publish.nav.heading")) {
    imu_.header.stamp = node_->now();
    imu_.header.frame_id = frame_id_;

    imu_.linear_acceleration_covariance[0] = -1;
    imu_.angular_velocity_covariance[0] = -1;

    // Transform angle since ublox is representing heading as NED but ROS uses ENU as convention (REP-103).
    // Alos convert the base-to-rover angle to a robot-to-base angle (consistent with frame_id).
    double heading = (static_cast<double>(m.rel_pos_heading) * 1e-5 / 180.0 * M_PI) - M_PI_2;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, heading);
    imu_.orientation.x = orientation[0];
    imu_.orientation.y = orientation[1];
    imu_.orientation.z = orientation[2];
    imu_.orientation.w = orientation[3];
    imu_.orientation_covariance[0] = 1000.0;
    imu_.orientation_covariance[4] = 1000.0;
    imu_.orientation_covariance[8] = 1000.0;
    // When heading is reported to be valid, use accuracy reported in 1e-5 deg units
    if (m.flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REL_POS_HEAD_VALID) {
      imu_.orientation_covariance[8] = ::pow(m.acc_heading / 10000.0, 2);
    }

    imu_pub_->publish(imu_);
  }

  last_rel_pos_ = m;
  updater_->force_update();
}

}  // namespace ublox_node
