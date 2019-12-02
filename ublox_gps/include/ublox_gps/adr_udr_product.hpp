#ifndef UBLOX_GPS_ADR_UDR_PRODUCT_HPP
#define UBLOX_GPS_ADR_UDR_PRODUCT_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>

#include <ublox_msgs/EsfMEAS.h>
#include <ublox_msgs/TimTM2.h>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for Automotive Dead Reckoning (ADR) and
 * Untethered Dead Reckoning (UDR) Devices.
 */
class AdrUdrProduct final : public virtual ComponentInterface {
 public:
  explicit AdrUdrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, ros::NodeHandle* node);

  /**
   * @brief Get the ADR/UDR parameters.
   *
   * @details Get the use_adr parameter and check that the nav_rate is 1 Hz.
   */
  void getRosParams() override;

  /**
   * @brief Configure ADR/UDR settings.
   * @details Configure the use_adr setting.
   * @return true if configured correctly, false otherwise
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Initialize the ROS diagnostics for the ADR/UDR device.
   * @todo unimplemented
   */
  void initializeRosDiagnostics() override {
    ROS_WARN("ROS Diagnostics specific to u-blox ADR/UDR devices is %s",
             "unimplemented. See AdrUdrProduct class in node.hpp & node.cpp.");
  }

  /**
   * @brief Subscribe to ADR/UDR messages.
   *
   * @details Subscribe to NavATT, ESF and HNR messages based on user
   * parameters.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  //! Whether or not to enable dead reckoning
  bool use_adr_;

  sensor_msgs::Imu imu_;
  sensor_msgs::TimeReference t_ref_;
  ublox_msgs::TimTM2 timtm2;

  void callbackEsfMEAS(const ublox_msgs::EsfMEAS &m);

  ros::Publisher imu_pub_;
  ros::Publisher time_ref_pub_;
  ros::Publisher nav_att_pub_;
  ros::Publisher esf_ins_pub_;
  ros::Publisher esf_meas_pub_;
  ros::Publisher esf_raw_pub_;
  ros::Publisher esf_status_pub_;
  ros::Publisher hnr_pvt_pub_;

  uint16_t nav_rate_;
  uint16_t meas_rate_;

  std::string frame_id_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  ros::NodeHandle* node_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_ADR_UDR_PRODUCT_HPP
