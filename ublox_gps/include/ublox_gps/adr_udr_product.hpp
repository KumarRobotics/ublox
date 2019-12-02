#ifndef UBLOX_GPS_ADR_UDR_PRODUCT_HPP
#define UBLOX_GPS_ADR_UDR_PRODUCT_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/esf_ins.hpp>
#include <ublox_msgs/msg/esf_meas.hpp>
#include <ublox_msgs/msg/esf_raw.hpp>
#include <ublox_msgs/msg/esf_status.hpp>
#include <ublox_msgs/msg/hnr_pvt.hpp>
#include <ublox_msgs/msg/nav_att.hpp>
#include <ublox_msgs/msg/tim_tm2.hpp>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for Automotive Dead Reckoning (ADR) and
 * Untethered Dead Reckoning (UDR) Devices.
 */
class AdrUdrProduct final : public virtual ComponentInterface {
 public:
  explicit AdrUdrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, rclcpp::Node* node);

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
    // RCLCPP_WARN("ROS Diagnostics specific to u-blox ADR/UDR devices is %s",
    //          "unimplemented. See AdrUdrProduct class in node.hpp & node.cpp.");
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

  sensor_msgs::msg::Imu imu_;
  sensor_msgs::msg::TimeReference t_ref_;

  void callbackEsfMEAS(const ublox_msgs::msg::EsfMEAS &m);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;
  rclcpp::Publisher<ublox_msgs::msg::NavATT>::SharedPtr nav_att_pub_;
  rclcpp::Publisher<ublox_msgs::msg::EsfINS>::SharedPtr esf_ins_pub_;
  rclcpp::Publisher<ublox_msgs::msg::EsfMEAS>::SharedPtr esf_meas_pub_;
  rclcpp::Publisher<ublox_msgs::msg::EsfRAW>::SharedPtr esf_raw_pub_;
  rclcpp::Publisher<ublox_msgs::msg::EsfSTATUS>::SharedPtr esf_status_pub_;
  rclcpp::Publisher<ublox_msgs::msg::HnrPVT>::SharedPtr hnr_pvt_pub_;

  uint16_t nav_rate_;
  uint16_t meas_rate_;

  std::string frame_id_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  rclcpp::Node* node_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_ADR_UDR_PRODUCT_HPP
