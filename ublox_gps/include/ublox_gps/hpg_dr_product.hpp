#ifndef UBLOX_GPS_HPG_DR_PRODUCT_HPP
#define UBLOX_GPS_HPG_DR_PRODUCT_HPP

#include <memory>
#include <string>
#include <vector>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <ublox_msgs/msg/esf_ins.hpp>
#include <ublox_msgs/msg/esf_meas.hpp>
#include <ublox_msgs/msg/esf_raw.hpp>
#include <ublox_msgs/msg/nav_att.hpp>
#include <ublox_msgs/msg/nav_hpposllh.hpp>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/hpg_rov_product.hpp>
#include <ublox_gps/rtcm.hpp>

namespace ublox_node {

/**
 * @brief High-Precision GNSS Dead Reckoning product interface for firmware >= 8
 */
class HpgDrProduct final : public virtual ComponentInterface {
 public:
  explicit HpgDrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node);

  /**
   * @brief Subscribe to High-Precision GNSS DR messages.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;


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

  void initializeRosDiagnostics() override {
    // RCLCPP_WARN("ROS Diagnostics specific to u-blox ADR/UDR devices is %s",
    //          "unimplemented. See AdrUdrProduct class in node.hpp & node.cpp.");
  }

 private:

  // Scaling factors for floating point representations of data
  //static constexpr float kConvertRadPerSec = std::pow(2, -12) * M_PI / 180.0F;
  //static constexpr float kConvertMPerSecSq = std::pow(2, -10);
  //static constexpr float kConvertDegCelsius{0.01F};

  // Callbacks relevant to ROS2 message output triggered on U-Blox message events
  void callbackEsfIns(const ublox_msgs::msg::EsfINS &m);
  void callbackEsfRAW(const ublox_msgs::msg::EsfRAW &m);
  void callbackEsfMEAS(const ublox_msgs::msg::EsfMEAS &m);
  void callbackEsfStatus(const ublox_msgs::msg::EsfSTATUS &m);
  void callbackNavHpPosLlh(const ublox_msgs::msg::NavHPPOSLLH &m);
  void callbackNavATT(const ublox_msgs::msg::NavATT &m);
  void callbackNavPvt(const ublox_msgs::msg::NavPVT &m);

  // Reused containers for ROS2 messages
  sensor_msgs::msg::Imu imu_meas_;
  sensor_msgs::msg::Imu imu_raw_;
  sensor_msgs::msg::Imu imu_att_;
  sensor_msgs::msg::Imu esf_ins_ros_;
  sensor_msgs::msg::Temperature imu_temp_;

  sensor_msgs::msg::NavSatFix fix_hp_;
  diagnostic_msgs::msg::DiagnosticStatus nav_diag_;

  //! Last global position (used for diagnostic updater)
  ublox_msgs::msg::NavHPPOSLLH last_hp_pos_;

  // ROS2 publishers for ROS2 messages
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_meas_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_att_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr esf_ins_ros_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_hp_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr nav_diag_pub_;

  // ROS2 republishers for U-Blox messages
  rclcpp::Publisher<ublox_msgs::msg::NavATT>::SharedPtr nav_att_pub_;
  rclcpp::Publisher<ublox_msgs::msg::NavHPPOSLLH>::SharedPtr nav_hpposllh_pub_;
  rclcpp::Publisher<ublox_msgs::msg::NavPVT>::SharedPtr nav_pvt_pub_;
  rclcpp::Publisher<ublox_msgs::msg::EsfINS>::SharedPtr esf_ins_pub_;
  rclcpp::Publisher<ublox_msgs::msg::EsfMEAS>::SharedPtr esf_meas_pub_;
  rclcpp::Publisher<ublox_msgs::msg::EsfRAW>::SharedPtr esf_raw_pub_;
  rclcpp::Publisher<ublox_msgs::msg::EsfSTATUS>::SharedPtr esf_status_pub_;

  // Relevant configuration state parameters
  bool use_adr_;
  uint16_t nav_rate_;
  uint16_t meas_rate_;

  std::string frame_id_;

  std::shared_ptr<diagnostic_updater::Updater> updater_;
  std::vector<ublox_gps::Rtcm> rtcms_;
  rclcpp::Node* node_;

  // Local cache of U-Blox messages to be fused within a navigation epoch (data frame)
  ublox_msgs::msg::NavATT last_nav_att_;
  ublox_msgs::msg::NavPVT last_nav_pvt_;
  std::pair<std::uint32_t, builtin_interfaces::msg::Time> last_itow_time_;

};

}  // namespace ublox_node

#endif  // UBLOX_GPS_HPG_DR_PRODUCT_HPP
