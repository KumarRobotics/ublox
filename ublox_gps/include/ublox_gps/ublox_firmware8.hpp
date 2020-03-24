#ifndef UBLOX_GPS_UBLOX_FIRMWARE8_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE8_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_nmea.hpp>
#include <ublox_msgs/msg/mon_hw.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <ublox_msgs/msg/nav_sat.hpp>
#include <ublox_msgs/msg/rxm_rtcm.hpp>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/ublox_firmware7plus.hpp>

namespace ublox_node {

/**
 *  @brief Implements functions for firmware version 8.
 */
class UbloxFirmware8 : public UbloxFirmware7Plus<ublox_msgs::msg::NavPVT> {
 public:
  explicit UbloxFirmware8(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, rclcpp::Node* node)
    : UbloxFirmware7Plus<ublox_msgs::msg::NavPVT>(frame_id, updater, freq_diag, gnss, node) {
    if (getRosBoolean(node_, "publish.nav.sat")) {
      nav_sat_pub_ = node->create_publisher<ublox_msgs::msg::NavSAT>("navstate", 1);
    }
    if (getRosBoolean(node_, "publish.mon.hw")) {
      mon_hw_pub_ = node->create_publisher<ublox_msgs::msg::MonHW>("monhw", 1);
    }
    if (getRosBoolean(node_, "publish.rxm.rtcm")) {
      rxm_rtcm_pub_ = node->create_publisher<ublox_msgs::msg::RxmRTCM>("rxmrtcm", 1);
    }
  }

  /**
   * @brief Get the ROS parameters specific to firmware version 8.
   *
   * @details Get the GNSS, NMEA, and UPD settings.
   */
  void getRosParams() override;

  /**
   * @brief Configure settings specific to firmware 8 based on ROS parameters.
   *
   * @details Configure GNSS, if it is different from current settings.
   * Configure the NMEA if desired by the user. It also may clear the
   * flash memory based on the ROS parameters.
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Subscribe to u-blox messages which are not generic to all firmware
   * versions.
   *
   * @details Subscribe to NavPVT, NavSAT, MonHW, and RxmRTCM messages based
   * on user settings.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  // Set from ROS parameters
  //! Whether or not to enable the Galileo GNSS
  bool enable_galileo_{false};
  //! Whether or not to enable the BeiDuo GNSS
  bool enable_beidou_{false};
  //! Whether or not to enable the IMES GNSS
  bool enable_imes_{false};
  //! Desired NMEA configuration.
  ublox_msgs::msg::CfgNMEA cfg_nmea_;
  //! Whether to clear the flash memory during configuration
  bool clear_bbr_{false};
  bool save_on_shutdown_{false};

  rclcpp::Publisher<ublox_msgs::msg::NavSAT>::SharedPtr nav_sat_pub_;
  rclcpp::Publisher<ublox_msgs::msg::MonHW>::SharedPtr mon_hw_pub_;
  rclcpp::Publisher<ublox_msgs::msg::RxmRTCM>::SharedPtr rxm_rtcm_pub_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE8_HPP
