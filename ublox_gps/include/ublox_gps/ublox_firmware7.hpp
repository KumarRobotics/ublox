#ifndef UBLOX_GPS_UBLOX_FIRMWARE7_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE7_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_nmea7.hpp>
#include <ublox_msgs/msg/mon_hw.hpp>
#include <ublox_msgs/msg/nav_pvt7.hpp>
#include <ublox_msgs/msg/nav_svinfo.hpp>

#include <ublox_gps/gnss.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/ublox_firmware7plus.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for firmware version 7.
 */
class UbloxFirmware7 final : public UbloxFirmware7Plus<ublox_msgs::msg::NavPVT7> {
 public:
  explicit UbloxFirmware7(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, rclcpp::Node* node)
    : UbloxFirmware7Plus<ublox_msgs::msg::NavPVT7>(frame_id, updater, freq_diag, gnss, node) {
    if (getRosBoolean(node_, "publish.nav.svinfo")) {
      nav_svinfo_pub_ = node->create_publisher<ublox_msgs::msg::NavSVINFO>("navsvinfo", 1);
    }
    if (getRosBoolean(node_, "publish.mon.hw")) {
      mon_hw_pub_ = node->create_publisher<ublox_msgs::msg::MonHW>("monhw", 1);
    }
  }

  /**
   * @brief Get the parameters specific to firmware version 7.
   *
   * @details Get the GNSS and NMEA settings.
   */
  void getRosParams() override;

  /**
   * @brief Configure GNSS individually. Only configures GLONASS.
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Subscribe to messages which are not generic to all firmware.
   *
   * @details Subscribe to NavPVT7 messages, RxmRAW, and RxmSFRB messages.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  //! Used to configure NMEA (if set_nmea_)
  /*!
   * Filled from ROS parameters
   */
  ublox_msgs::msg::CfgNMEA7 cfg_nmea_;

  rclcpp::Publisher<ublox_msgs::msg::NavSVINFO>::SharedPtr nav_svinfo_pub_;
  rclcpp::Publisher<ublox_msgs::msg::MonHW>::SharedPtr mon_hw_pub_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE7_HPP
