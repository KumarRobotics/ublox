#ifndef UBLOX_GPS_UBLOX_FIRMWARE7_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE7_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <ublox_msgs/CfgNMEA7.h>
#include <ublox_msgs/MonHW.h>
#include <ublox_msgs/NavPVT7.h>
#include <ublox_msgs/NavSVINFO.h>

#include <ublox_gps/gnss.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/ublox_firmware7plus.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for firmware version 7.
 */
class UbloxFirmware7 final : public UbloxFirmware7Plus<ublox_msgs::NavPVT7> {
 public:
  explicit UbloxFirmware7(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, ros::NodeHandle* node)
    : UbloxFirmware7Plus<ublox_msgs::NavPVT7>(frame_id, updater, freq_diag, gnss, node) {
    nav_svinfo_pub_ = node->advertise<ublox_msgs::NavSVINFO>("navsvinfo", 1);
    mon_hw_pub_ = node->advertise<ublox_msgs::MonHW>("monhw", 1);
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
  ublox_msgs::CfgNMEA7 cfg_nmea_;

  ros::Publisher nav_svinfo_pub_;
  ros::Publisher mon_hw_pub_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE7_HPP
