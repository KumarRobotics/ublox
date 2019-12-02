#ifndef UBLOX_GPS_UBLOX_FIRMWARE8_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE8_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <ublox_msgs/CfgNMEA.h>
#include <ublox_msgs/MonHW.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavSAT.h>
#include <ublox_msgs/RxmRTCM.h>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/ublox_firmware7plus.hpp>

namespace ublox_node {

/**
 *  @brief Implements functions for firmware version 8.
 */
class UbloxFirmware8 : public UbloxFirmware7Plus<ublox_msgs::NavPVT> {
 public:
  explicit UbloxFirmware8(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, ros::NodeHandle* node)
    : UbloxFirmware7Plus<ublox_msgs::NavPVT>(frame_id, updater, freq_diag, gnss, node) {
    nav_sat_pub_ = node->advertise<ublox_msgs::NavSAT>("navstate", 1);
    mon_hw_pub_ = node->advertise<ublox_msgs::MonHW>("monhw", 1);
    rxm_rtcm_pub_ = node->advertise<ublox_msgs::RxmRTCM>("rxmrtcm", 1);
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
  bool enable_galileo_;
  //! Whether or not to enable the BeiDuo GNSS
  bool enable_beidou_;
  //! Whether or not to enable the IMES GNSS
  bool enable_imes_;
  //! Desired NMEA configuration.
  ublox_msgs::CfgNMEA cfg_nmea_;
  //! Whether to clear the flash memory during configuration
  bool clear_bbr_;
  bool save_on_shutdown_;

  ros::Publisher nav_sat_pub_;
  ros::Publisher mon_hw_pub_;
  ros::Publisher rxm_rtcm_pub_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE8_HPP
