#ifndef UBLOX_GPS_UBLOX_FIRMWARE6_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE6_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <ublox_msgs/CfgNMEA6.h>
#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavSOL.h>
#include <ublox_msgs/NavVELNED.h>

#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/ublox_firmware.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for firmware version 6.
 */
class UbloxFirmware6 final : public UbloxFirmware {
 public:
  explicit UbloxFirmware6(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, ros::NodeHandle* node);

  /**
   * @brief Sets the fix status service type to GPS.
   */
  void getRosParams() override;

  /**
   * @brief Prints a warning, GNSS configuration not available in this version.
   * @return true if configured correctly, false otherwise
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Subscribe to NavPVT, RxmRAW, and RxmSFRB messages.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 protected:
  /**
   * @brief Updates fix diagnostic from NavPOSLLH, NavVELNED, and NavSOL
   * messages.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) override;

 private:
  /**
   * @brief Publish the fix and call the fix diagnostic updater.
   *
   * @details Also updates the last known position and publishes the NavPosLLH
   * message if publishing is enabled.
   * @param m the message to process
   */
  void callbackNavPosLlh(const ublox_msgs::NavPOSLLH& m);

  /**
   * @brief Update the last known velocity.
   *
   * @details Publish the message if publishing is enabled.
   * @param m the message to process
   */
  void callbackNavVelNed(const ublox_msgs::NavVELNED& m);

  /**
   * @brief Update the number of SVs used for the fix.
   *
   * @details Publish the message if publishing is enabled.
   * @param m the message to process
   */
  void callbackNavSol(const ublox_msgs::NavSOL& m);

  //! The last received navigation position
  ublox_msgs::NavPOSLLH last_nav_pos_;
  //! The last received navigation velocity
  ublox_msgs::NavVELNED last_nav_vel_;
  //! The last received num SVs used
  ublox_msgs::NavSOL last_nav_sol_;
  //! The last NavSatFix based on last_nav_pos_
  sensor_msgs::NavSatFix fix_;
  //! The last Twist based on last_nav_vel_
  geometry_msgs::TwistWithCovarianceStamped velocity_;

  //! Used to configure NMEA (if set_nmea_) filled with ROS parameters
  ublox_msgs::CfgNMEA6 cfg_nmea_;

  ros::Publisher nav_pos_llh_pub_;
  ros::Publisher fix_pub_;
  ros::Publisher nav_vel_ned_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher nav_sol_pub_;
  ros::Publisher nav_svinfo_pub_;
  ros::Publisher mon_hw_pub_;

  std::string frame_id_;
  std::shared_ptr<FixDiagnostic> freq_diag_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE6_HPP
