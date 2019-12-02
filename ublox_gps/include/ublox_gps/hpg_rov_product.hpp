#ifndef UBLOX_GPS_HPG_ROV_PRODUCT_HPP
#define UBLOX_GPS_HPG_ROV_PRODUCT_HPP

#include <memory>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <ublox_msgs/NavRELPOSNED.h>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for High Precision GNSS Rover devices.
 */
class HpgRovProduct final : public virtual ComponentInterface {
 public:
  // Constants for diagnostic updater
  //! Diagnostic updater: RTCM topic frequency min [Hz]
  constexpr static double kRtcmFreqMin = 1;
  //! Diagnostic updater: RTCM topic frequency max [Hz]
  constexpr static double kRtcmFreqMax = 10;
  //! Diagnostic updater: RTCM topic frequency tolerance [%]
  constexpr static double kRtcmFreqTol = 0.1;
  //! Diagnostic updater: RTCM topic frequency window [num messages]
  constexpr static int kRtcmFreqWindow = 25;

  explicit HpgRovProduct(uint16_t nav_rate, std::shared_ptr<diagnostic_updater::Updater> updater, ros::NodeHandle* node);

  /**
   * @brief Get the ROS parameters specific to the Rover configuration.
   *
   * @details Get the DGNSS mode.
   */
  void getRosParams() override;

  /**
   * @brief Configure rover settings.
   *
   * @details Configure the DGNSS mode.
   * @return true if configured correctly, false otherwise
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Add diagnostic updaters for rover GNSS status, including
   * status of RTCM messages.
   */
  void initializeRosDiagnostics() override;

  /**
   * @brief Subscribe to Rover messages, such as NavRELPOSNED.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  /**
   * @brief Update the rover diagnostics, including the carrier phase solution
   * status (float or fixed).
   */
  void carrierPhaseDiagnostics(
      diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Set the last received message and call rover diagnostic updater
   *
   * @details Publish received NavRELPOSNED messages if enabled
   */
  void callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED &m);


  //! Last relative position (used for diagnostic updater)
  ublox_msgs::NavRELPOSNED last_rel_pos_;

  //! The DGNSS mode
  /*! see CfgDGNSS message for possible values */
  uint8_t dgnss_mode_;

  //! The RTCM topic frequency diagnostic updater
  UbloxTopicDiagnostic freq_rtcm_;

  ros::Publisher nav_rel_pos_ned_pub_;

  uint16_t nav_rate_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  ros::NodeHandle* node_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_HPG_ROV_PRODUCT_HPP
