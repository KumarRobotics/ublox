#ifndef UBLOX_GPS_RAW_DATA_PRODUCT_HPP
#define UBLOX_GPS_RAW_DATA_PRODUCT_HPP

#include <memory>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/ublox_topic_diagnostic.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for Raw Data products.
 */
class RawDataProduct final : public virtual ComponentInterface {
 public:
  double kRtcmFreqTol = 0.15;
  int kRtcmFreqWindow = 25;

  explicit RawDataProduct(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, ros::NodeHandle* node);

  /**
   * @brief Does nothing since there are no Raw Data product specific settings.
   */
  void getRosParams() override {}

  /**
   * @brief Does nothing since there are no Raw Data product specific settings.
   * @return always returns true
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override {
    (void)gps;
    return true;
  }

  /**
   * @brief Adds frequency diagnostics for RTCM topics.
   */
  void initializeRosDiagnostics() override;

  /**
   * @brief Subscribe to Raw Data Product messages and set up ROS publishers.
   *
   * @details Subscribe to RxmALM, RxmEPH, RxmRAW, and RxmSFRB messages.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:
  //! Topic diagnostic updaters
  std::vector<std::shared_ptr<UbloxTopicDiagnostic> > freq_diagnostics_;

  ros::Publisher rxm_raw_pub_;
  ros::Publisher rxm_sfrb_pub_;
  ros::Publisher rxm_eph_pub_;
  ros::Publisher rxm_alm_pub_;

  uint16_t nav_rate_;
  uint16_t meas_rate_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  ros::NodeHandle* node_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_RAW_DATA_PRODUCT_HPP
