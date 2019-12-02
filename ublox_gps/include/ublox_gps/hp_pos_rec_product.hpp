#ifndef UBLOX_GPS_HP_POS_REC_PRODUCT_HPP
#define UBLOX_GPS_HP_POS_REC_PRODUCT_HPP

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <ublox_msgs/NavRELPOSNED9.h>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/hpg_ref_product.hpp>
#include <ublox_gps/rtcm.hpp>

namespace ublox_node {

class HpPosRecProduct final : public virtual HpgRefProduct {
 public:
  explicit HpPosRecProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, ros::NodeHandle* node);

  /**
   * @brief Subscribe to Rover messages, such as NavRELPOSNED.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 private:

  /**
   * @brief Set the last received message and call rover diagnostic updater
   *
   * @details Publish received NavRELPOSNED messages if enabled
   */
  void callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED9 &m);

  sensor_msgs::Imu imu_;

  //! Last relative position (used for diagnostic updater)
  ublox_msgs::NavRELPOSNED9 last_rel_pos_;

  ros::Publisher nav_relposned_pub_;
  ros::Publisher imu_pub_;

  std::string frame_id_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_HP_POS_REC_PRODUCT_HPP
