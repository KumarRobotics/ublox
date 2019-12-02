#ifndef UBLOX_GPS_UBLOX_FIRMWARE_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE_HPP

#include <memory>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gnss.hpp>

namespace ublox_node {

/**
 * @brief This abstract class represents a firmware component.
 *
 * @details The Firmware components update the fix diagnostics.
 */
class UbloxFirmware : public virtual ComponentInterface {
 public:
  //! Subscribe Rate for u-blox SV Info messages
  constexpr static uint32_t kNavSvInfoSubscribeRate = 20;

  explicit UbloxFirmware(std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<Gnss> gnss, ros::NodeHandle* node);

  /**
   * @brief Add the fix diagnostics to the updater.
   */
  void initializeRosDiagnostics() override;

 protected:
  /**
   * @brief Handle to send fix status to ROS diagnostics.
   */
  virtual void fixDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper& stat) = 0;

  std::shared_ptr<diagnostic_updater::Updater> updater_;
  std::shared_ptr<Gnss> gnss_;
  //! The fix status service type, set in the Firmware Component
  //! based on the enabled GNSS
  int fix_status_service_{0};
  ros::NodeHandle* node_;
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE_HPP
