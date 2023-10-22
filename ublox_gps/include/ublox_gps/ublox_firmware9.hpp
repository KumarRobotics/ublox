#ifndef UBLOX_GPS_UBLOX_FIRMWARE9_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE9_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/cfg_valset.hpp>
#include <ublox_msgs/msg/cfg_valset_cfgdata.hpp>

#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware8.hpp>

namespace ublox_node {

/**
 *  @brief Implements functions for firmware version 9.
 */
class UbloxFirmware9 final : public UbloxFirmware8 {
public:
  explicit UbloxFirmware9(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, rclcpp::Node* node);

  /**
    * @brief Configure settings specific to firmware 9 based on ROS parameters.
    *
    * @details Configure GNSS.  The hardware has internal logic for
    * detecting differences between the new and active GNSS
    * confifuration and will internally trigger a reset if necessary.
    * Configure the NMEA if desired by the user. It also may clear the
    * flash memory based on the ROS parameters.
    */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;
private:
  /**
    * @brief Populate the CfgVALSETCfgData data type
    *
    * @details A helper function used to generate a configuration for a singal signal. 
    */
  ublox_msgs::msg::CfgVALSETCfgdata generateSignalConfig_(uint32_t signalID, bool enable);
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE9_HPP
