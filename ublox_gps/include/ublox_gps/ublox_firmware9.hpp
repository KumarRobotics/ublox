#ifndef UBLOX_GPS_UBLOX_FIRMWARE9_HPP
#define UBLOX_GPS_UBLOX_FIRMWARE9_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware8.hpp>

namespace ublox_node {

/**
 *  @brief Implements functions for firmware version 9.
 *  For now it simply re-uses the firmware version 8 class
 *  but allows for future expansion of functionality
 */
class UbloxFirmware9 final : public UbloxFirmware8 {
public:
  explicit UbloxFirmware9(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, rclcpp::Node* node);
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_UBLOX_FIRMWARE9_HPP
