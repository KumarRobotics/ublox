#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/fix_diagnostic.hpp>
#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware8.hpp>
#include <ublox_gps/ublox_firmware9.hpp>

namespace ublox_node {

UbloxFirmware9::UbloxFirmware9(const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<FixDiagnostic> freq_diag, std::shared_ptr<Gnss> gnss, rclcpp::Node* node)
  : UbloxFirmware8(frame_id, updater, freq_diag, gnss, node)
{
}

}  // namespace ublox_node
