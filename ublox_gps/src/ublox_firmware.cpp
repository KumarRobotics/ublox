#include <memory>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <ublox_gps/gnss.hpp>
#include <ublox_gps/ublox_firmware.hpp>

namespace ublox_node {

//
// U-Blox Firmware (all versions)
//
UbloxFirmware::UbloxFirmware(std::shared_ptr<diagnostic_updater::Updater> updater, std::shared_ptr<Gnss> gnss, ros::NodeHandle* node) : updater_(updater), gnss_(gnss), node_(node)
{
}

void UbloxFirmware::initializeRosDiagnostics() {
  updater_->add("fix", this, &UbloxFirmware::fixDiagnostic);
  updater_->force_update();
}

}  // namespace ublox_node
