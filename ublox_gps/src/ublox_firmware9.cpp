#include <memory>
#include <string>
#include <vector>

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

bool UbloxFirmware9::configureUblox(std::shared_ptr<ublox_gps::Gps> gps)
{
  if (clear_bbr_)
  {
    // clear flash memory
    if (!gps->clearBbr())
    {
      RCLCPP_ERROR(node_->get_logger(), "u-blox failed to clear flash memory");
    }
  }

  gps->setSaveOnShutdown(save_on_shutdown_);

  //
  // Configure the GNSS, only if the configuration is different
  //
  // First, get the current GNSS configuration
  ublox_msgs::msg::CfgGNSS cfg_gnss;
  if (gps->poll(cfg_gnss))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Read GNSS config.");
    RCLCPP_DEBUG(node_->get_logger(), "Num. tracking channels in hardware: %i", cfg_gnss.num_trk_ch_hw);
    RCLCPP_DEBUG(node_->get_logger(), "Num. tracking channels to use: %i", cfg_gnss.num_trk_ch_use);
  }
  else
  {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  ublox_msgs::msg::CfgVALSET cfg_signal;
  cfg_signal.layers = ublox_msgs::msg::CfgVALSET::LAYER_RAM;

  using signal = ublox_msgs::msg::CfgVALSETCfgdata;

  // Configure GPS Signals
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GPS_ENABLE, enable_gps_));
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GPS_L1CA_ENABLE, enable_gps_));
  if (gnss_->isSupported("GPS_L2C"))
  {
    cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GPS_L2C_ENABLE, enable_gps_));
  }

  // Configure SBAS Signals
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::SBAS_ENABLE, enable_gps_));
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::SBAS_L1CA_ENABLE, enable_gps_));

  // Configure Galileo Signals
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GAL_ENABLE, enable_galileo_));
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GAL_E1_ENABLE, enable_galileo_));
  if (gnss_->isSupported("GAL_E5B"))
  {
    cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GAL_E5B_ENABLE, enable_galileo_));
  }

  // Configure Beidou Signals
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::BDS_ENABLE, enable_beidou_));
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::BDS_B1_ENABLE, enable_beidou_));
  if (gnss_->isSupported("BDS_B2"))
  {
    cfg_signal.cfgdata.push_back(generateSignalConfig(signal::BDS_B2_ENABLE, enable_beidou_));
  }

  // Configure QZSS Signals
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::QZSS_ENABLE, enable_qzss_ && enable_gps_));
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::QZSS_L1CA_ENABLE, enable_qzss_ && enable_gps_));
  if (gnss_->isSupported("QZSS_L2C"))
  {
    cfg_signal.cfgdata.push_back(generateSignalConfig(signal::QZSS_L2C_ENABLE, enable_qzss_ && enable_gps_));
  }

  // Configure GLONASS Signals
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GLO_ENABLE, enable_glonass_));
  cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GLO_L1_ENABLE, enable_glonass_));
  if (gnss_->isSupported("GLO_L2"))
  {
    cfg_signal.cfgdata.push_back(generateSignalConfig(signal::GLO_L2_ENABLE, enable_glonass_));
  }

  RCLCPP_DEBUG(node_->get_logger(), "Ready to configure");
  gps->configure(cfg_signal, false);

  //
  // NMEA config
  //
  if (getRosBoolean(node_, "nmea.set") && !gps->configure(cfg_nmea_))
  {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

ublox_msgs::msg::CfgVALSETCfgdata UbloxFirmware9::generateSignalConfig(uint32_t signalID, bool enable)
{
  ublox_msgs::msg::CfgVALSETCfgdata signalConfig;
  signalConfig.key = signalID;
  signalConfig.data.resize(1);
  signalConfig.data[0] = enable;
  return signalConfig;
}


}  // namespace ublox_node
