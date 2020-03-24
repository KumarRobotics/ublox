#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <ublox_msgs/msg/cfg_gnss_block.hpp>
#include <ublox_msgs/msg/mon_hw.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <ublox_msgs/msg/nav_sat.hpp>
#include <ublox_msgs/msg/rxm_rtcm.hpp>

#include <ublox_gps/ublox_firmware8.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// Ublox Version 8
//
void UbloxFirmware8::getRosParams() {
  // UPD SOS configuration
  clear_bbr_ = getRosBoolean(node_, "clear_bbr");
  save_on_shutdown_ = getRosBoolean(node_, "save_on_shutdown");

  // GNSS enable/disable
  enable_gps_ = getRosBoolean(node_, "gnss.gps");
  enable_galileo_ = getRosBoolean(node_, "gnss.galileo");
  enable_beidou_ = getRosBoolean(node_, "gnss.beidou");
  enable_imes_ = getRosBoolean(node_, "gnss.imes");
  enable_glonass_ = getRosBoolean(node_, "gnss.glonass");
  enable_qzss_ = getRosBoolean(node_, "gnss.qzss");

  // QZSS Signal Configuration
  getRosUint(node_, "gnss.qzss_sig_cfg", qzss_sig_cfg_,
              ublox_msgs::msg::CfgGNSSBlock::SIG_CFG_QZSS_L1CA);

  if (enable_gps_ && !gnss_->isSupported("GPS")) {
    RCLCPP_WARN(node_->get_logger(), "gnss.gps is true, but GPS GNSS is not supported by %s",
             "this device");
  }
  if (enable_glonass_ && !gnss_->isSupported("GLO")) {
    RCLCPP_WARN(node_->get_logger(), "gnss.glonass is true, but GLONASS is not supported by %s",
             "this device");
  }
  if (enable_galileo_ && !gnss_->isSupported("GAL")) {
    RCLCPP_WARN(node_->get_logger(), "gnss.galileo is true, but Galileo GNSS is not supported %s",
             "by this device");
  }
  if (enable_beidou_ && !gnss_->isSupported("BDS")) {
    RCLCPP_WARN(node_->get_logger(), "gnss.beidou is true, but Beidou GNSS is not supported %s",
             "by this device");
  }
  if (enable_imes_ && !gnss_->isSupported("IMES")) {
    RCLCPP_WARN(node_->get_logger(), "gnss.imes is true, but IMES GNSS is not supported by %s",
             "this device");
  }
  if (enable_qzss_ && !gnss_->isSupported("QZSS")) {
    RCLCPP_WARN(node_->get_logger(), "gnss.qzss is true, but QZSS is not supported by this device");
  }
  if (getRosBoolean(node_, "gnss.sbas") && !gnss_->isSupported("SBAS")) {
    RCLCPP_WARN(node_->get_logger(), "gnss.sbas is true, but SBAS is not supported by this device");
  }

  // Fix Service type, used when publishing fix status messages
  fix_status_service_ = sensor_msgs::msg::NavSatStatus::SERVICE_GPS
      + (enable_glonass_ ? 1 : 0) * sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS
      + (enable_beidou_ ? 1 : 0) * sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS
      + (enable_galileo_ ? 1 : 0) * sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

  //
  // NMEA Configuration
  //
  if (getRosBoolean(node_, "nmea.set")) {
    bool compat, consider;
    cfg_nmea_.version = ublox_msgs::msg::CfgNMEA::VERSION; // message version

    // Verify that parameters are set
    if (!getRosUint(node_, "nmea.version", cfg_nmea_.nmea_version)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.version must be set");
    }
    if (!getRosUint(node_, "nmea.num_sv", cfg_nmea_.num_sv)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
                "true, therefore nmea.num_sv must be set");
    }
    if (!getRosUint(node_, "nmea.sv_numbering", cfg_nmea_.sv_numbering)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.sv_numbering must be set");
    }
    if (!node_->get_parameter("nmea.compat", compat)) {
        throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.compat must be set");
    }
    if (!node_->get_parameter("nmea.consider", consider)) {
      throw std::runtime_error(std::string("Invalid settings: nmea.set is ") +
          "true, therefore nmea.consider must be set");
    }

    // set flags
    cfg_nmea_.flags = compat ? ublox_msgs::msg::CfgNMEA::FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= consider ? ublox_msgs::msg::CfgNMEA::FLAGS_CONSIDER : 0;
    cfg_nmea_.flags |= getRosBoolean(node_, "nmea.limit82") ? ublox_msgs::msg::CfgNMEA::FLAGS_LIMIT82 : 0;
    cfg_nmea_.flags |= getRosBoolean(node_, "nmea.high_prec") ? ublox_msgs::msg::CfgNMEA::FLAGS_HIGH_PREC : 0;
    // set filter
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.pos") ? ublox_msgs::msg::CfgNMEA::FILTER_POS : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.msk_pos") ? ublox_msgs::msg::CfgNMEA::FILTER_MSK_POS : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.time") ? ublox_msgs::msg::CfgNMEA::FILTER_TIME : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.date") ? ublox_msgs::msg::CfgNMEA::FILTER_DATE : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.gps_only") ? ublox_msgs::msg::CfgNMEA::FILTER_GPS_ONLY : 0;
    cfg_nmea_.filter |= getRosBoolean(node_, "nmea.filter.track") ? ublox_msgs::msg::CfgNMEA::FILTER_TRACK : 0;
    // set gnssToFilter
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.gps") ? ublox_msgs::msg::CfgNMEA::GNSS_TO_FILTER_GPS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.sbas") ? ublox_msgs::msg::CfgNMEA::GNSS_TO_FILTER_SBAS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.qzss") ? ublox_msgs::msg::CfgNMEA::GNSS_TO_FILTER_QZSS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.glonass") ? ublox_msgs::msg::CfgNMEA::GNSS_TO_FILTER_GLONASS : 0;
    cfg_nmea_.gnss_to_filter |= getRosBoolean(node_, "nmea.gnssToFilter.beidou") ? ublox_msgs::msg::CfgNMEA::GNSS_TO_FILTER_BEIDOU : 0;

    getRosUint(node_, "nmea.main_talker_id", cfg_nmea_.main_talker_id);
    getRosUint(node_, "nmea.gsv_talker_id", cfg_nmea_.gsv_talker_id);

    std::vector<uint8_t> bds_talker_id;
    getRosUint(node_, "nmea.bds_talker_id", bds_talker_id);
    cfg_nmea_.bds_talker_id[0] = bds_talker_id[0];
    cfg_nmea_.bds_talker_id[1] = bds_talker_id[1];
  }
}

bool UbloxFirmware8::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  if (clear_bbr_) {
    // clear flash memory
    if (!gps->clearBbr()) {
      RCLCPP_ERROR(node_->get_logger(), "u-blox failed to clear flash memory");
    }
  }

  gps->setSaveOnShutdown(save_on_shutdown_);

  //
  // Configure the GNSS, only if the configuration is different
  //
  // First, get the current GNSS configuration
  ublox_msgs::msg::CfgGNSS cfg_gnss;
  if (gps->poll(cfg_gnss)) {
    RCLCPP_DEBUG(node_->get_logger(), "Read GNSS config.");
    RCLCPP_DEBUG(node_->get_logger(), "Num. tracking channels in hardware: %i", cfg_gnss.num_trk_ch_hw);
    RCLCPP_DEBUG(node_->get_logger(), "Num. tracking channels to use: %i", cfg_gnss.num_trk_ch_use);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  // Then, check the configuration for each GNSS. If it is different, change it.
  bool correct = true;
  for (size_t i = 0; i < cfg_gnss.blocks.size(); i++) {  // NOLINT(modernize-loop-convert)
    ublox_msgs::msg::CfgGNSSBlock block = cfg_gnss.blocks[i];
    if (block.gnss_id == ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_GPS
        && enable_gps_ != (block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE) | enable_gps_;
      RCLCPP_DEBUG(node_->get_logger(), "GPS Configuration is different");
    } else if (block.gnss_id == ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_SBAS
               && getRosBoolean(node_, "gnss.sbas") != (block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE) | getRosBoolean(node_, "gnss.sbas");
      RCLCPP_DEBUG(node_->get_logger(), "SBAS Configuration is different");
    } else if (block.gnss_id == ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_GALILEO
               && enable_galileo_ != (block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE) | enable_galileo_;
      RCLCPP_DEBUG(node_->get_logger(), "Galileo GNSS Configuration is different");
    } else if (block.gnss_id == ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_BEIDOU
               && enable_beidou_ != (block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE) | enable_beidou_;
      RCLCPP_DEBUG(node_->get_logger(), "BeiDou Configuration is different");
    } else if (block.gnss_id == ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_IMES
               && enable_imes_ != (block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE) | enable_imes_;
    } else if (block.gnss_id == ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_QZSS
               && (enable_qzss_ != (block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE)
               || (enable_qzss_
               && qzss_sig_cfg_ != (block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_SIG_CFG_MASK)))) {
      RCLCPP_DEBUG(node_->get_logger(), "QZSS Configuration is different %u, %u",
                block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE,
                enable_qzss_);
      correct = false;
      RCLCPP_DEBUG(node_->get_logger(), "QZSS Configuration: %u", block.flags);
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE) | enable_qzss_;
      RCLCPP_DEBUG(node_->get_logger(), "QZSS Configuration: %u", cfg_gnss.blocks[i].flags);
      if (enable_qzss_) {
        // Only change sig cfg if enabling
        cfg_gnss.blocks[i].flags |= qzss_sig_cfg_;
      }
    } else if (block.gnss_id == ublox_msgs::msg::CfgGNSSBlock::GNSS_ID_GLONASS
               && enable_glonass_ != (block.flags & ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags =
          (cfg_gnss.blocks[i].flags & ~ublox_msgs::msg::CfgGNSSBlock::FLAGS_ENABLE) | enable_glonass_;
      RCLCPP_DEBUG(node_->get_logger(), "GLONASS Configuration is different");
    }
  }

  // If the GNSS is already configured correctly, do not re-configure GNSS
  // since this requires a cold reset
  if (correct) {
    RCLCPP_DEBUG(node_->get_logger(), "U-Blox GNSS configuration is correct. GNSS not re-configured.");
  } else if (!gps->configGnss(cfg_gnss, std::chrono::seconds(15))) {
    throw std::runtime_error(std::string("Failed to cold reset device ") +
                             "after configuring GNSS");
  }

  //
  // NMEA config
  //
  if (getRosBoolean(node_, "nmea.set") && !gps->configure(cfg_nmea_)) {
    throw std::runtime_error("Failed to configure NMEA");
  }

  return true;
}

void UbloxFirmware8::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to Nav PVT
  gps->subscribe<ublox_msgs::msg::NavPVT>(
    std::bind(&UbloxFirmware7Plus::callbackNavPvt, this, std::placeholders::_1), 1);

  // Subscribe to Nav SAT messages
  if (getRosBoolean(node_, "publish.nav.sat")) {
    gps->subscribe<ublox_msgs::msg::NavSAT>([this](const ublox_msgs::msg::NavSAT &m) { nav_sat_pub_->publish(m); },
                                       kNavSvInfoSubscribeRate);
  }

  // Subscribe to Mon HW
  if (getRosBoolean(node_, "publish.mon.hw")) {
    gps->subscribe<ublox_msgs::msg::MonHW>([this](const ublox_msgs::msg::MonHW &m) { mon_hw_pub_->publish(m); },
                                      1);
  }

  // Subscribe to RTCM messages
  if (getRosBoolean(node_, "publish.rxm.rtcm")) {
    gps->subscribe<ublox_msgs::msg::RxmRTCM>([this](const ublox_msgs::msg::RxmRTCM &m) { rxm_rtcm_pub_->publish(m); },
                                        1);
  }
}

}  // namespace ublox_node
