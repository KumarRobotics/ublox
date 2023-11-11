#ifndef UBLOX_GPS_HPG_REF_PRODUCT_FIRMWARE9_HPP
#define UBLOX_GPS_HPG_REF_PRODUCT_FIRMWARE9_HPP

#include <memory>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/nav_svin.hpp>

#include <ublox_gps/hpg_ref_product.hpp>
#include <ublox_gps/gps.hpp>
#include <ublox_gps/rtcm.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for High Precision GNSS Reference station
 * devices.
 */
class HpgRefProductFirmware9 final : public HpgRefProduct {
 public:

  explicit HpgRefProductFirmware9(uint16_t nav_rate, uint16_t meas_rate, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node);

  /**
   * @brief Configure the u-blox Reference Station settings.
   *
   * @details Configure the TMODE3 settings and sets the internal state based
   * on the TMODE3 status. If the TMODE3 is set to fixed, it will configure
   * the RTCM messages.
   * @return true if configured correctly, false otherwise
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override;

protected:
  /**
   * @brief Sets nav solution rate and configures RTCM
   *
   * @details Used to configure
   */
  bool setTimeMode(std::shared_ptr<ublox_gps::Gps> gps) override;

  /**
   * @brief Configures the measurement and navigation solution rates
   *
   * @details Populates the CfgVALSET message with rates for the measurement
   * and navigation solution rates before sending the config request.
   * @param meas_rate The elapsed time in milliseconds betwen GNSS measurements 
   * @param nav_rate The ratio between the number of measurements and 
   * the number of navigation solutions. e.g. a nav_rate of 4 and a meas_rate of 
   * 250ms would provide a nav solution at 1Hz.
   * @return true if configured correctly, false otherwise
   */
  bool configNavSolutionRate(uint16_t meas_rate, uint16_t nav_rate);

  /**
   * @brief Enable RTCM Messages
   *
   * @details Send a configuration to the reciver that enables the specific
   * rtcm messages individually.
   * @return true if configured correctly, false otherwise
   */
  bool configRTCM();

  /**
   * @brief Configures TMODE3
   *
   * @details Send a configuration to the reciver that sets TMODE3 to one of three modes:
   * 0 - Disabled
   * 1 - Survey-in
   * 2 - Fixed 
   * @param mode An enumerated value representing the desired TMODE3 mode.
   * @return true if configured correctly, false otherwise
   */
  bool configTMODE3(uint8_t mode);


  /**
   * @brief Initiate survey-in mode
   *
   * @details Sumultaneously set the minimum duration and accuracy of the
   * survey-in while also setting TMODE3 to survey-in.
   * @param sv_in_min_dur_ The minimum duration of the servey-in period in seconds.
   * @param sv_in_acc_lim_ The minimum accuracy required to complete the survey-in in mm.
   * @return true if configured correctly, false otherwise
   */
  bool configTmode3SurveyIn(uint32_t sv_in_min_dur_, uint32_t sv_in_acc_lim_);

  /**
   * @brief Set the TMODE settings to fixed.
   *
   * @details Sets the at the given antenna reference point (ARP) position in
   * either Latitude Longitude Altitude (LLA) or ECEF coordinates.
   * @param arp_position a vector of size 3 representing the ARP position in
   * ECEF coordinates [m] or LLA coordinates [deg]
   * @param arp_position_hp a vector of size 3 a vector of size 3 representing
   * the ARP position in ECEF coordinates [0.1 mm] or LLA coordinates
   * [deg * 1e-9]
   * @param lla_flag true if position is given in LAT/LON/ALT, false if ECEF
   * @param fixed_pos_acc Fixed position 3D accuracy [m]
   * @return true on ACK, false if settings are incorrect or on other conditions
   */
  bool configTmode3Fixed(bool lla_flag,
                         std::vector<double> arp_position,
                         std::vector<int8_t> arp_position_hp,
                         float fixed_pos_acc);

  /**
   * @brief Convert RTCM message IDs into Configuration Keys
   *
   * @details UBX configuration messages in UBX protocol versions <=23 have a
   * unique message ID for each RTCM output message configuration.
   * For UBX protocol versions >23, RTCM ouput messages are configured using
   * CfgVALSET messages and a key value pair.  This function maps the
   * appropriate message ID with its key.
   * @param msgID The message ID associated with the relevant RTCM message.
   * @return The relevant configuration key associated with the provided message ID,
   * zero if unable to match.
   */
  uint32_t rtcmID2Key(uint8_t msgID);
};

}  // namespace ublox_node

#endif  // UBLOX_GPS_HPG_REF_PRODUCT_FIRMWARE9_HPP
