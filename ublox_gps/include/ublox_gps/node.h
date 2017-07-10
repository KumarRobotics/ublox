//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_NODE_H
#define UBLOX_GPS_NODE_H

// STL
#include <vector>
#include <set>
// Boost
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
// ROS includes
#include <ros/ros.h>
#include <ros/serialization.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
// ROS messages
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
// Other U-Blox package includes
#include <ublox_msgs/ublox_msgs.h>
// Ublox GPS includes
#include <ublox_gps/gps.h>
#include <ublox_gps/utils.h>

// This file declares the UbloxInterface which contains functions applicable
// to all Ublox firmware or hardware such as configuring the U-Blox and 
// subscribe to messages.
// It also declares UbloxNode which implements the UbloxInterface and all 
// functionality which applies to any U-Blox device, regardless of the
// firmware version or product type. It contains instances of UbloxInterface
// for the specific firmware and product version.
// UbloxFirmware is an abstract class which implements UbloxInterface and 
// functions generic to all firmware (such as the initializing the diagnostics).
// This file also declares subclasses of UbloxFirmware for firmware versions
// 6-8.
// Lastly, this file declares Product specific classes which also implement
// U-Blox interface, currently only a default class and a class for High 
// Precision GNSS devices.
namespace ublox_node {
// Queue size for ROS publishers
const static uint32_t kROSQueueSize = 1;
const static uint16_t kDefaultMeasPeriod = 250; 
// Subscribe Rate for U-Blox SV Info messages
const static uint32_t kNavSvInfoSubscribeRate = 20; // [Hz]
// Subscribe Rate of U-Blox msgs
const static uint32_t kSubscribeRate = 1; // [Hz]

const static char * const mode_names[] = {"Init", 
                                          "Fixed", 
                                          "Disabled",
                                          "Survey-In", 
                                          "Time"};
// Current mode of U-Blox
static enum {INIT, FIXED, DISABLED, SURVEY_IN, TIME} mode;
// ROS objects
boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> freq_diag;
boost::shared_ptr<ros::NodeHandle> nh;

ublox_gps::Gps gps;
// Which GNSS are supported by the device
std::set<std::string> supported;
// Whether or not to enable the given message subscriber
std::map<std::string, bool> enabled;
// The ROS frame ID of this GPS
std::string frame_id;
// The fix status service type, set based on the enabled GNSS
int fix_status_service;
// The measurement and navigation rate, see CfgRate message
int meas_rate, nav_rate;
// IDs of RTCM out messages to configure
std::vector<int> rtcm_ids;
// Rate of RTCM out messages [Hz]
int rtcm_rate;

/**
 * @brief Publish a ROS message of type MessageT. Should be used to publish
 * all messages which are simply read from U-blox and published.
 * @param m the message to publish
 * @param topic the topic to publish the message on
 */
template <typename MessageT>
void publish(const MessageT& m, const std::string& topic) {
  static ros::Publisher publisher =
      nh->advertise<MessageT>(topic, kROSQueueSize);
  publisher.publish(m);
}

/**
 * @gnss The string representing the GNSS. Refer MonVER message protocol.
 * i.e. GPS, GLO, GAL, BDS, QZSS, SBAS, IMES
 * @return true if the device supports the given GNSS
 */
bool supportsGnss(std::string gnss) {
  return supported.count(gnss) > 0;
}

/** 
 * @brief This interface is used to add functionality to the node. The main node 
 * class and firmware and hardware specific classes implement this interface. 
 * The main node class calls the functions in the interface all data members 
 * which implement this interface. This interface is generic and can be 
 * implemented for other features besides the hardware and firmware versions, 
 * e.g. such as for configuring an external device attached to the Ublox, etc. 
 */
class UbloxInterface {
 public:
  /**
   * @brief Get the ROS parameters.
   */
  virtual void getRosParams() = 0;
  
  /**
   * @brief Configure the U-Blox settings.
   * @returns true if configured correctly, false otherwise
   */
  virtual bool configureUblox() = 0;

  /**
   * @brief Initialize the diagnostics. If none, function should be empty.
   */
  virtual void initializeRosDiagnostics() = 0;

  /**
   * @brief Subscribe to U-Blox messages and publish as ROS messages.
   */
  virtual void subscribe() = 0;
};

/**
 * @brief This class represents U-Blox ROS node for *all* firmware and hardware 
 * versions. It gets the user parameters, configures the U-Blox 
 * device, subscribes to U-Blox messages, and configures the device hardware. 
 * Functionality specific to a given product or firmware version, etc. should 
 * NOT be implemented in this class. Instead, the user should extend U-Blox 
 * interface, then add a pointer the object to the xware_ vector.
 * The UbloxNode will then call the interface functions.
 */
class UbloxNode : public virtual UbloxInterface {
 public:
  // how often (in seconds) to call poll messages
  const static double kPollDuration = 1.0;
  // Constants used for diagnostic frequency updater
  const static float kDiagnosticPeriod = 0.2; // [s] 5Hz diagnostic period
  const static double kTolerance = 0.05;
  const static double kWindow = 10;
  const static double kTimeStampStatusMin = 0;

  /**
   * @brief Set the firmware object (add to xware_) based on the ublox_version 
   * param and call initialize.
   */
  UbloxNode();

  /**
   * @brief Get class parameters from the ROS node parameters.
   * @return 0 if successful
   */
  void getRosParams();

  /**
   * @brief Configure U-Blox device based on ROS parameters.
   * @return true if configured successfully
   */
  bool configureUblox();

  /*
   * @brief Subscribe to all requested U-Blox messages. Call subscribe version.
   */
  void subscribe();
  
  /**
   * @brief Initialize the diagnostic updater and add the fix diagnostic.
   */
  void initializeRosDiagnostics();

  /**
   * @brief Prints an INF message to the ROS console.
   */
  void printInf(const ublox_msgs::Inf &m, uint8_t id);

 private:
  /**
   * @brief Initialize the U-Blox node. Configure the U-Blox and subscribe to 
   * messages.
   */
  void initialize();

  /**
   * Process the MonVer message. Find the protocol version, hardware type and 
   * supported GNSS.
   */
  void processMonVer();

  /**
   * @brief Set the hardware member which is used for hardware specific 
   * configuration, subscribers, & diagnostics.
   * Currently only supports HPG. To add functionality for other hardware,
   * extend UbloxInterface and modify this function.
   * @param the product category, e.g. SPG, HPG, ADR, FTS.
   * @param for HPG/TIM products, this value is either REF or ROV, for other
   * products this string is empty
   */
  void setHardware(std::string product_category, std::string ref_rov);

  /**
   * @brief Poll messages from the U-Blox device.
   */
  void pollMessages(const ros::TimerEvent& event);

  /* Variables set from parameter server */
  std::string device_, dynamic_model_, fix_mode_;
  // Set from dynamic model & fix mode strings
  uint8_t dmodel_, fmode_;
  // UART baudrate and in/out protocol (see CfgPRT message for constants)
  int baudrate_, uart_in_, uart_out_; 
  int rate_;
  
  // If true: enable the GNSS
  bool enable_sbas_, enable_ppp_;
  int sbas_usage_, max_sbas_, dr_limit_;

  /** Determined From Mon VER */
  float protocol_version_;

  // The node will call the functions in these interfaces for each object
  // in the vector, this allows the user to easily add new features
  std::vector<boost::shared_ptr<UbloxInterface> > xware_;
};

/** 
 * @brief This abstract class is used to add functionality specific to a given
 * firmware version to the node.
 */
class UbloxFirmware : public virtual UbloxInterface {
 public:
  /**
   * @brief Add the fix diagnostics to the updater.
   */
  void initializeRosDiagnostics();

 protected:
  /**
   * @brief Handle to send fix status to ROS diagnostics.
   */
  virtual void fixDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper& stat) = 0;
};

/**
 * Represents a U-Blox node for firmware version 6.
 */
class UbloxFirmware6 : public UbloxFirmware {
 public:
  UbloxFirmware6();

  /**
   * @brief Sets the fix status service type to GPS.
   */
  void getRosParams();

  /**
   * @brief Prints a warning, GNSS configuration not available in this version.
   * @returns true if configured correctly, false otherwise
   */
  bool configureUblox();

  /**
   * @brief Subscribes to NavPVT, RxmRAW, and RxmSFRB messages. 
   */
  void subscribe();

 protected:
  /**
   * @brief Updates fix diagnostic from NavPOSLLH, NavVELNED, and NavSOL 
   * messages.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  
 private:
  /*
   * @brief Publish a NavPOSLLh message & update the fix diagnostics & 
   * last known position.
   */
  void publishNavPosLlh(const ublox_msgs::NavPOSLLH& m);
  
  /*
   * @brief Publish a NavVELNED message & update the last known velocity.
   */
  void publishNavVelNed(const ublox_msgs::NavVELNED& m);

  /*
   * @brief Publish a NavSOL message and update the number of SVs used for the 
   * fix.
   */
  void publishNavSol(const ublox_msgs::NavSOL& m);

  // The last received navigation position
  ublox_msgs::NavPOSLLH last_nav_pos_;
  // The last received navigation velocity
  ublox_msgs::NavVELNED last_nav_vel_;
  // The last received num svs used
  ublox_msgs::NavSOL last_nav_sol_;
  // The last Nav Sat fix based on last_nav_pos_
  sensor_msgs::NavSatFix fix_;
  // The last Twist based on last_nav_vel_
  geometry_msgs::TwistWithCovarianceStamped velocity_;
};

/**
 * Abstract class defining functions applicable to Firmware versions >=7.
 */
class UbloxFirmware7Plus : public UbloxFirmware {
 public:
  /**
   * Publish a NavPVT message. Also publishes Fix and Twist messages and
   * updates the fix diagnostics.
   */
  void publishNavPvt(const ublox_msgs::NavPVT& m);

 protected:

  /**
   * @brief Update the fix diagnostics from PVT message.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
 
  // The last received NavPVT message
  ublox_msgs::NavPVT last_nav_pvt_; 
  // Whether or not to enable the given GNSS
  bool enable_gps_, enable_glonass_, enable_qzss_, enable_sbas_;
  // The QZSS Signal configuration, see CfgGNSS message
  int qzss_sig_cfg_;
};

/**
 * Represents a U-Blox node for firmware version 7.
 */
class UbloxFirmware7 : public UbloxFirmware7Plus {
 public:
  UbloxFirmware7();

  /**
   * @brief Gets the GNSS params.
   */
  void getRosParams();
  
  /**
   * @brief Configures GNSS individually. Only configures GLONASS.
   */
  bool configureUblox();
  
  /**
   * @brief Subscribes to NavPVT, RxmRAW, and RxmSFRB messages. 
   */
  void subscribe();
};

/**
 * Represents a U-Blox node for firmware version 8.
 */
class UbloxFirmware8 : public UbloxFirmware7Plus {
 public:
  UbloxFirmware8();

  /**
   * @brief Gets the GNSS parameters.
   */
  void getRosParams();
  
  /**
   * @brief Configures all GNSS systems in 1 message based on ROS params and
   * configure DGNSS.
   */
  bool configureUblox();
  
  /**
   * @brief Subscribes to NavPVT messages. 
   */
  void subscribe();

 private:
  // Whether or not to enable the given GNSS
  bool enable_galileo_, enable_beidou_, enable_imes_;
};

/**
 * @brief Implements functions for High Precision GNSS Reference station.
 */
class UbloxHpgRef: public UbloxInterface {
 public:
  /**
   * @brief Gets the Reference Station GNSS parameters, e.g. tmode3.
   */
  void getRosParams();

  /**
   * @brief Configures Reference Station settings, e.g. TMODE3.
   * @returns true if configured correctly, false otherwise
   */
  bool configureUblox();

  /**
   * @brief Subscribes to Reference Station messages, e.g. NavSVIN.
   */
  void subscribe();

  /**
   * @brief Adds diagnostic updaters for Reference Station status, including
   * TMODE status.
   */
  void initializeRosDiagnostics();

  /**
   * @brief Publishes received Nav SVIN messages. When the survey in finishes, 
   * it changes the measurement & navigation rate to the user configured values 
   * and enables the user configured RTCM messages.
   */
  void publishNavSvIn(ublox_msgs::NavSVIN msg);

 protected:
  /**
   * @brief Update the High Precision Diagnostics, including the TMODE3 and 
   * the status of the survey-in if in survey-in mode or the RTCM messages if in
   * time mode.
   */
  void diagnosticUpdater(
    diagnostic_updater::DiagnosticStatusWrapper& stat);

  // The last received Nav SVIN message
  ublox_msgs::NavSVIN last_nav_svin_;

  // TMODE3 to set, e.g. disabled, survey-in, fixed
  int tmode3_;
  
  // TMODE3 = Fixed mode settings
  // True if coordinates are in LLA, false if ECEF
  bool lla_flag_; 
  // Antenna Reference Point Position [m]
  std::vector<float> arp_position_;
  // Antenna Reference Point Position - High Precision [0.1 mm]
  std::vector<float> arp_position_hp_;
  // Fixed Position Accuracy [m]
  float fixed_pos_acc_;
  
  // TMODE3 = Survey in settings
  // Measurement period used during Survey-In [s]
  int sv_in_min_dur_;
  // Survey in accuracy limit [m]
  float sv_in_acc_lim_;
};

/**
 * @brief Implements functions for High Precision GNSS Rover.
 */
class UbloxHpgRov: public UbloxInterface {
 public:
  /**
   * @brief Gets the High Precision GNSS rover parameters, e.g. DGNSS mode.
   */
  void getRosParams();

  /**
   * @brief Configures rover settings, e.g. DGNSS.
   * @returns true if configured correctly, false otherwise
   */
  bool configureUblox();

  /**
   * @brief Subscribes to Rover messages, e.g. NavRELPOSNED.
   */
  void subscribe();

  /**
   * @brief Adds diagnostic updaters for rover GNSS status, including
   * status of RTCM messages.
   */
  void initializeRosDiagnostics();

 protected:
  /**
   * @brief Update the High Precision Diagnostics the RTCM messages status.
   */
  void diagnosticUpdater(
    diagnostic_updater::DiagnosticStatusWrapper& stat);

  // Map of RTCM IDs and the last time the message was received
  std::map<int, ros::Time> last_received_rtcm_;

  // The DGNSS mode, see CfgDGNSS message for possible values
  int dgnss_mode_;
};


/**
 * @brief Implements functions for Time Sync products.
 */
class UbloxTim: public UbloxInterface {
  /**
   * @brief Gets the Time Sync parameters. Currently unimplemented.
   */
  void getRosParams() {}

  /**
   * @brief Configures Time Sync settings. Currently unimplemented.
   */
  bool configureUblox() {}

  /**
   * @brief Subscribes to Time Sync GNSS messages: currently RxmRAWX & RxmSFRBX.
   */
  void subscribe();

  /**
   * @brief Adds diagnostic updaters for Time Sync status. 
   * Currently unimplemented.
   */
  void initializeRosDiagnostics() {}
};

}

#endif