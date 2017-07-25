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
#include <ros/console.h>
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

// This file declares the ComponentInterface which acts as a high level 
// interface for u-blox firmware, product categories, etc. It contains methods
// to configure the u-blox and subscribe to u-blox messages.
//
// This file also declares UbloxNode which implements ComponentInterface and is 
// the main class and ros node. it implements functionality which applies to
// any u-blox device, regardless of the firmware version or product type. 
// The class is designed in compositional style; it contains ComponentInterfaces 
// which implement features specific to the device based on its firmware version
// and product category. UbloxNode calls the public methods of each component.
//
// This file declares UbloxFirmware is an abstract class which implements
// ComponentInterface and functions generic to all firmware (such as the 
// initializing the fix diagnostics). Subclasses of UbloxFirmware for firmware 
// versions 6-8 are also declared in this file.
//
// Lastly, this file declares classes for each product category which also 
// implement u-blox interface, currently only the class for High Precision 
// GNSS devices has been fully implemented and tested.

namespace ublox_node {

// Queue size for ROS publishers
const static uint32_t kROSQueueSize = 1;
const static uint16_t kDefaultMeasPeriod = 250; 
// Subscribe Rate for U-Blox SV Info messages
const static uint32_t kNavSvInfoSubscribeRate = 20; // [Hz]
// Subscribe Rate of U-Blox msgs
const static uint32_t kSubscribeRate = 1; // [Hz]

// ROS objects
boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> freq_diag;
boost::shared_ptr<ros::NodeHandle> nh;

// Handles communication with the U-Blox Device
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
uint16_t meas_rate, nav_rate;
// IDs and rates of RTCM out messages to configure, lengths must match
std::vector<uint8_t> rtcm_ids, rtcm_rates;

/**
 * @brief Determine dynamic model from human-readable string.
 * @param model One of the following (case-insensitive):
 *  - portable
 *  - stationary
 *  - pedestrian
 *  - automotive
 *  - sea
 *  - airborne1
 *  - airborne2
 *  - airborne4
 *  - wristwatch
 * @return DynamicModel
 * @throws std::runtime_error on invalid argument.
 */
uint8_t modelFromString(const std::string& model);

/**
 * @brief Determine fix mode from human-readable string.
 * @param mode One of the following (case-insensitive):
 *  - 2d
 *  - 3d
 *  - auto
 * @return FixMode
 * @throws std::runtime_error on invalid argument.
 */
uint8_t fixModeFromString(const std::string& mode);

/**
 * @brief Check that the parameter is above the minimum.
 * @param val the value to check
 * @param min the minimum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error if it is below the minimum
 */
template <typename V, typename T>
void checkMin(V val, T min, std::string name) {
  if(val < min) {
    std::stringstream oss;
    oss << "Invalid settings: " << name << " must be > " << min;
    throw std::runtime_error(oss.str());
  }
}

/**
 * @brief Check that the parameter is in the range.
 * @param val the value to check
 * @param min the minimum for this value
 * @param max the maximum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error if it is out of bounds
 */
template <typename V, typename T>
void checkRange(V val, T min, T max, std::string name) {
  if(val < min || val > max) {
    std::stringstream oss;
    oss << "Invalid settings: " << name << " must be in range [" << min << 
        ", " << max << "].";
    throw std::runtime_error(oss.str());
  }
}

/**
 * @brief Check that the elements of the vector are in the range.
 * @param val the vector to check
 * @param min the minimum for this value
 * @param max the maximum for this value
 * @param name the name of the parameter
 * @throws std::runtime_error value it is out of bounds
 */
template <typename V, typename T>
bool checkRange(std::vector<V> val, T min, T max, std::string name) {
  for(size_t i = 0; i < val.size(); i++)  {
    std::stringstream oss;
    oss << name << "[" << i << "]";
    checkRange(val[i], min, max, name);
  }
}

/**
 * @brief Get a unsigned integer value from the parameter server.
 * @param key the key to be used in the parameter server's dictionary
 * @param u storage for the retrieved value.
 * @throws std::runtime_error if the parameter is out of bounds
 * @returns true if found, false if not found.
 */
template <typename U>
bool getRosParam(const std::string& key, U &u) {
  int param;
  if (!nh->getParam(key, param)) return false;
  // Check the bounds
  U min = 0;
  U max = ~0;
  checkRange(param, min, max, key);
  // set the output
  u = (U) param;
  return true;
}

/**
 * @brief Get a unsigned integer value from the parameter server.
 * @param key the key to be used in the parameter server's dictionary
 * @param u storage for the retrieved value.
 * @param val value to use if the server doesn't contain this parameter.
 * @throws std::runtime_error if the parameter is out of bounds
 * @returns true if found, false if not found.
 */
template <typename U, typename V>
void getRosParam(const std::string& key, U &u, V default_val) {
  if(!getRosParam(key, u))
    u = default_val;
}

/**
 * @brief Get a unsigned integer vector from the parameter server.
 * @throws std::runtime_error if the parameter is out of bounds.
 * @returns true if found, false if not found.
 */
template <typename U>
bool getRosParam(const std::string& key, std::vector<U> &u) {
  std::vector<int> param;
  if (!nh->getParam(key, param)) return false;
  
  // Check the bounds
  U min = 0;
  U max = ~0;
  checkRange(param, min, max, key);

  // set the output
  u.insert(u.begin(), param.begin(), param.end());
  return true;
}

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
 * @param gnss The string representing the GNSS. Refer MonVER message protocol.
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
class ComponentInterface {
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

typedef boost::shared_ptr<ComponentInterface> ComponentPtr;

/**
 * @brief This class represents u-blox ROS node for *all* firmware and product 
 * versions. It loads the user parameters, configures the u-blox 
 * device, subscribes to u-blox messages, and configures the device hardware. 
 * Functionality specific to a given product or firmware version, etc. should 
 * NOT be implemented in this class. Instead, the user should add the 
 * functionality to the appropriate firmware or product class and if necessary, 
 * create a class which implements u-blox interface, then add a pointer to
 * to an instance of the class to the components vector.
 * The UbloxNode calls the public methods of ComponentInterface for each value 
 * in the components vector.
 */
class UbloxNode : public virtual ComponentInterface {
 public:
  // how often (in seconds) to call poll messages
  const static double kPollDuration = 1.0;
  // Constants used for diagnostic frequency updater
  const static float kDiagnosticPeriod = 0.2; // [s] 5Hz diagnostic period
  // Tolerance for Fix topic frequency as percentage of target frequency
  const static double kFixFreqTol = 0.15; 
  const static double kFixFreqWindow = 10;
  const static double kTimeStampStatusMin = 0;

  /**
   * @brief Set the firmware object (add to components) based on the 
   * ublox_version param and call initialize.
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
   * @brief Send a reset message the u-blox device & re-initialize the I/O.
   * @return true if reset was successful, false otherwise.
   */
  bool resetDevice();

  /**
   * Process the MonVer message. Find the protocol version, hardware type and 
   * supported GNSS. Add the appropriate firmware and product components.
   */
  void processMonVer();

  /**
   * @brief Add the interface for firmware specific configuration, subscribers, 
   * & diagnostics. This assumes the protocol_version_ has been set.
   */
  void addFirmwareInterface();

  /**
   * @brief Add the interface which is used for product category 
   * configuration, subscribers, & diagnostics.
   * @param the product category, i.e. SPG, HPG, ADR, UDR, TIM, or FTS.
   * @param for HPG/TIM products, this value is either REF or ROV, for other
   * products this string is empty
   */
  void addProductInterface(std::string product_category, 
                           std::string ref_rov = "");

  /**
   * @brief Poll messages from the U-Blox device.
   * @param event a timer indicating how often to poll the messages
   */
  void pollMessages(const ros::TimerEvent& event);

  /**
   * @brief Configure INF messages, call after subscribe.
   */
  void configureInf();

  // Used for diagnostic updater
  double min_freq, max_freq;

  // Variables set from parameter server
  std::string device_, dynamic_model_, fix_mode_;
  // Set from dynamic model & fix mode strings
  uint8_t dmodel_, fmode_;
  // UART1 baudrate and in/out protocol (see CfgPRT message for constants)
  uint32_t baudrate_;
  uint16_t uart_in_, uart_out_; 
  double rate_;
  // User-defined Datum (only used if the set_dat param is true)
  bool set_dat_;
  ublox_msgs::CfgDAT cfg_dat_;
  // If true: enable the GNSS
  bool enable_sbas_, enable_ppp_;
  uint8_t sbas_usage_, max_sbas_, dr_limit_;

  // Determined From Mon VER
  float protocol_version_;

  // The node will call the functions in these interfaces for each object
  // in the vector, this allows the user to easily add new features
  std::vector<boost::shared_ptr<ComponentInterface> > components_;
};

/** 
 * @brief This abstract class is used to add functionality specific to a given
 * firmware version to the node.
 */
class UbloxFirmware : public virtual ComponentInterface {
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
 * U-Blox functionality for firmware version 6.
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
   * @param m the message to publish
   */
  void publishNavPosLlh(const ublox_msgs::NavPOSLLH& m);
  
  /*
   * @brief Publish a NavVELNED message & update the last known velocity.
   * @param m the message to publish
   */
  void publishNavVelNed(const ublox_msgs::NavVELNED& m);

  /*
   * @brief Publish a NavSOL message and update the number of SVs used for the 
   * fix.
   * @param m the message to publish
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

  // Used to configure NMEA (if set_nmea_), filled with ros params
  ublox_msgs::CfgNMEA6 cfg_nmea_;
  bool set_nmea_;
};

/**
 * Abstract class defining functions applicable to Firmware versions >=7.
 */
template<typename NavPVT>
class UbloxFirmware7Plus : public UbloxFirmware {
 public:
  /**
   * Publish a NavPVT message. Also publishes Fix and Twist messages and
   * updates the fix diagnostics.
   * @param m the message to publish
   */
  // template<typename NavPVT>
  void publishNavPvt(const NavPVT& m) {
    static ros::Publisher publisher =
        nh->advertise<NavPVT>("navpvt", kROSQueueSize);
    publisher.publish(m);

    /** Fix message */
    static ros::Publisher fixPublisher =
        nh->advertise<sensor_msgs::NavSatFix>("fix", kROSQueueSize);
    // timestamp
    sensor_msgs::NavSatFix fix;
    fix.header.stamp.sec = toUtcSeconds(m);
    fix.header.stamp.nsec = m.nano;

    bool fixOk = m.flags & m.FLAGS_GNSS_FIX_OK;
    uint8_t cpSoln = m.flags & m.CARRIER_PHASE_FIXED;

    fix.header.frame_id = frame_id;
    fix.latitude = m.lat * 1e-7; // to deg
    fix.longitude = m.lon * 1e-7; // to deg
    fix.altitude = m.height * 1e-3; // to [m]
    if (fixOk && m.fixType >= m.FIX_TYPE_2D) {
      fix.status.status = fix.status.STATUS_FIX;
      if(cpSoln == m.CARRIER_PHASE_FIXED)
        fix.status.status = fix.status.STATUS_GBAS_FIX;
    }
    else {
      fix.status.status = fix.status.STATUS_NO_FIX;
    }

    const double varH = pow(m.hAcc / 1000.0, 2); // to [m^2]
    const double varV = pow(m.vAcc / 1000.0, 2); // to [m^2]
    fix.position_covariance[0] = varH;
    fix.position_covariance[4] = varH;
    fix.position_covariance[8] = varV;
    fix.position_covariance_type =
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    fix.status.service = fix_status_service;
    fixPublisher.publish(fix);

    /** Fix Velocity */
    static ros::Publisher velocityPublisher =
        nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity",
                                                                 kROSQueueSize);
    geometry_msgs::TwistWithCovarianceStamped velocity;
    velocity.header.stamp = fix.header.stamp;
    velocity.header.frame_id = frame_id;

    // convert to XYZ linear velocity [m/s] in ENU
    velocity.twist.twist.linear.x = m.velE * 1e-3;
    velocity.twist.twist.linear.y = m.velN * 1e-3;
    velocity.twist.twist.linear.z = -m.velD * 1e-3;

    const double covSpeed = pow(m.sAcc * 1e-3, 2);

    const int cols = 6;
    velocity.twist.covariance[cols * 0 + 0] = covSpeed;
    velocity.twist.covariance[cols * 1 + 1] = covSpeed;
    velocity.twist.covariance[cols * 2 + 2] = covSpeed;
    velocity.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

    velocityPublisher.publish(velocity);

    /** Update diagnostics **/
    last_nav_pvt_ = m;
    freq_diag->tick(fix.header.stamp);
    updater->update();
  }

 protected:

  /**
   * @brief Update the fix diagnostics from PVT message.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    //  check the last message, convert to diagnostic
    if (last_nav_pvt_.fixType == 
        ublox_msgs::NavPVT::FIX_TYPE_DEAD_RECKONING_ONLY) {
      stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.message = "Dead reckoning only";
    } else if (last_nav_pvt_.fixType == ublox_msgs::NavPVT::FIX_TYPE_2D) {
      stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.message = "2D fix";
    } else if (last_nav_pvt_.fixType == ublox_msgs::NavPVT::FIX_TYPE_3D) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "3D fix";
    } else if (last_nav_pvt_.fixType ==
               ublox_msgs::NavPVT::FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "GPS and dead reckoning combined";
    } else if (last_nav_pvt_.fixType == 
               ublox_msgs::NavPVT::FIX_TYPE_TIME_ONLY) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Time only fix";
    }

    // If fix not ok (w/in DOP & Accuracy Masks), raise the diagnostic level
    if (!(last_nav_pvt_.flags & ublox_msgs::NavPVT::FLAGS_GNSS_FIX_OK)) {
      stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.message += ", fix not ok";
    } 
    // Raise diagnostic level to error if no fix
    if (last_nav_pvt_.fixType == ublox_msgs::NavPVT::FIX_TYPE_NO_FIX) {
      stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.message = "No fix";
    }

    //  append last fix position
    stat.add("iTOW [ms]", last_nav_pvt_.iTOW);
    stat.add("Latitude [deg]", last_nav_pvt_.lat * 1e-7);
    stat.add("Longitude [deg]", last_nav_pvt_.lon * 1e-7);
    stat.add("Altitude [m]", last_nav_pvt_.height * 1e-3);
    stat.add("Height above MSL [m]", last_nav_pvt_.hMSL * 1e-3);
    stat.add("Horizontal Accuracy [m]", last_nav_pvt_.hAcc * 1e-3);
    stat.add("Vertical Accuracy [m]", last_nav_pvt_.vAcc * 1e-3);
    stat.add("# SVs used", (int)last_nav_pvt_.numSV);
  }
 
  // The last received NavPVT message
  NavPVT last_nav_pvt_; 
  // Whether or not to enable the given GNSS
  bool enable_gps_, enable_glonass_, enable_qzss_, enable_sbas_;
  // The QZSS Signal configuration, see CfgGNSS message
  uint32_t qzss_sig_cfg_;
};

/**
 * U-Blox functionality for firmware version 7.
 */
class UbloxFirmware7 : public UbloxFirmware7Plus<ublox_msgs::NavPVT7> {
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
  
  private:
    // Used to configure NMEA (if set_nmea_), filled with ros params
    ublox_msgs::CfgNMEA7 cfg_nmea_;
    bool set_nmea_;
};

/**
 *  U-Blox functionality for firmware version 8.
 */
class UbloxFirmware8 : public UbloxFirmware7Plus<ublox_msgs::NavPVT> {
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
  // Type of device reset, only used if GNSS configuration is changed
  // see CfgRST message for constants
  uint8_t reset_mode_;
  // Used to configure NMEA (if set_nmea_), filled with ros params
  ublox_msgs::CfgNMEA cfg_nmea_;
  bool set_nmea_;
};

/**
 * @brief Interface for Automotive Dead Reckoning (ADR) and Untethered
 * Dead Reckoning (UDR) Devices.
 */
class UbloxAdrUdr: public virtual ComponentInterface {
 public:
  /**
   * @brief Gets the ADR/UDR parameters, e.g. useAdr.
   */
  void getRosParams();

  /**
   * @brief Configures ADR/UDR settings, e.g. useAdr.
   * @returns true if configured correctly, false otherwise
   */
  bool configureUblox();

  /**
   * @brief Subscribes to ADR/UDR messages, e.g. NavATT, Esf and HNR messages
   */
  void subscribe();

  /**
   * @brief Does nothing.
   */
  void initializeRosDiagnostics() {
    ROS_WARN("ROS Diagnostics specific to U-Blox ADR/UDR devices is %s",
             "unimplemented. See UbloxAdrUdr class in node.h & node.cpp.");
  }

  // Whether or not to enable dead reckoning
  bool use_adr_;
};

/**
 * @brief Implements functions for FTS products. Currently unimplemented. TODO
 */
class UbloxFts: public virtual ComponentInterface {
  /**
   * @brief Gets the FTS parameters. Currently unimplemented.
   */
  void getRosParams() {
    ROS_WARN("Functionality specific to U-Blox FTS devices is %s",
             "unimplemented. See UbloxFts class in node.h & node.cpp.");
  }

  /**
   * @brief Configures FTS settings. Currently unimplemented.
   */
  bool configureUblox() {}

  /**
   * @brief Subscribes to FTS GNSS messages.
   */
  void subscribe() {}

  /**
   * @brief Adds diagnostic updaters for FTS status. 
   */
  void initializeRosDiagnostics() {}
};

/**
 * @brief Implements functions for High Precision GNSS Reference station.
 */
class UbloxHpgRef: public virtual ComponentInterface {
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
   * @param m the message to publish
   */
  void publishNavSvIn(ublox_msgs::NavSVIN m);

 protected:
  /**
   * @brief Update the TMODE3 diagnostics and the status of the survey-in if in
   * survey-in mode or the RTCM messages if in time mode.
   */
  void tmode3Diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Set the mode to time mode (internal state variable) and configure 
   * the RTCM messages and measurement and nav rate.
   */
  bool setTimeMode();

  // The last received Nav SVIN message
  ublox_msgs::NavSVIN last_nav_svin_;

  // TMODE3 to set, e.g. disabled, survey-in, fixed
  uint8_t tmode3_;
  
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
  // If true, will reset the survey-in, if false, will only reset if 
  // there's no fix and survey in is disabled
  bool svin_reset_;
  // Measurement period used during Survey-In [s]
  uint32_t sv_in_min_dur_;
  // Survey in accuracy limit [m]
  float sv_in_acc_lim_;

  // Current mode of U-Blox
  enum {INIT, FIXED, DISABLED, SURVEY_IN, TIME} mode_;
};

/**
 * @brief Implements functions for High Precision GNSS Rover.
 */
class UbloxHpgRov: public virtual ComponentInterface {
 public:
  // Constants for diagnostic updater
  const static double kRtcmFreqMin = 1;
  const static double kRtcmFreqMax = 10;
  const static double kRtcmFreqTol = 0.1;
  const static int kRtcmFreqWindow = 25;
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
   * @brief Update the rover diagnostics, including the carrier phase solution 
   * status (float or fixed).
   */
  void carrierPhaseDiagnostics(
      diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Publish received NavRELPOSNED, save the last received message, and
   * update the rover diagnostics.
   */
  void publishNavRelPosNed(const ublox_msgs::NavRELPOSNED &m);


  ublox_msgs::NavRELPOSNED last_rel_pos_;

  // For RTCM frequency diagnostic updater
  double rtcm_freq_min, rtcm_freq_max;

  // The DGNSS mode, see CfgDGNSS message for possible values
  uint8_t dgnss_mode_;

  boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freq_rtcm_;
};

/**
 * @brief Implements functions for Time Sync products.
 */
class UbloxTim: public virtual ComponentInterface {
  /**
   * @brief Gets the Time Sync parameters. Currently unimplemented.
   */
  void getRosParams() {
    ROS_WARN("Functionality specific to U-Blox TIM devices is only %s",
             "partially implemented. See UbloxTim class in node.h & node.cpp.");
  }

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