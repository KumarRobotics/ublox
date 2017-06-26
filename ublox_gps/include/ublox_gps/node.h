#ifndef UBLOX_GPS_NODE_H
#define UBLOX_GPS_NODE_H

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <ublox_gps/gps.h>
#include <ublox_msgs/NavPOSLLH.h>
#include <ublox_msgs/NavVELNED.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavSTATUS.h>
#include <ublox_msgs/NavSOL.h>
#include <ublox_msgs/CfgGNSS_Block.h>
#include <ublox_msgs/CfgPRT.h>

namespace ublox_node {
/**
 * Abstract class representing a U-blox node.
 */
class UbloxNode {
 public:
  // Queue size for ROS publishers
  const static uint32_t kROSQueueSize = 1;
  /**
   * @brief Set the node handle parameter.
   */ 
  void setNh(boost::shared_ptr<ros::NodeHandle> _nh) {
    nh = _nh;
  }

  /**
   * @brief Publish a ROS message of type MessageT. Should be used to publish
   * all messages which are simply read from U-blox and published.
   * @param m the message to publish
   * @param topic the topic to publish the message on
   */
  template <typename MessageT>
  void publish(const MessageT& m, const std::string& topic);

  // Asynchronous IO objects
  boost::asio::io_service io_service;
  boost::shared_ptr<boost::asio::serial_port> serial_handle;
  boost::shared_ptr<boost::asio::ip::tcp::socket> tcp_handle;

  // ROS objects
  boost::shared_ptr<ros::NodeHandle> nh;
  boost::shared_ptr<diagnostic_updater::Updater> updater;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> freq_diag;

  ublox_gps::Gps gps;

  // Variables set from parameter server
  std::map<std::string, bool> enabled;
  std::string frame_id_, device_, dynamic_model_, fix_mode_;
  // Set from dynamic model & fix mode strings
  uint8_t dmodel, fmode;
  // UART in out protocol
  int baudrate_, uart_in_, uart_out_; 
  int rate_, meas_rate_, nav_rate_, rtcm_rate_;
  std::vector<int> rtcm_ids_;
  bool enable_gps_, enable_sbas_, enable_galileo_, enable_beidou_, enable_imes_; 
  bool enable_qzss_, enable_glonass_, enable_ppp_;
  int dr_limit_, qzss_sig_cfg_;
  int fix_status_service_;
  int sbas_usage_, max_sbas_;
 
 protected:
  /**
   * @brief Initialize the U-blox node. Configure the U-blox and subscribe to 
   * messages.
   */
  void initialize();

  /**
   * @brief Handle to send fix status to ROS diagnostics.
   */
  virtual void fixDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper& stat) = 0;

  /**
   * @brief Configure the U-Blox GNSS settings.
   */
  virtual void configureGnss() = 0;

  /* 
   * @brief Subscribe to ublox messages available on this version. 
   * (e.g. NavPVT, RxmRAWX)
   */
  virtual void subscribeVersion() = 0; 

 private:
  // how often (in seconds) to call poll messages
  const static double kPollDuration = 1.0;
  // Constants used for diagnostic frequency updater
  const static double kTolerance = 0.05;
  const static double kWindow = 10;
  const static double kTimeStampStatusMin = 0;

  /**
   * @brief Set class parameters from the ROS node parameters.
   * @return 0 if successful
   */
  int setParams();

  /**
   * @brief Poll messages from the U-Blox device.
   */
  void pollMessages(const ros::TimerEvent& event);

  /**
   * @brief Initialize the Serial / TCP IO.
   */
  void initializeIo();
  
  /**
   * @brief Initialize the diagnostic updater and add the fix diagnostic.
   */
  void initDiagnostics();

  /**
   * @brief Configure Ublox device based on requested settings.
   * @return true if configured succesfully
   */
  bool configureUblox();

  /*
   * @brief Subscribe to all requested U-Blox messages. Call subscribe version.
   */
  void subscribeAll();
};

/**
 * Represents a U-Blox node for firmware version 6.
 */
class UbloxNode6 : public UbloxNode {
 public:
  UbloxNode6(boost::shared_ptr<ros::NodeHandle> _nh);

  /*
   * @brief Publish a NavPOSLLh message & update the fix diagnostics & 
   * last known position.
   */
  void publishNavPOSLLH(const ublox_msgs::NavPOSLLH& m);
  
  /*
   * @brief Publish a NavVELNED message & update the last known velocity.
   */
  void publishNavVELNED(const ublox_msgs::NavVELNED& m);

  /*
   * @brief Publish a NavSOL message and update the number of SVs used for the 
   * fix.
   */
  void publishNavSOL(const ublox_msgs::NavSOL& m);
 protected:
  /**
   * @brief Updates fix diagnostic from NavPOSLLH, NavVELNED, and NavSOL 
   * messages.
   */
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  /**
   * @brief Prints a warning, GNSS configuration not available in this version.
   */
  void configureGnss();
    /**
   * @brief Subscribes to NavPVT, RxmRAW, and RxmSFRB messages. 
   */
  void subscribeVersion();

 private:
  // The last received navigation position
  ublox_msgs::NavPOSLLH last_nav_pos_;
  // The last received navigation velocity
  ublox_msgs::NavVELNED last_nav_vel_;
  // The last received num svs used
  int num_svs_used_;
  // The last Nav Sat fix based on last_nav_pos_
  sensor_msgs::NavSatFix fix;
  // The last Twist based on last_nav_vel_
  geometry_msgs::TwistWithCovarianceStamped velocity;
  // The last received nav status
  ublox_msgs::NavSTATUS status;
};

/**
 * Abstract class defining functions applicable to Firmware versions >=7.
 */
class UbloxNode7Plus : public UbloxNode {
 public:
  /**
   * Publish a NavPVT message. Also publishes Fix and Twist messages and
   * updates the fix diagnostics.
   */
  void publishNavPVT(const ublox_msgs::NavPVT& m);
  
 
 protected:
  void fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  /* Implement these in version specific classes */
  virtual void configureGnss() = 0;
  virtual void subscribeVersion() = 0; 
  ublox_msgs::NavPVT last_nav_pvt_; 
};

/**
 * Represents a U-Blox node for firmware version 7.
 */
class UbloxNode7 : public UbloxNode7Plus {
 public:
  UbloxNode7(boost::shared_ptr<ros::NodeHandle> _nh);

 protected:
  /**
   * @brief Subscribes to NavPVT, RxmRAW, and RxmSFRB messages. 
   */
  void subscribeVersion();
  /**
   * @brief Configures GNSS individually. Only configures GLONASS.
   */
  void configureGnss();
};

/**
 * Represents a U-Blox node for firmware version 8.
 */
class UbloxNode8 : public UbloxNode7Plus {
 public:
  UbloxNode8(boost::shared_ptr<ros::NodeHandle> _nh);

 protected:
  /**
   * @brief Subscribes to NavPVT, RxmRAWX, and RxmSFRBX messages. 
   */
  void subscribeVersion();
  /**
   * @brief Configures all GNSS systems in 1 message based on ROS params.
   */
  void configureGnss();
};

}

#endif