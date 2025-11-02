#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <ublox_gps/hpg_dr_product.hpp>
#include <ublox_gps/adr_udr_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// Extract U-Blox 24-bit signed integers from a 32-bit data blob
// and cast into int32_t
//
static inline std::int32_t extract_int24(std::uint32_t data) {
    const std::uint8_t* b = reinterpret_cast<std::uint8_t*>(&data);
    return (std::int32_t)(
        (((std::uint32_t)b[2] << 24) & 0xFF000000) |
        (((std::uint32_t)b[1] << 16) & 0x00FF0000) |
        (((std::uint32_t)b[0] <<  8) & 0x0000FF00)
    ) / 256;
}

//
// U-Blox High Precision GNSS product with Dead Reckoning
// These appear to only have firmware version >= 8
//
HpgDrProduct::HpgDrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node)
  : use_adr_(false), nav_rate_(nav_rate), meas_rate_(meas_rate), frame_id_(frame_id), updater_(updater), rtcms_(rtcms), node_(node)
{
  if (getRosBoolean(node_, "publish.esf.meas")) {
    esf_meas_pub_ = node_->create_publisher<ublox_msgs::msg::EsfMEAS>("esfmeas", 1);
  }
  if (getRosBoolean(node_, "publish.nav.att")) {
    nav_att_pub_ = node_->create_publisher<ublox_msgs::msg::NavATT>("navatt", 1);
  }
  if (getRosBoolean(node_, "publish.nav.pvt")) {
    nav_pvt_pub_ = node_->create_publisher<ublox_msgs::msg::NavPVT>("navpvt", 1);
  }
  if (getRosBoolean(node_, "publish.esf.ins")) {
    esf_ins_pub_ = node_->create_publisher<ublox_msgs::msg::EsfINS>("esfins", 1);
  }
  if (getRosBoolean(node_, "publish.esf.raw")) {
    esf_raw_pub_ = node_->create_publisher<ublox_msgs::msg::EsfRAW>("esfraw", 1);
  }
  if (getRosBoolean(node_, "publish.esf.status")) {
    esf_status_pub_ = node_->create_publisher<ublox_msgs::msg::EsfSTATUS>("esfstatus", 1);
  }

  imu_meas_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("~/imu_meas", 1);
  imu_att_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("~/imu_att", 1);
  esf_ins_ros_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("~/veh_kinematics", 1);
  imu_raw_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("~/imu_raw", 1);
  fix_hp_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("~/fix_highprecision", 1);
  esf_diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/fusion_status", 1);

  // Perform any message metadata value setting we can do only once, including default values
  // This only improves performance a little, but removes duplcate code
  imu_meas_.header.frame_id = frame_id_;

  imu_raw_.header.frame_id = frame_id_;
  imu_raw_.orientation_covariance[0] = -1;
  imu_raw_.linear_acceleration_covariance[0] = -1;
  imu_raw_.angular_velocity_covariance[0] = -1;

  imu_att_.header.frame_id = frame_id_;
  imu_att_.linear_acceleration_covariance[0] = -1;  // signifies missing data
  imu_att_.angular_velocity_covariance[0] = -1;  // signifies missing data

  esf_ins_ros_.header.frame_id = frame_id_;
  esf_ins_ros_.linear_acceleration_covariance[0] = -1; // signifies missing data
  esf_ins_ros_.angular_velocity_covariance[0] = -1;  // signifies missing data

  fix_hp_.header.frame_id = frame_id_;
}

void HpgDrProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to ADR/UDR Navigation Attitude messages
  gps->subscribe<ublox_msgs::msg::NavATT>(std::bind(
    &HpgDrProduct::callbackNavATT, this, std::placeholders::_1), 1);

  gps->subscribe<ublox_msgs::msg::NavPVT>(std::bind(
    &HpgDrProduct::callbackNavPvt, this, std::placeholders::_1), 1);

  // ESF status for diagnostics
  gps->subscribe<ublox_msgs::msg::EsfSTATUS>(std::bind(
    &HpgDrProduct::callbackEsfStatus, this, std::placeholders::_1), 1);

  // Subscribe to High-Precision Geodetic Position messages
  gps->subscribe<ublox_msgs::msg::NavHPPOSLLH>(std::bind(
    &HpgDrProduct::callbackNavHpPosLlh, this, std::placeholders::_1), 1);

  // Subscribe to ADR/UDR Post-Processed IMU messages
  gps->subscribe<ublox_msgs::msg::EsfMEAS>(std::bind(
    &HpgDrProduct::callbackEsfMEAS, this, std::placeholders::_1), 1);

  // Subscribe to ADR/UDR Inertial Navigation System kinematics messages
  gps->subscribe<ublox_msgs::msg::EsfINS>(std::bind(
    &HpgDrProduct::callbackEsfIns, this, std::placeholders::_1), 1);

  // Subscribe to ADR/UDR Raw IMU messages
  gps->subscribe<ublox_msgs::msg::EsfRAW>(std::bind(
    &HpgDrProduct::callbackEsfRAW, this, std::placeholders::_1), 1);
}

void HpgDrProduct::getRosParams() {
  use_adr_ = getRosBoolean(node_, "use_adr");
  // Check the nav rate
  float nav_rate_hz = 1000.0 / (meas_rate_ * nav_rate_);
  if (nav_rate_hz != 1) {
    RCLCPP_WARN(node_->get_logger(), "ADR/UDR Nav Rate recommended to be 1 Hz");
  }
}

bool HpgDrProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  if (!gps->setUseAdr(use_adr_)) {
    throw std::runtime_error(std::string("Failed to ")
                             + (use_adr_ ? "enable" : "disable") + "use_adr");
  }
  return true;
}

void HpgDrProduct::callbackNavATT(const ublox_msgs::msg::NavATT &m) {
    imu_att_.header.stamp = node_->now();

    if (getRosBoolean(node_, "publish.nav.att")) {
      nav_att_pub_->publish(m);
    }

    constexpr double kNavAttScaleAndRadianConversion{1e-5 * M_PI / 180.0};

    // Transform U-Blox Euler angles to Quaternion and populate covariances
    const double roll = M_PI_2 - (static_cast<double>(m.roll) * kNavAttScaleAndRadianConversion);
    const double pitch = M_PI_2 - (static_cast<double>(m.pitch) * kNavAttScaleAndRadianConversion);
    const double heading = M_PI_2 - (static_cast<double>(m.heading) * kNavAttScaleAndRadianConversion);
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, heading);

    imu_att_.orientation.x = orientation[0];
    imu_att_.orientation.y = orientation[1];
    imu_att_.orientation.z = orientation[2];
    imu_att_.orientation.w = orientation[3];

    imu_att_.orientation_covariance[0] = std::pow(m.acc_roll * kNavAttScaleAndRadianConversion, 2);
    imu_att_.orientation_covariance[4] = std::pow(m.acc_pitch * kNavAttScaleAndRadianConversion, 2);
    imu_att_.orientation_covariance[8] = std::pow(m.acc_heading * kNavAttScaleAndRadianConversion, 2);

    imu_att_pub_->publish(imu_att_);
    last_nav_att_ = m;
}

void HpgDrProduct::callbackEsfIns(const ublox_msgs::msg::EsfINS &m) {
  esf_ins_ros_.header.stamp = node_->now();

  if (getRosBoolean(node_, "publish.esf.ins")) {
    esf_ins_pub_->publish(m);
  }

  // To avoid mutexing, let's just grab a copy of the last NavATT frame to match for data frame ID
  ublox_msgs::msg::NavATT temp_att = last_nav_att_;
  // If the last NavATT (orientation) message's data frame ID matches that of EsfINS, include the orientation from NavATT
  if (temp_att.i_tow == m.i_tow) {
    esf_ins_ros_.header.stamp.sec = last_itow_time_.second.sec;
    esf_ins_ros_.header.stamp.nanosec = last_itow_time_.second.nanosec;

    constexpr double kNavAttScaleAndRadianConversion{1e-5 * M_PI / 180.0};

    const double roll = M_PI_2 - (static_cast<double>(temp_att.roll) * kNavAttScaleAndRadianConversion);
    const double pitch = M_PI_2 - (static_cast<double>(temp_att.pitch) * kNavAttScaleAndRadianConversion);
    const double heading = M_PI_2 - (static_cast<double>(temp_att.heading) * kNavAttScaleAndRadianConversion);
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, heading);

    esf_ins_ros_.orientation.x = orientation[0];
    esf_ins_ros_.orientation.y = orientation[1];
    esf_ins_ros_.orientation.z = orientation[2];
    esf_ins_ros_.orientation.w = orientation[3];

    esf_ins_ros_.orientation_covariance[0] = std::pow(temp_att.acc_roll * kNavAttScaleAndRadianConversion, 2);
    esf_ins_ros_.orientation_covariance[4] = std::pow(temp_att.acc_pitch * kNavAttScaleAndRadianConversion, 2);
    esf_ins_ros_.orientation_covariance[8] = std::pow(temp_att.acc_heading * kNavAttScaleAndRadianConversion, 2);
  } else {  // No data available for this data frame
    esf_ins_ros_.orientation_covariance[0] = -1;
    esf_ins_ros_.orientation_covariance[4] = -1;
    esf_ins_ros_.orientation_covariance[8] = -1;
  }

  constexpr double kMilliGramsToNewtons{1e-6};
  constexpr double kScaleAndRadianConversion{1e-3 * M_PI / 180.0};

  esf_ins_ros_.angular_velocity.x = static_cast<double>(m.x_ang_rate) * kScaleAndRadianConversion;
  esf_ins_ros_.angular_velocity.y = static_cast<double>(m.y_ang_rate) * kScaleAndRadianConversion;
  esf_ins_ros_.angular_velocity.z = static_cast<double>(m.z_ang_rate) * kScaleAndRadianConversion;
  esf_ins_ros_.angular_velocity.x = static_cast<double>(m.x_accel) * kMilliGramsToNewtons;
  esf_ins_ros_.angular_velocity.y = static_cast<double>(m.y_accel) * kMilliGramsToNewtons;
  esf_ins_ros_.angular_velocity.z = static_cast<double>(m.z_accel) * kMilliGramsToNewtons;

  esf_ins_ros_pub_->publish(esf_ins_ros_);
}

void HpgDrProduct::callbackEsfStatus(const ublox_msgs::msg::EsfSTATUS &m) {
  std::uint8_t wheel_tick_status = m.reserved1[0] & 0xb00000011;
  std::uint8_t imu_align_status =  m.reserved1[0] & 0xb00011100;
  std::uint8_t ins_init_status =   m.reserved1[0] & 0xb01100000;
  std::uint8_t imu_init_status =   m.reserved1[1] & 0xb00000011;

  diagnostic_msgs::msg::KeyValue wt_status;
  wt_status.key = "wheel_tick_status";
  wt_status.value = wheel_tick_status == 2 ? "initialized" : (wheel_tick_status == 1 ? "initializing" : "off");
  diagnostic_msgs::msg::KeyValue imu_alg;
  imu_alg.key = "IMU_alignment_status";
  imu_alg.value = imu_align_status > 1 ? "initialized" : (imu_align_status == 1 ? "initializing" : "off");
  diagnostic_msgs::msg::KeyValue ins_ini;
  ins_ini.key = "INS_init_status";
  ins_ini.value = ins_init_status == 2 ? "initialized" : (ins_init_status == 1 ? "initializing" : "off");
  diagnostic_msgs::msg::KeyValue imu_ini;
  imu_ini.key = "IMU_init_status";
  imu_ini.value = imu_init_status == 2 ? "initialized" : (ins_init_status == 1 ? "initializing" : "off");
  diagnostic_msgs::msg::KeyValue fusion_mode;
  fusion_mode.key = "fusion_mode";
  fusion_mode.value = m.fusion_mode == 3 ? "disabled_fault" : (m.fusion_mode == 2 ? "suspended" : (m.fusion_mode == 1 ? "online" : "initializing"));
  diagnostic_msgs::msg::KeyValue num_sens;
  num_sens.key = "num_sensors";
  num_sens.value = std::to_string(m.num_sens);

  diagnostic_msgs::msg::DiagnosticStatus nav_diag;
  nav_diag.name = "NavigationDiagnostics";
  nav_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  nav_diag.hardware_id = "ublox ZED-F9";

  nav_diag.values.push_back(imu_alg);
  nav_diag.values.push_back(imu_ini);
  nav_diag.values.push_back(wt_status);
  nav_diag.values.push_back(ins_ini);
  nav_diag.values.push_back(fusion_mode);
  nav_diag.values.push_back(num_sens);
  nav_diag_pub_->publish(nav_diag);
}

//
// Decode the Processed IMU measurement output
//
void HpgDrProduct::callbackEsfMEAS(const ublox_msgs::msg::EsfMEAS &m) {
  imu_meas_.header.stamp = node_->now();

  if (getRosBoolean(node_, "publish.esf.meas")) {
    esf_meas_pub_->publish(m);
  }

  imu_meas_.orientation_covariance[0] = -1;
  imu_meas_.linear_acceleration_covariance[0] = -1;
  imu_meas_.angular_velocity_covariance[0] = -1;

  const std::vector<std::uint32_t> imu_data = m.data;
  for (const std::uint32_t datapoint : imu_data) {
    //grab the last six bits of data as the data type description field
    const std::uint8_t data_type = datapoint >> 24;
    // Interpret the first 24 bits as a signed integer
    const std::int32_t data_value = extract_int24(datapoint);

    switch (data_type) {
      case ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_X:
        imu_meas_.angular_velocity.x = static_cast<double>(data_value) * kConvertRadPerSec; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_X:
        imu_meas_.linear_acceleration.x = static_cast<double>(data_value) * kConvertMPerSecSq; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Y:
        imu_meas_.angular_velocity.y = static_cast<double>(data_value) * kConvertRadPerSec; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Y:
        imu_meas_.linear_acceleration.y = static_cast<double>(data_value) * kConvertMPerSecSq; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Z:
        imu_meas_.angular_velocity.z = static_cast<double>(data_value) * kConvertRadPerSec; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Z:
        imu_meas_.linear_acceleration.z = static_cast<double>(data_value) * kConvertMPerSecSq; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_TEMPERATURE:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_LEFT:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_RIGHT:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_LEFT:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_RIGHT:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SINGLE_TICK:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SPEED:
        break;  // Do nothing, just catch
      default:
        RCLCPP_INFO(node_->get_logger(), "Unknown IMU measurement, data_type: %u , data_value: %d", data_type, data_value);
    }
    imu_meas_pub_->publish(imu_meas_);
  }
}

//
// Decode the Raw IMU measurement output
//
void HpgDrProduct::callbackEsfRAW(const ublox_msgs::msg::EsfRAW &m) {
  if (getRosBoolean(node_, "publish.esf.raw")) {
    esf_raw_pub_->publish(m);
  }
  imu_raw_.header.stamp = node_->now();

  const std::vector<ublox_msgs::msg::EsfRAWBlock> imu_data_blocks = m.blocks;
  for (const ublox_msgs::msg::EsfRAWBlock &imu_data_entry : imu_data_blocks) {
    const std::uint32_t datapoint = imu_data_entry.data;

    //grab the last six bits of data as the data type description field
    const std::uint8_t data_type = datapoint >> 24;
    // Interpret the first 24 bits as a signed integer, and cast to double
    const std::int32_t data_value = extract_int24(datapoint);

    switch (data_type) {
      case ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_X:
        imu_raw_.angular_velocity.x = static_cast<double>(data_value) * kConvertRadPerSec; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_X:
        imu_raw_.linear_acceleration.x = static_cast<double>(data_value) * kConvertMPerSecSq; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Y:
        imu_raw_.angular_velocity.y = static_cast<double>(data_value) * kConvertRadPerSec; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Y:
        imu_raw_.linear_acceleration.y = static_cast<double>(data_value) * kConvertMPerSecSq; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_ANG_RATE_Z:
        imu_raw_.angular_velocity.z = static_cast<double>(data_value) * kConvertRadPerSec; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_ACCELEROMETER_Z:
        imu_raw_.linear_acceleration.z = static_cast<double>(data_value) * kConvertMPerSecSq; break;
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_GYRO_TEMPERATURE:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_LEFT:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_FRONT_RIGHT:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_LEFT:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_WHEEL_TICKS_REAR_RIGHT:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SINGLE_TICK:
        break;  // Do nothing, just catch
      case  ublox_msgs::msg::EsfMEAS::DATA_TYPE_SPEED:
        break;  // Do nothing, just catch
      default:
        RCLCPP_INFO(node_->get_logger(), "Unknown IMU measurement, data_type: %u , data_value: %d", data_type, data_value);
    }
  }
  imu_raw_pub_->publish(imu_raw_);
}

//
// Decode the High-Precision Geodetic Position message (HPPOSLLH) into NavSatFix
//
void HpgDrProduct::callbackNavHpPosLlh(const ublox_msgs::msg::NavHPPOSLLH& m) {
  // Do not publish invalid HPPOSLLH data
  if (m.flags != 0) {
    return;
  }
  fix_hp_.header.stamp = node_->now();  // Ideally, we should get a timestamp from the device

  if (last_nav_pvt_.fix_type >= ublox_msgs::msg::NavPVT::FIX_TYPE_2D) {
    fix_hp_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else {
    fix_hp_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }

  // Calculate the high-precision lat, lon, alt values
  fix_hp_.latitude = 1e-7 *  (static_cast<double>(m.lat) + (static_cast<double>(m.lat_hp) * 1e-2));
  fix_hp_.longitude = 1e-7 * (static_cast<double>(m.lon) + (static_cast<double>(m.lon_hp) * 1e-2));
  fix_hp_.altitude = 1e-3 * (static_cast<double>(m.height) + (static_cast<double>(m.height_hp) * 1e-1));

  // Populate the covariance data
  const double var_h = std::pow(m.h_acc / 1000.0, 2);
  const double var_v = std::pow(m.v_acc / 1000.0, 2);
  fix_hp_.position_covariance[0] = var_h;
  fix_hp_.position_covariance[4] = var_h;
  fix_hp_.position_covariance[8] = var_v;
  fix_hp_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix_hp_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix_hp_pub_->publish(fix_hp_);
}

//
// Use NavPVT for some message fusion diagnostics data
//
void HpgDrProduct::callbackNavPvt(const ublox_msgs::msg::NavPVT& m) {
  if (getRosBoolean(node_, "publish.nav.pvt")) {
    nav_pvt_pub_->publish(m);
  }

  // update the iTow timestamp
  uint8_t valid_time = m.VALID_DATE | m.VALID_TIME | m.VALID_FULLY_RESOLVED;
  if (((m.valid & valid_time) == valid_time) &&
      (m.flags2 & m.FLAGS2_CONFIRMED_AVAILABLE)) {
    // Use NavPVT timestamp since it is valid
    // The time in nanoseconds from the NavPVT message can be between -1e9 and 1e9
    //  The ros time uses only unsigned values, so a negative nano seconds must be
    //  converted to a positive value
    last_itow_time_.first = m.i_tow;
    if (m.nano < 0) {
      last_itow_time_.second.sec = toUtcSeconds(m) - 1;
      last_itow_time_.second.nanosec = static_cast<uint32_t>(m.nano + 1e9);
    }
    else {
      last_itow_time_.second.sec = toUtcSeconds(m);
      last_itow_time_.second.nanosec = static_cast<uint32_t>(m.nano);
    }
  }
}


}  // namespace ublox_node
