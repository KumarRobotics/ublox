//==============================================================================
// Copyright (c) 2019, Peter Weissig, TU Chemnitz
// All rights reserved.
//
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
//
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

// This file is an addon from TUC-ProAut (https://github.com/TUC-ProAut/).
// It was created to log all data from the ublox to a logfile and to publish
// the data as ros messages. This is used by our group to also evaluate the
// measured data with the rtklib.


#ifndef UBLOX_GPS_RAW_DATA_PA_HPP
#define UBLOX_GPS_RAW_DATA_PA_HPP

// STL
#include <fstream>
#include <string>

// ROS includes
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <std_msgs/msg/u_int8_multi_array.hpp>

/**
 * @namespace ublox_node
 * This namespace is for the ROS u-blox node and handles anything regarding
 * ROS parameters, message passing, diagnostics, etc.
 */
namespace ublox_node {

/**
 * @brief Implements functions for raw data stream.
 */
class RawDataStreamPa final : public rclcpp::Node {
 public:

  /**
   * @brief Constructor.
   * Initialises variables and the nodehandle.
   */
  explicit RawDataStreamPa(bool is_ros_subscriber = false);

  /**
   * @brief Get the raw data stream parameters.
   */
  void getRosParams();

  /**
   * @brief Returns the if raw data streaming is enabled.
   */
  bool isEnabled();

  /**
   * @brief Initializes raw data streams
   * If storing to file is enabled, the filename is created and the
   * corresponding filedescriptor will be opened.
   * If publishing ros messages is enabled, an empty msg will be published.
   * (This will implicitly create the publisher)
   */
  void initialize();

  /**
   * @brief Callback function which handles raw data.
   * @param data the buffer of u-blox messages to process
   * @param size the size of the buffer
   */
  void ubloxCallback(const unsigned char* data,
                     std::size_t size);

 private:
  /**
   * @brief Callback function which handles raw data.
   * @param msg ros message
   */
  void msgCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  /**
   * @brief Converts a string into an uint8 multibyte array
   */
  std_msgs::msg::UInt8MultiArray str2uint8(const std::string & str);

  /**
   * @brief Publishes data stream as ros message
   * @param str raw data stream as string
   */
  void publishMsg(const std::string & str);

  /**
   * @brief Stores data to given file
   * @param str raw data stream as string
   */
  void saveToFile(const std::string & str);

  //! Directory name for storing raw data
  std::string file_dir_;
  //! Filename for storing raw data
  std::string file_name_;
  //! Handle for file access
  std::ofstream file_handle_;

  //! Flag for publishing raw data
  bool flag_publish_;

  //! Internal flag
  //! true : subscribing to ros messages and storing those to file
  //! false: publishing ros messages and/or storing to file
  bool is_ros_subscriber_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_data_stream_sub_;
};

}  // namespace ublox_node

#endif
