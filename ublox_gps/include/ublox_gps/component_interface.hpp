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

#ifndef UBLOX_GPS_COMPONENT_INTERFACE_HPP
#define UBLOX_GPS_COMPONENT_INTERFACE_HPP

#include <memory>

#include <ublox_gps/gps.hpp>

// This file declares the ComponentInterface which acts as a high level
// interface for u-blox firmware, product categories, etc. It contains methods
// to configure the u-blox and subscribe to u-blox messages.
//

namespace ublox_node {

/**
 * @brief This interface is used to add functionality to the main node.
 *
 * @details This interface is generic and can be implemented for other features
 * besides the main node, hardware versions, and firmware versions.
 */
class ComponentInterface {
 public:
  /**
   * @brief Get the ROS parameters.
   * @throws std::runtime_error if a parameter is invalid or required
   * parameters are not set.
   */
  virtual void getRosParams() = 0;

  /**
   * @brief Configure the U-Blox settings.
   * @return true if configured correctly, false otherwise
   */
  virtual bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) = 0;

  /**
   * @brief Initialize the diagnostics.
   *
   * @details Function may be empty.
   */
  virtual void initializeRosDiagnostics() = 0;

  /**
   * @brief Subscribe to u-blox messages and publish to ROS topics.
   */
  virtual void subscribe(std::shared_ptr<ublox_gps::Gps> gps) = 0;
};

}  // namespace ublox_node

#endif
