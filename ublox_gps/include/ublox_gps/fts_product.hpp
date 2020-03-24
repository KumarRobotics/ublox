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

#ifndef UBLOX_GPS_FTS_PRODUCT_HPP
#define UBLOX_GPS_FTS_PRODUCT_HPP

#include <memory>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for FTS products. Currently unimplemented.
 * @todo Unimplemented.
 */
class FtsProduct final : public virtual ComponentInterface {
  /**
   * @brief Get the FTS parameters.
   * @todo Currently unimplemented.
   */
  void getRosParams() override {
    // RCLCPP_WARN("Functionality specific to u-blox FTS devices is %s",
    //          "unimplemented. See FtsProduct class in node.hpp & node.cpp.");
  }

  /**
   * @brief Configure FTS settings.
   * @todo Currently unimplemented.
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override {
    (void)gps;
    return false;
  }

  /**
   * @brief Adds diagnostic updaters for FTS status.
   * @todo Currently unimplemented.
   */
  void initializeRosDiagnostics() override {}

  /**
   * @brief Subscribe to FTS messages.
   * @todo Currently unimplemented.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override {
    (void)gps;
  }
};

}  // namespace ublox_node

#endif
