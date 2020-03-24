#ifndef UBLOX_GPS_FIX_DIAGNOSTIC_HPP
#define UBLOX_GPS_FIX_DIAGNOSTIC_HPP

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

namespace ublox_node {

//! Topic diagnostics for fix / fix_velocity messages
class FixDiagnostic {
public:
  /**
   * @brief Add a topic diagnostic to the diagnostic updater for fix topics.
   *
   * @details The minimum and maximum frequency are equal to the nav rate in Hz.
   * @param name the ROS topic
   * @param freq_tol the tolerance [%] for the topic frequency
   * @param freq_window the number of messages to use for diagnostic statistics
   * @param stamp_min the minimum allowed time delay
   */
  explicit FixDiagnostic(const std::string & name, double freq_tol, int freq_window,
                         double stamp_min, uint16_t nav_rate, uint16_t meas_rate,
                         std::shared_ptr<diagnostic_updater::Updater> updater) {
    const double target_freq = 1.0 / (meas_rate * 1e-3 * nav_rate); // Hz
    min_freq = target_freq;
    max_freq = target_freq;
    diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq,
                                                        freq_tol, freq_window);
    double stamp_max = meas_rate * 1e-3 * (1 + freq_tol);
    diagnostic_updater::TimeStampStatusParam time_param(stamp_min, stamp_max);
    diagnostic = std::make_shared<diagnostic_updater::TopicDiagnostic>(name,
                                                                       *updater,
                                                                       freq_param,
                                                                       time_param);
  }

  // Must not copy this struct (would confuse FrequencyStatusParam pointers)
  FixDiagnostic(FixDiagnostic &&c) = delete;
  FixDiagnostic &operator=(FixDiagnostic &&c) = delete;
  FixDiagnostic(const FixDiagnostic &c) = delete;
  FixDiagnostic &operator=(const FixDiagnostic &c) = delete;

  ~FixDiagnostic() = default;

  //! Topic frequency diagnostic updater
  std::shared_ptr<diagnostic_updater::TopicDiagnostic> diagnostic;

private:
  //! Minimum allow frequency of topic
  double min_freq;
  //! Maximum allow frequency of topic
  double max_freq;
};

}  // namespace ublox_node

#endif
