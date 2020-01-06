#ifndef UBLOX_GPS_RTCM_HPP
#define UBLOX_GPS_RTCM_HPP

#include <cstdint>

namespace ublox_gps {

struct Rtcm {
  //! ID of RTCM out message to configure.
  uint8_t id;
  //! Rate of RTCM out message.
  uint8_t rate;
};

}  // namespace ublox_gps

#endif
