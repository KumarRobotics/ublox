#ifndef UBLOX_GPS_UTILS_H
#define UBLOX_GPS_UTILS_H

#include <math.h>
#include <time.h>
#include "ublox_msgs/NavPVT.h"

extern "C" {
  #include "ublox_gps/mkgmtime.h"
}

/**
 * @brief Convert date/time to UTC time in seconds.
 */
template<typename NavPVT>
long toUtcSeconds(const NavPVT& msg) {
  // Create TM struct for mkgmtime
  struct tm time = {0};
  time.tm_year = msg.year - 1900; 
  time.tm_mon = msg.month - 1; 
  time.tm_mday = msg.day;
  time.tm_hour = msg.hour;   
  time.tm_min = msg.min; 
  time.tm_sec = msg.sec;
  // C++ STL doesn't have a mkgmtime (though other libraries do)
  // STL mktime converts date/time to seconds in local time 
  // A modified version of code external library is used for mkgmtime
  return mkgmtime(&time);
}

#endif
