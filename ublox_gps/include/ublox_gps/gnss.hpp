#ifndef UBLOX_GPS_GNSS_HPP
#define UBLOX_GPS_GNSS_HPP

#include <set>
#include <string>

namespace ublox_node {

class Gnss final {
public:
  Gnss() = default;
  ~Gnss() = default;

  Gnss(Gnss &&c) = delete;
  Gnss &operator=(Gnss &&c) = delete;
  Gnss(const Gnss &c) = delete;
  Gnss &operator=(const Gnss &c) = delete;

  void add(const std::string & gnss);
  bool isSupported(const std::string & gnss);

private:
  std::set<std::string> supported_;
};

}  // namespace ublox_node

#endif
