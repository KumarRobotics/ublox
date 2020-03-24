#include <string>

#include <ublox_gps/gnss.hpp>

namespace ublox_node {

void Gnss::add(const std::string & gnss)
{
  supported_.insert(gnss);
}

bool Gnss::isSupported(const std::string & gnss)
{
  return supported_.count(gnss) > 0;
}

}  // namespace ublox_node
