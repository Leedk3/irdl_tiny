#include "geodetic_converter/ECEF.h"

#include <iostream>
#include <stdexcept>
#include <GeographicLib/Constants.hpp>

namespace geo_converter
{

EcefProjector::EcefProjector(const sensor_msgs::msg::NavSatFix& origin)
: m_origin(origin)
, m_ecefOrigin()
, m_geocentric(GeographicLib::Geocentric::WGS84())  // Use the WGS84 ellipsoid
{
  try
  {
    // Convert origin from geodetic (lat, lon, alt) to ECEF
    m_geocentric.Forward(
      m_origin.latitude, 
      m_origin.longitude,
      m_origin.altitude,
      m_ecefOrigin.x, 
      m_ecefOrigin.y, 
      m_ecefOrigin.z
    );
  }
  catch(const GeographicLib::GeographicErr& e)
  {
    std::cerr << "Error converting origin to ECEF: " << e.what() << std::endl;
  }
}

geometry_msgs::msg::Point EcefProjector::forward(const sensor_msgs::msg::NavSatFix& gps)
{
  geometry_msgs::msg::Point ecefRelative;

  try
  {
    // Convert this GPS point to absolute ECEF (X, Y, Z)
    double xAbs, yAbs, zAbs;
    m_geocentric.Forward(gps.latitude, gps.longitude, gps.altitude, xAbs, yAbs, zAbs);

    // Subtract the origin’s ECEF => relative coordinates
    ecefRelative.x = xAbs - m_ecefOrigin.x;
    ecefRelative.y = yAbs - m_ecefOrigin.y;
    ecefRelative.z = zAbs - m_ecefOrigin.z;
  }
  catch(const GeographicLib::GeographicErr& e)
  {
    std::cerr << "ECEF forward error: " << e.what() << std::endl;
  }

  return ecefRelative;
}

sensor_msgs::msg::NavSatFix EcefProjector::reverse(const geometry_msgs::msg::Point& ecef)
{
  sensor_msgs::msg::NavSatFix gps;

  try
  {
    // Add the origin’s ECEF back => absolute ECEF
    double xAbs = ecef.x + m_ecefOrigin.x;
    double yAbs = ecef.y + m_ecefOrigin.y;
    double zAbs = ecef.z + m_ecefOrigin.z;

    // Convert from absolute ECEF to geodetic
    m_geocentric.Reverse(xAbs, yAbs, zAbs, gps.latitude, gps.longitude, gps.altitude);
  }
  catch(const GeographicLib::GeographicErr& e)
  {
    std::cerr << "ECEF reverse error: " << e.what() << std::endl;
  }

  return gps;
}

}  // namespace geo_converter
