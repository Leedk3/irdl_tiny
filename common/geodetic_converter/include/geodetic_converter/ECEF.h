#pragma once

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <GeographicLib/Geocentric.hpp>

namespace geo_converter
{

class EcefProjector
{
public:
  /**
   * @brief Construct the projector with an origin in lat/lon/alt.
   *
   * The constructor converts this origin to ECEF (X0, Y0, Z0)
   * and stores it for relative calculations.
   */
  explicit EcefProjector(const sensor_msgs::msg::NavSatFix& origin);

  /**
   * @brief Convert from geodetic (lat, lon, alt) to relative ECEF (X, Y, Z).
   *
   * This subtracts the origin’s ECEF so that (0,0,0) is at the origin.
   */
  geometry_msgs::msg::Point forward(const sensor_msgs::msg::NavSatFix& gps);

  /**
   * @brief Convert from relative ECEF (X, Y, Z) back to geodetic (lat, lon, alt).
   *
   * This adds the origin’s ECEF before converting back to lat/lon/alt.
   */
  sensor_msgs::msg::NavSatFix reverse(const geometry_msgs::msg::Point& ecef);

private:
  // Store the origin (lat/lon/alt) if needed
  sensor_msgs::msg::NavSatFix m_origin;

  // Store the origin’s ECEF coordinates
  geometry_msgs::msg::Point m_ecefOrigin;

  // For doing the geodetic <-> ECEF transforms
  GeographicLib::Geocentric m_geocentric;
};

}  // namespace geo_converter
