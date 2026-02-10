#include "geodetic_converter/UTM.h"

using namespace geo_converter;

UtmProjector::UtmProjector(sensor_msgs::msg::NavSatFix& origin) : m_isInNorthernHemisphere(true), m_origin(origin)
{
  GeographicLib::UTMUPS::Forward(m_origin.latitude, m_origin.longitude, m_zone,
                                 m_isInNorthernHemisphere, m_utmOrigin.x, m_utmOrigin.y);

}

geometry_msgs::msg::Pose2D UtmProjector::forward(const sensor_msgs::msg::NavSatFix& gps){

  geometry_msgs::msg::Pose2D utmXY;

  if(!m_isInNorthernHemisphere)
    std::cerr << "Is it Southern Hemisphere?" << std::endl;

  try {
    GeographicLib::UTMUPS::Forward(gps.latitude, gps.longitude, m_zone, m_isInNorthernHemisphere, utmXY.x, utmXY.y);

    // printf("utm Data: \n x: %.9f ,y: %.9f \n" , utmXY.x, utmXY.y);
    // printf("origin Data: \n x: %.9f ,y: %.9f \n" , m_utmOrigin.x, m_utmOrigin.y);

    utmXY.x = utmXY.x - m_utmOrigin.x;
    utmXY.y = utmXY.y - m_utmOrigin.y;

  } catch (GeographicLib::GeographicErr& e) {
    std::cerr << "GeographicLib FORWARD ERROR!" << std::endl;
  }

  return utmXY;
}

sensor_msgs::msg::NavSatFix UtmProjector::reverse(const geometry_msgs::msg::Pose2D& utm){

  sensor_msgs::msg::NavSatFix GpsRaw;
  geometry_msgs::msg::Pose2D utmXY; 

  if(!m_isInNorthernHemisphere)
    std::cerr << "Is it Southern Hemisphere?" << std::endl;

  try { 
    utmXY.x = utm.x + m_utmOrigin.x;
    utmXY.y = utm.y + m_utmOrigin.y;
    GeographicLib::UTMUPS::Reverse(m_zone, m_isInNorthernHemisphere, utmXY.x, utmXY.y, GpsRaw.latitude, GpsRaw.longitude);
  } catch (GeographicLib::GeographicErr& e) {
    std::cerr << "UTM Reverse Error" << std::endl;
  }

  return GpsRaw;
}
