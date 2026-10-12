#pragma once

#include <istream>
#include <string>
#include <vector>

#include "real_gps/core/Geometry.hh"

namespace real_gps::core
{
constexpr double kGpsEpochUnixSeconds = 315964800.0;
constexpr double kGpsWeekSeconds = 604800.0;

struct SatelliteAlmanac
{
  unsigned int prn{0};
  unsigned int health{0};
  int gpsWeek{0};
  double timeOfApplicabilitySeconds{0.0};
  double eccentricity{0.0};
  double inclinationRad{0.0};
  double rightAscensionRateRadPerSec{0.0};
  double sqrtSemiMajorAxis{0.0};
  double rightAscensionAtWeekRad{0.0};
  double argumentOfPerigeeRad{0.0};
  double meanAnomalyRad{0.0};
  double clockBiasSeconds{0.0};
  double clockDriftSecondsPerSecond{0.0};
};

double UnixToGpsSeconds(double unixSeconds, int gpsUtcLeapSeconds);
int ResolveGpsWeek(int encodedWeek, double referenceGpsSeconds);
std::vector<SatelliteAlmanac> ParseYumaAlmanac(std::istream &input,
                                            double referenceGpsSeconds);
std::vector<SatelliteAlmanac> LoadYumaAlmanac(const std::string &path,
                                           double referenceGpsSeconds);
// Explicitly synthetic 24-satellite constellation; never live orbit data.
std::vector<SatelliteAlmanac> SyntheticGpsConstellation(double referenceGpsSeconds);
Vec3 SatellitePositionEcef(const SatelliteAlmanac &satellite, double gpsSeconds);
}  // namespace real_gps::core
