#pragma once

#include <array>
#include <vector>

namespace real_gps::core
{
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Geodetic
{
  double latitudeDeg{0.0};
  double longitudeDeg{0.0};
  double altitudeM{0.0};
};

Vec3 GeodeticToEcef(double latitudeDeg, double longitudeDeg, double altitudeM);
Geodetic EcefToGeodetic(const Vec3 &ecef);
Vec3 EcefVectorToEnu(const Vec3 &vector, double latitudeDeg, double longitudeDeg);
Vec3 EnuVectorToEcef(const Vec3 &vector, double latitudeDeg, double longitudeDeg);
Geodetic OffsetGeodetic(const Geodetic &origin, const Vec3 &offsetEnuM);
double SurfaceDistanceM(double latitude1Deg, double longitude1Deg,
                        double latitude2Deg, double longitude2Deg);

struct DilutionOfPrecision
{
  bool valid{false};
  double hdop{9999.0};
  double vdop{9999.0};
  double pdop{9999.0};
  // ENU and receiver clock covariance for unit pseudorange variance.
  std::array<std::array<double, 4>, 4> covariance{};
};

// Directions point from the receiver to the satellites and must be unit ENU vectors.
DilutionOfPrecision ComputeDop(const std::vector<Vec3> &unitDirectionsEnu);
}  // namespace real_gps::core
