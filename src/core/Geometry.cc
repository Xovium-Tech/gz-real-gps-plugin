#include "real_gps/core/Geometry.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace real_gps::core
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadians = kPi / 180.0;
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);
}

Vec3 GeodeticToEcef(double latitudeDeg, double longitudeDeg, double altitudeM)
{
  const double lat = latitudeDeg * kRadians;
  const double lon = longitudeDeg * kRadians;
  const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
  return {(n + altitudeM) * std::cos(lat) * std::cos(lon),
          (n + altitudeM) * std::cos(lat) * std::sin(lon),
          (n * (1.0 - kE2) + altitudeM) * std::sin(lat)};
}

Geodetic EcefToGeodetic(const Vec3 &ecef)
{
  const double p = std::hypot(ecef.x, ecef.y);
  if (p < 1e-9)
  {
    if (std::abs(ecef.z) < 1e-9)
      throw std::invalid_argument("ECEF origin has no geodetic coordinates");
    return {std::copysign(90.0, ecef.z), 0.0,
            std::abs(ecef.z) - kA * (1.0 - kF)};
  }
  double lat = std::atan2(ecef.z, p * (1.0 - kE2));
  for (int iteration = 0; iteration < 12; ++iteration)
  {
    const double sine = std::sin(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * sine * sine);
    const double next = std::atan2(ecef.z + kE2 * n * sine, p);
    if (std::abs(next - lat) < 1e-14)
    {
      lat = next;
      break;
    }
    lat = next;
  }
  const double sine = std::sin(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * sine * sine);
  // Stable near the poles and equator, without division by cos(latitude).
  const double alt = p * std::cos(lat) + ecef.z * sine -
                     n * (1.0 - kE2 * sine * sine);
  return {lat / kRadians, std::atan2(ecef.y, ecef.x) / kRadians, alt};
}

Vec3 EcefVectorToEnu(const Vec3 &v, double latitudeDeg, double longitudeDeg)
{
  const double lat = latitudeDeg * kRadians;
  const double lon = longitudeDeg * kRadians;
  const double sLat = std::sin(lat), cLat = std::cos(lat);
  const double sLon = std::sin(lon), cLon = std::cos(lon);
  return {-sLon * v.x + cLon * v.y,
          -sLat * cLon * v.x - sLat * sLon * v.y + cLat * v.z,
          cLat * cLon * v.x + cLat * sLon * v.y + sLat * v.z};
}

Vec3 EnuVectorToEcef(const Vec3 &v, double latitudeDeg, double longitudeDeg)
{
  const double lat = latitudeDeg * kRadians;
  const double lon = longitudeDeg * kRadians;
  const double sLat = std::sin(lat), cLat = std::cos(lat);
  const double sLon = std::sin(lon), cLon = std::cos(lon);
  return {-sLon * v.x - sLat * cLon * v.y + cLat * cLon * v.z,
          cLon * v.x - sLat * sLon * v.y + cLat * sLon * v.z,
          cLat * v.y + sLat * v.z};
}

Geodetic OffsetGeodetic(const Geodetic &origin, const Vec3 &offsetEnuM)
{
  const auto p = GeodeticToEcef(origin.latitudeDeg, origin.longitudeDeg, origin.altitudeM);
  const auto d = EnuVectorToEcef(offsetEnuM, origin.latitudeDeg, origin.longitudeDeg);
  return EcefToGeodetic({p.x + d.x, p.y + d.y, p.z + d.z});
}

double SurfaceDistanceM(double lat1, double lon1, double lat2, double lon2)
{
  const double dLat = (lat2 - lat1) * kRadians;
  const double dLon = (lon2 - lon1) * kRadians;
  const double sLat = std::sin(dLat / 2.0), sLon = std::sin(dLon / 2.0);
  const double a = sLat * sLat + std::cos(lat1 * kRadians) *
                   std::cos(lat2 * kRadians) * sLon * sLon;
  return 6371008.8 * 2.0 * std::asin(std::sqrt(std::clamp(a, 0.0, 1.0)));
}

DilutionOfPrecision ComputeDop(const std::vector<Vec3> &directions)
{
  DilutionOfPrecision result;
  if (directions.size() < 4)
    return result;
  double augmented[4][8]{};
  for (const auto &u : directions)
  {
    const double norm = std::hypot(std::hypot(u.x, u.y), u.z);
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-5)
      return result;
    const double row[4]{u.x, u.y, u.z, 1.0};
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        augmented[i][j] += row[i] * row[j];
  }
  for (int i = 0; i < 4; ++i)
    augmented[i][i + 4] = 1.0;
  for (int col = 0; col < 4; ++col)
  {
    int pivot = col;
    for (int row = col + 1; row < 4; ++row)
      if (std::abs(augmented[row][col]) > std::abs(augmented[pivot][col]))
        pivot = row;
    if (std::abs(augmented[pivot][col]) < 1e-10)
      return result;
    for (int j = 0; j < 8; ++j)
      std::swap(augmented[col][j], augmented[pivot][j]);
    const double scale = augmented[col][col];
    for (double &value : augmented[col])
      value /= scale;
    for (int row = 0; row < 4; ++row)
    {
      if (row == col)
        continue;
      const double factor = augmented[row][col];
      for (int j = 0; j < 8; ++j)
        augmented[row][j] -= factor * augmented[col][j];
    }
  }
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      result.covariance[i][j] = augmented[i][j + 4];
  for (int i = 0; i < 4; ++i)
    if (!std::isfinite(result.covariance[i][i]) || result.covariance[i][i] <= 0.0)
      return DilutionOfPrecision{};
  result.hdop = std::sqrt(result.covariance[0][0] + result.covariance[1][1]);
  result.vdop = std::sqrt(result.covariance[2][2]);
  result.pdop = std::hypot(result.hdop, result.vdop);
  result.valid = std::isfinite(result.pdop) && result.pdop < 9999.0;
  return result;
}
}  // namespace real_gps::core
