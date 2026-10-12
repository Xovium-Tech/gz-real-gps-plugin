#pragma once

#include <cmath>
#include <stdexcept>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/config.hh>

namespace real_gps::gzsim
{
using Spherical = gz::math::SphericalCoordinates;
#if GZ_MATH_MAJOR_VERSION >= 9
inline constexpr auto kLocal = Spherical::LOCAL;
#else
// Math 7 LOCAL has the inverse-heading bug; LOCAL2 is the corrected frame.
inline constexpr auto kLocal = Spherical::LOCAL2;
#endif

inline gz::math::Vector3d Geodetic(const Spherical &frame,
                                 const gz::math::Vector3d &position)
{
#if GZ_MATH_MAJOR_VERSION >= 9
  const auto out = frame.PositionTransform(gz::math::CoordinateVector3::Metric(position),
                                           kLocal, Spherical::SPHERICAL);
  if (!out) throw std::runtime_error("Cannot transform world coordinates");
  return {out->Lat()->Degree(), out->Lon()->Degree(), *out->Z()};
#else
  constexpr double degrees = 180.0 / 3.14159265358979323846;
  const auto out = frame.PositionTransform(position, kLocal, Spherical::SPHERICAL);
  return {out.X() * degrees, out.Y() * degrees, out.Z()};
#endif
}

inline gz::math::Vector3d VelocityEnu(const Spherical &frame,
                                    const gz::math::Vector3d &velocity,
                                    double latitudeDeg, double longitudeDeg)
{
  // Transform through ECEF so the ENU axes follow the aircraft, including on
  // long flights away from the world's geodetic origin.
#if GZ_MATH_MAJOR_VERSION >= 9
  const auto result = frame.VelocityTransform(gz::math::CoordinateVector3::Metric(velocity),
                                             kLocal, Spherical::ECEF);
  if (!result) throw std::runtime_error("Cannot transform world velocity");
  const auto v = result->AsMetricVector().value();
#else
  const auto v = frame.VelocityTransform(velocity, kLocal, Spherical::ECEF);
#endif
  constexpr double radians = 3.14159265358979323846 / 180.0;
  const double lat = latitudeDeg * radians, lon = longitudeDeg * radians;
  return {-std::sin(lon)*v.X() + std::cos(lon)*v.Y(),
    -std::sin(lat)*std::cos(lon)*v.X() - std::sin(lat)*std::sin(lon)*v.Y() + std::cos(lat)*v.Z(),
    std::cos(lat)*std::cos(lon)*v.X() + std::cos(lat)*std::sin(lon)*v.Y() + std::sin(lat)*v.Z()};
}
}
