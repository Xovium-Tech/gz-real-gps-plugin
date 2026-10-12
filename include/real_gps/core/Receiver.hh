#pragma once

#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "real_gps/core/Almanac.hh"

namespace real_gps::core
{
enum class Quality { Excellent, Normal, Poor, Spoofing, Denied };
Quality ParseQuality(const std::string &name);
std::string QualityName(Quality quality);

struct DegradationZone
{
  double latitudeDeg{0.0};
  double longitudeDeg{0.0};
  double radiusM{100.0};
  double minAltitudeM{-10000.0};
  double maxAltitudeM{100000.0};
  Quality quality{Quality::Poor};
  // Negative uses the receiver mask. Otherwise overrides it inside this zone.
  double elevationMaskDeg{-1.0};
};

struct Config
{
  Quality quality{Quality::Normal};
  // 2026-01-01T00:00:00Z. Simulation time is added; no host clock dependency.
  double epochUnixSeconds{1767225600.0};
  int gpsUtcLeapSeconds{18};
  std::string almanacPath;
  double elevationMaskDeg{10.0};
  double updateRateHz{5.0};
  std::uint64_t seed{1};
  double noiseCorrelationSeconds{30.0};
  double spoofStartSeconds{30.0};
  double spoofRampSeconds{60.0};
  Vec3 spoofOffsetEnuM{200.0, 0.0, 0.0};
  // Applies only to files. A nonpositive value explicitly allows stale almanacs.
  double maxAlmanacAgeDays{14.0};
  std::vector<DegradationZone> zones;
};

struct Measurement
{
  double latitudeDeg{0.0};
  double longitudeDeg{0.0};
  double altitudeM{0.0};
  Vec3 velocityEnuMps;
  unsigned int satellitesVisible{0};
  unsigned int satellitesUsed{0};
  unsigned int fixType{1};  // MAVLink/PX4: 1 = no fix, 3 = 3D fix.
  double ephM{9999.0};
  double epvM{9999.0};
  double hdop{9999.0};
  double vdop{9999.0};
  double speedAccuracyMps{9999.0};
  bool valid{false};
  bool spoofing{false};
  Quality quality{Quality::Normal};
};

class Receiver
{
 public:
  explicit Receiver(Config config);
  Measurement Sample(double simulationSeconds, double latitudeDeg,
                     double longitudeDeg, double altitudeM, Vec3 velocityEnuMps);
  void Reset();
  const std::string &Source() const { return source_; }
  // Maximum absolute age across file records; zero for the synthetic constellation.
  double AlmanacAgeSeconds(double simulationSeconds) const;

 private:
  Config config_;
  std::vector<SatelliteAlmanac> satellites_;
  std::string source_;
  std::mt19937_64 generator_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  Vec3 positionNoise_;
  Vec3 velocityNoise_;
  double lastTime_{std::numeric_limits<double>::quiet_NaN()};
  Measurement lastMeasurement_;
  bool hasFix_{false};
};
}  // namespace real_gps::core
