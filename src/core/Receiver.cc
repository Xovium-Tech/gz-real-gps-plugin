#include "real_gps/core/Receiver.hh"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace real_gps::core
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDaySeconds = 86400.0;

bool Finite(const Vec3 &v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void CheckGeodetic(double lat, double lon, double alt)
{
  if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt) ||
      lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || alt < -100000.0)
    throw std::invalid_argument("Expected finite WGS84 coordinates in degrees/metres");
}

Vec3 ScaleCovariance(const DilutionOfPrecision &dop, const Vec3 &unitNoise, double sigma)
{
  // Cholesky factor of the position block preserves geometry-induced correlations.
  double lower[3][3]{};
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j <= i; ++j)
    {
      double value = dop.covariance[i][j];
      for (int k = 0; k < j; ++k)
        value -= lower[i][k] * lower[j][k];
      lower[i][j] = i == j ? std::sqrt(std::max(0.0, value)) :
          (lower[j][j] > 0.0 ? value / lower[j][j] : 0.0);
    }
  }
  return {sigma * lower[0][0] * unitNoise.x,
          sigma * (lower[1][0] * unitNoise.x + lower[1][1] * unitNoise.y),
          sigma * (lower[2][0] * unitNoise.x + lower[2][1] * unitNoise.y +
                   lower[2][2] * unitNoise.z)};
}
}

Quality ParseQuality(const std::string &name)
{
  std::string normalized;
  for (const unsigned char c : name)
    normalized += static_cast<char>(std::tolower(c));
  if (normalized == "excellent") return Quality::Excellent;
  if (normalized == "normal") return Quality::Normal;
  if (normalized == "poor") return Quality::Poor;
  if (normalized == "spoofing" || normalized == "gps_spoofing" || normalized == "gps-spoofing")
    return Quality::Spoofing;
  if (normalized == "denied" || normalized == "jamming") return Quality::Denied;
  if (normalized == "none") return Quality::None;
  throw std::invalid_argument("Unknown GPS quality: " + name);
}

std::string QualityName(Quality quality)
{
  switch (quality)
  {
    case Quality::Excellent: return "excellent";
    case Quality::Normal: return "normal";
    case Quality::Poor: return "poor";
    case Quality::Spoofing: return "spoofing";
    case Quality::Denied: return "denied";
    case Quality::None: return "none";
  }
  throw std::invalid_argument("Invalid GPS quality enum");
}

Receiver::Receiver(Config config)
  : config_(std::move(config)), scenario_(config_.scenario), generator_(config_.seed)
{
  QualityName(config_.quality);
  if (!std::isfinite(config_.epochUnixSeconds) || config_.epochUnixSeconds < kGpsEpochUnixSeconds ||
      config_.epochUnixSeconds > 100000000000.0 || config_.gpsUtcLeapSeconds < 0 ||
      config_.gpsUtcLeapSeconds > 100 || !std::isfinite(config_.elevationMaskDeg) ||
      config_.elevationMaskDeg < 0.0 || config_.elevationMaskDeg > 90.0 ||
      !std::isfinite(config_.updateRateHz) || config_.updateRateHz <= 0.0 ||
      !std::isfinite(config_.noiseCorrelationSeconds) || config_.noiseCorrelationSeconds <= 0.0 ||
      !std::isfinite(config_.spoofStartSeconds) || config_.spoofStartSeconds < 0.0 ||
      !std::isfinite(config_.spoofRampSeconds) || config_.spoofRampSeconds <= 0.0 ||
      !Finite(config_.spoofOffsetEnuM) || !std::isfinite(config_.maxAlmanacAgeDays) ||
      !std::isfinite(config_.reacquisitionSeconds) || config_.reacquisitionSeconds < 0.0)
    throw std::invalid_argument("Invalid GNSS receiver configuration");
  if (!scenario_.Empty() && config_.quality == Quality::Spoofing)
    throw std::invalid_argument("Use scenario spoofing_level with a reception quality, not quality=spoofing");
  for (const auto &zone : config_.zones)
  {
    CheckGeodetic(zone.latitudeDeg, zone.longitudeDeg, zone.minAltitudeM);
    QualityName(zone.quality);
    if (!scenario_.Empty() && zone.quality == Quality::Spoofing)
      throw std::invalid_argument("Scenario timelines cannot be combined with legacy spoofing zones");
    if (!std::isfinite(zone.radiusM) || zone.radiusM <= 0.0 ||
        !std::isfinite(zone.maxAltitudeM) || zone.maxAltitudeM < zone.minAltitudeM ||
        !std::isfinite(zone.elevationMaskDeg) || zone.elevationMaskDeg < -1.0 ||
        zone.elevationMaskDeg > 90.0)
      throw std::invalid_argument("Invalid GNSS degradation zone");
  }
  const double gps = UnixToGpsSeconds(config_.epochUnixSeconds, config_.gpsUtcLeapSeconds);
  if (config_.almanacPath.empty())
  {
    satellites_ = SyntheticGpsConstellation(gps);
    source_ = "synthetic_gps";
  }
  else
  {
    satellites_ = LoadYumaAlmanac(config_.almanacPath, gps);
    source_ = "yuma:" + config_.almanacPath;
    if (config_.maxAlmanacAgeDays > 0.0 &&
        AlmanacAgeSeconds(0.0) > config_.maxAlmanacAgeDays * kDaySeconds)
      throw std::invalid_argument("Yuma almanac is too far from simulation epoch; choose a matching "
                                  "epoch/file or explicitly disable max_almanac_age_days");
  }
}

void Receiver::Reset()
{
  generator_.seed(config_.seed);
  normal_.reset();
  positionNoise_ = {};
  velocityNoise_ = {};
  lastTime_ = std::numeric_limits<double>::quiet_NaN();
  lastMeasurement_ = {};
  hasFix_ = false;
  receptionLost_ = false;
  reacquireUntil_ = 0.0;
}

double Receiver::AlmanacAgeSeconds(double simulationSeconds) const
{
  if (config_.almanacPath.empty())
    return 0.0;
  const double gps = UnixToGpsSeconds(config_.epochUnixSeconds + simulationSeconds,
                                    config_.gpsUtcLeapSeconds);
  double oldest = 0.0;
  for (const auto &s : satellites_)
    oldest = std::max(oldest, std::abs(gps - s.gpsWeek * kGpsWeekSeconds -
                                      s.timeOfApplicabilitySeconds));
  return oldest;
}

Measurement Receiver::Sample(double simulationSeconds, double lat, double lon,
                             double alt, Vec3 trueVelocity)
{
  CheckGeodetic(lat, lon, alt);
  if (!std::isfinite(simulationSeconds) || simulationSeconds < 0.0 || !Finite(trueVelocity))
    throw std::invalid_argument("Invalid simulation time or velocity");
  if (std::isfinite(lastTime_) && simulationSeconds < lastTime_)
    Reset();
  const bool first = !std::isfinite(lastTime_);
  const double dt = first ? 0.0 : simulationSeconds - lastTime_;
  if (first || dt > 0.0)
  {
    const auto advance = [&](Vec3 &noise, double correlation)
    {
      const double a = first ? 0.0 : std::exp(-dt / correlation);
      const double b = std::sqrt(std::max(0.0, 1.0 - a * a));
      noise.x = a * noise.x + b * normal_(generator_);
      noise.y = a * noise.y + b * normal_(generator_);
      noise.z = a * noise.z + b * normal_(generator_);
    };
    advance(positionNoise_, config_.noiseCorrelationSeconds);
    advance(velocityNoise_, std::min(2.0, config_.noiseCorrelationSeconds));
  }
  lastTime_ = simulationSeconds;

  Measurement m;
  m.scenario = scenario_.Evaluate(simulationSeconds);
  m.quality = config_.quality;
  double mask = config_.elevationMaskDeg;
  // Last matching zone wins, allowing a small denied region within a poor region.
  for (const auto &zone : config_.zones)
  {
    if (alt >= zone.minAltitudeM && alt <= zone.maxAltitudeM &&
        SurfaceDistanceM(lat, lon, zone.latitudeDeg, zone.longitudeDeg) <= zone.radiusM)
    {
      m.quality = zone.quality;
      mask = zone.elevationMaskDeg >= 0.0 ? zone.elevationMaskDeg : config_.elevationMaskDeg;
    }
  }
  if (m.scenario.quality) m.quality = *m.scenario.quality;
  // Evaluate attack state independently of reception. Test diagnostics must not
  // claim an attack stopped just because denial or bad geometry removed the fix.
  if (!scenario_.Empty())
  {
    m.spoofing = m.scenario.spoofing;
  }
  else if (m.quality == Quality::Spoofing && simulationSeconds >= config_.spoofStartSeconds)
  {
    m.spoofing = true;
    const double u = std::clamp((simulationSeconds - config_.spoofStartSeconds) /
                               config_.spoofRampSeconds, 0.0, 1.0);
    const double fraction = u * u * (3.0 - 2.0 * u);
    const double derivative = (u > 0.0 && u < 1.0) ?
        6.0 * u * (1.0 - u) / config_.spoofRampSeconds : 0.0;
    m.scenario.spoofing = true;
    m.scenario.spoofingLevel = fraction;
    m.scenario.transitioning = u < 1.0;
    m.scenario.offsetEnuM = {config_.spoofOffsetEnuM.x * fraction,
                           config_.spoofOffsetEnuM.y * fraction,
                           config_.spoofOffsetEnuM.z * fraction};
    m.scenario.velocityOffsetEnuMps = {config_.spoofOffsetEnuM.x * derivative,
                                     config_.spoofOffsetEnuM.y * derivative,
                                     config_.spoofOffsetEnuM.z * derivative};
  }
  if (m.quality == Quality::None)
  {
    // An absent/disconnected receiver produces no packets, including no-fix
    // reports. Quality takes precedence over an event's output_enabled=true.
    m.scenario.outputEnabled = false;
    m.fixType = 0;
    m.status = "receiver_absent";
    m.latitudeDeg = lat;
    m.longitudeDeg = lon;
    m.altitudeM = alt;
    hasFix_ = false;
    receptionLost_ = true;
    reacquireUntil_ = 0.0;
    lastMeasurement_ = m;
    return m;
  }
  const double usedMask = std::min(90.0, mask + (m.quality == Quality::Poor ? 15.0 : 0.0));
  const Vec3 receiver = GeodeticToEcef(lat, lon, alt);
  const double gps = UnixToGpsSeconds(config_.epochUnixSeconds + simulationSeconds,
                                    config_.gpsUtcLeapSeconds);
  std::vector<Vec3> usedDirections;
  for (const auto &satellite : satellites_)
  {
    if (satellite.health != 0)
      continue;
    const Vec3 position = SatellitePositionEcef(satellite, gps);
    const Vec3 delta{position.x - receiver.x, position.y - receiver.y, position.z - receiver.z};
    const Vec3 local = EcefVectorToEnu(delta, lat, lon);
    const double norm = std::hypot(std::hypot(local.x, local.y), local.z);
    const double elevation = std::asin(std::clamp(local.z / norm, -1.0, 1.0)) * 180.0 / kPi;
    // Strict comparison also makes a 90-degree mask an intentional outage.
    if (elevation > mask)
      ++m.satellitesVisible;
    if (elevation > usedMask && m.quality != Quality::Denied)
      usedDirections.push_back({local.x / norm, local.y / norm, local.z / norm});
  }
  const bool stale = !config_.almanacPath.empty() && config_.maxAlmanacAgeDays > 0.0 &&
      AlmanacAgeSeconds(simulationSeconds) > config_.maxAlmanacAgeDays * kDaySeconds;
  if (stale || m.scenario.dropout)
    usedDirections.clear();
  m.satellitesUsed = static_cast<unsigned int>(usedDirections.size());
  const auto dop = ComputeDop(usedDirections);
  if (!dop.valid)
  {
    receptionLost_ = true;
    m.status = m.quality == Quality::Denied ? "denied" :
        m.scenario.dropout ? "dropout" : stale ? "stale_almanac" : "insufficient_geometry";
  }
  else
  {
    if (receptionLost_)
    {
      receptionLost_ = false;
      reacquireUntil_ = simulationSeconds + config_.reacquisitionSeconds;
    }
    m.reacquiring = simulationSeconds < reacquireUntil_;
    m.status = m.reacquiring ? "reacquiring" : "ok";
  }
  if (!dop.valid || m.reacquiring)
  {
    m.latitudeDeg = hasFix_ ? lastMeasurement_.latitudeDeg : lat;
    m.longitudeDeg = hasFix_ ? lastMeasurement_.longitudeDeg : lon;
    m.altitudeM = hasFix_ ? lastMeasurement_.altitudeM : alt;
    m.velocityEnuMps = hasFix_ ? lastMeasurement_.velocityEnuMps : Vec3{};
    lastMeasurement_ = m;
    return m;
  }

  double rangeSigma = 1.5;
  double rangeRateSigma = 0.08;
  if (m.quality == Quality::Excellent)
  {
    rangeSigma = 0.45;
    rangeRateSigma = 0.025;
  }
  else if (m.quality == Quality::Poor)
  {
    rangeSigma = 6.0;
    rangeRateSigma = 0.3;
  }
  rangeSigma *= m.scenario.noiseScale;
  rangeRateSigma *= m.scenario.noiseScale;
  Vec3 positionError = ScaleCovariance(dop, positionNoise_, rangeSigma);
  Vec3 velocityError = ScaleCovariance(dop, velocityNoise_, rangeRateSigma);
  positionError.x += m.scenario.offsetEnuM.x;
  positionError.y += m.scenario.offsetEnuM.y;
  positionError.z += m.scenario.offsetEnuM.z;
  velocityError.x += m.scenario.velocityOffsetEnuMps.x;
  velocityError.y += m.scenario.velocityOffsetEnuMps.y;
  velocityError.z += m.scenario.velocityOffsetEnuMps.z;
  const auto position = OffsetGeodetic({lat, lon, alt}, positionError);
  m.latitudeDeg = position.latitudeDeg;
  m.longitudeDeg = position.longitudeDeg;
  m.altitudeM = position.altitudeM;
  m.velocityEnuMps = {trueVelocity.x + velocityError.x, trueVelocity.y + velocityError.y,
                     trueVelocity.z + velocityError.z};
  // Keep ideal/noiseless test fixes usable by consumers that interpret zero
  // uncertainty as missing metadata. These reporting floors do not add noise.
  m.ephM = std::max(0.01, rangeSigma * dop.hdop);
  m.epvM = std::max(0.01, rangeSigma * dop.vdop);
  m.hdop = dop.hdop;
  m.vdop = dop.vdop;
  m.speedAccuracyMps = std::max(0.001, rangeRateSigma * dop.pdop);
  m.fixType = 3;
  m.valid = true;
  hasFix_ = true;
  lastMeasurement_ = m;
  return m;
}
}  // namespace real_gps::core
