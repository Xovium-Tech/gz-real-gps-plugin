#include "real_gps/core/Receiver.hh"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace
{
using namespace real_gps::core;

void Check(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

void Near(double actual, double expected, double tolerance, const std::string &message)
{
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
    throw std::runtime_error(message + ": got " + std::to_string(actual) +
                             ", expected " + std::to_string(expected));
}

void Invalid(const std::function<void()> &action, const std::string &message)
{
  try { action(); }
  catch (const std::invalid_argument &) { return; }
  throw std::runtime_error("Expected invalid_argument: " + message);
}

void Geometry()
{
  const auto equator = GeodeticToEcef(0, 0, 0);
  Near(equator.x, 6378137.0, 1e-6, "WGS84 equatorial radius");
  Near(equator.y, 0, 1e-6, "Equator Y");
  Near(equator.z, 0, 1e-6, "Equator Z");
  const auto pole = EcefToGeodetic({0, 0, 6356752.314245179});
  Near(pole.latitudeDeg, 90, 1e-9, "North pole latitude");
  Near(pole.altitudeM, 0, 1e-6, "WGS84 polar radius");
  Invalid([] { EcefToGeodetic({}); }, "ECEF origin");
  for (const Geodetic origin : {Geodetic{52.23, 21.02, 123.4}, {-45, -179.99, -100},
                                {89.99999, 120, 10000}, {0, 180, 0}})
  {
    const auto result = EcefToGeodetic(GeodeticToEcef(
      origin.latitudeDeg, origin.longitudeDeg, origin.altitudeM));
    Near(result.latitudeDeg, origin.latitudeDeg, 1e-9, "Geodetic latitude round trip");
    Near(result.longitudeDeg, origin.longitudeDeg, 1e-9, "Geodetic longitude round trip");
    Near(result.altitudeM, origin.altitudeM, 1e-6, "Geodetic altitude round trip");
    const Vec3 velocity{15, -7, 3};
    const auto enu = EcefVectorToEnu(EnuVectorToEcef(velocity, origin.latitudeDeg,
      origin.longitudeDeg), origin.latitudeDeg, origin.longitudeDeg);
    Near(enu.x, velocity.x, 1e-12, "ENU east round trip");
    Near(enu.y, velocity.y, 1e-12, "ENU north round trip");
    Near(enu.z, velocity.z, 1e-12, "ENU up round trip");
  }
  const auto east = EnuVectorToEcef({1, 0, 0}, 0, 0);
  Near(east.x, 0, 1e-12, "ENU east at Greenwich X");
  Near(east.y, 1, 1e-12, "ENU east at Greenwich Y");
  const auto north = OffsetGeodetic({0, 0, 0}, {0, 100, 0});
  Check(north.latitudeDeg > 0 && std::abs(north.longitudeDeg) < 1e-9,
        "North offset must increase latitude");
  Near(SurfaceDistanceM(0, 179.9, 0, -179.9), 22239.0160467, 1e-4,
       "Distance must cross the date line on the short arc");

  // Four regular-tetrahedron sight lines have isotropic position covariance 3/4.
  const double u = 1 / std::sqrt(3.0);
  const auto dop = ComputeDop({{u,u,u}, {u,-u,-u}, {-u,u,-u}, {-u,-u,u}});
  Check(dop.valid, "Full-rank satellite geometry must have a DOP");
  Near(dop.hdop, std::sqrt(1.5), 1e-12, "Symmetric geometry HDOP");
  Near(dop.vdop, std::sqrt(0.75), 1e-12, "Symmetric geometry VDOP");
  Near(dop.pdop, 1.5, 1e-12, "Symmetric geometry PDOP");
  Check(!ComputeDop({{1,0,0}, {0,1,0}, {0,0,1}}).valid, "Three satellites cannot fix");
  Check(!ComputeDop({{1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}}).valid,
        "Singular geometry cannot fix");
  Check(!ComputeDop({{2,0,0}, {0,1,0}, {0,0,1}, {-1,0,0}}).valid,
        "Non-unit sight lines must be rejected");
}

const std::string kYuma =
  "******** Week 352 almanac for PRN-01 ********\n"
  "ID: 1\nHealth: 0\nEccentricity: 1.0D-2\nTime of Applicability(s): 0\n"
  "Orbital Inclination(rad): 0.959931\nRate of Right Ascen(r/s): -8e-9\n"
  "SQRT(A)  (m 1/2): 5153.63949069005\nRight Ascen at Week(rad): 0\n"
  "Argument of Perigee(rad): 0\nMean Anom(rad): 0\nAf0(s): 0\nAf1(s/s): 0\nweek: 352\n";

std::vector<SatelliteAlmanac> Parse(const std::string &text)
{
  std::istringstream input(text);
  return ParseYumaAlmanac(input, 2400 * kGpsWeekSeconds);
}

void Almanac()
{
  Near(UnixToGpsSeconds(kGpsEpochUnixSeconds, 0), 0, 0, "GPS epoch conversion");
  Near(UnixToGpsSeconds(kGpsEpochUnixSeconds + 100, 18), 118, 0, "Leap seconds conversion");
  Check(ResolveGpsWeek(352, 2400 * kGpsWeekSeconds) == 2400, "10-bit week rollover");
  Check(ResolveGpsWeek(2400, 0) == 2400, "Full GPS week is not reinterpreted");
  Invalid([] { ResolveGpsWeek(-1, 0); }, "Negative GPS week");
  const auto records = Parse(kYuma);
  Check(records.size() == 1 && records[0].prn == 1 && records[0].gpsWeek == 2400,
        "Yuma record and week decoding");
  Near(records[0].eccentricity, 0.01, 0, "Fortran exponent parsing");
  Invalid([] { Parse(""); }, "Empty almanac");
  Invalid([] { Parse("ID: 1\nHealth: 0\n"); }, "Incomplete record");
  Invalid([] { Parse(kYuma + kYuma); }, "Duplicate PRN");
  Invalid([] { Parse(kYuma + "week: 352\n"); }, "Duplicate field");
  Invalid([] { Parse(kYuma + "unknown: 1\n"); }, "Unknown field");
  Invalid([] { Parse("Health: 0\n" + kYuma); }, "Field before ID");
  Invalid([] { Parse(kYuma + "******** Week 352 almanac for PRN-02 ********\n"); },
          "Truncated final record");
  for (const auto &bad : {std::string("nan"), std::string("1.2"), std::string("33"),
                          std::string("1 trailing")})
  {
    auto invalid = kYuma;
    invalid.replace(invalid.find("ID: 1"), 5, "ID: " + bad);
    Invalid([&] { Parse(invalid); }, "Malformed satellite identifier");
  }
  const double epoch = 2400 * kGpsWeekSeconds;
  SatelliteAlmanac circular;
  circular.gpsWeek = 2400;
  circular.sqrtSemiMajorAxis = std::sqrt(26560000.0);
  const auto position = SatellitePositionEcef(circular, epoch);
  Near(position.x, 26560000, 1e-7, "Circular orbit starts on X axis");
  Near(position.y, 0, 1e-7, "Circular orbit Y at epoch");
  Near(position.z, 0, 1e-7, "Equatorial orbit Z");
  const auto later = SatellitePositionEcef(circular, epoch + 3600);
  Near(std::hypot(std::hypot(later.x, later.y), later.z), 26560000, 1e-7,
       "Circular orbit radius remains constant");
  Check(std::abs(later.y) > 1000000, "Orbit must advance with simulation time");
  const auto constellation = SyntheticGpsConstellation(epoch);
  std::set<unsigned int> prns;
  for (const auto &satellite : constellation)
  {
    prns.insert(satellite.prn);
    const auto p = SatellitePositionEcef(satellite, epoch + 1000000);
    Near(std::hypot(std::hypot(p.x, p.y), p.z), 26560000, 1e-6, "Synthetic orbit radius");
  }
  Check(constellation.size() == 32 && prns.size() == 32 && *prns.begin() == 1 &&
        *prns.rbegin() == 32, "Unique synthetic constellation within GPS PRN range");
}

Measurement Sample(Receiver &receiver, double time)
{
  return receiver.Sample(time, 52.23, 21.02, 100, {20, -5, 1});
}

void SameFix(const Measurement &left, const Measurement &right)
{
  Near(left.latitudeDeg, right.latitudeDeg, 0, "Reproducible latitude");
  Near(left.longitudeDeg, right.longitudeDeg, 0, "Reproducible longitude");
  Near(left.altitudeM, right.altitudeM, 0, "Reproducible altitude");
  Near(left.velocityEnuMps.x, right.velocityEnuMps.x, 0, "Reproducible east velocity");
  Near(left.velocityEnuMps.y, right.velocityEnuMps.y, 0, "Reproducible north velocity");
  Near(left.velocityEnuMps.z, right.velocityEnuMps.z, 0, "Reproducible up velocity");
  Check(left.fixType == right.fixType && left.satellitesUsed == right.satellitesUsed,
        "Reproducible fix status");
}

void Px4DefaultGeometry()
{
  // A receiver-level 3D fix is insufficient: PX4 also checks satellite count,
  // PDOP and reported accuracy. Cover a full day, including satellite rise/set,
  // at McMillan (the reported regression), Warsaw, Zurich and the equator.
  for (const Geodetic location : {Geodetic{35.716880, -120.764640, 280.396},
                                  {52.23, 21.02, 100},
                                  {47.397742, 8.545594, 488}, {0, 0, 0}})
  {
    Config config;
    config.seed = 42;
    Receiver receiver(config);
    const auto check = [&](double time)
    {
      const auto fix = receiver.Sample(time, location.latitudeDeg,
                                      location.longitudeDeg, location.altitudeM, {});
      const std::string context = " at latitude " + std::to_string(location.latitudeDeg) +
                                  ", t=" + std::to_string(time);
      Check(fix.valid && fix.fixType == 3 && fix.satellitesUsed >= 6,
            "Default receiver must meet PX4 fix/satellite requirements" + context);
      Check(std::hypot(fix.hdop, fix.vdop) <= 2.5,
            "Default receiver must meet PX4 EKF2_REQ_PDOP=2.5" + context);
      Check(fix.ephM <= 3.0 && fix.epvM <= 5.0 && fix.speedAccuracyMps <= 0.5,
            "Default receiver must meet PX4 reported-accuracy requirements" + context);
    };
    // Native 5 Hz during initialization, then 1 Hz through the rest of the day.
    for (int sample = 0; sample <= 3000; ++sample) check(sample / 5.0);
    for (int second = 601; second <= 86400; ++second) check(second);
  }
}

void ReceiverBehavior()
{
  Config config;
  Receiver receiver(config), second(config);
  const auto initial = Sample(receiver, 0);
  Check(initial.valid && initial.fixType == 3 && initial.satellitesUsed >= 4,
        "Default constellation should fix at Warsaw");
  Check(initial.ephM > 0 && initial.epvM > 0 && initial.hdop < 9999,
        "Valid fix must report usable accuracy");
  SameFix(initial, Sample(second, 0));
  SameFix(initial, Sample(receiver, 0));
  for (double time : {0.2, 1.0, 20.0}) SameFix(Sample(receiver, time), Sample(second, time));
  receiver.Reset();
  SameFix(initial, Sample(receiver, 0));
  Sample(receiver, 10);
  SameFix(initial, Sample(receiver, 0)); // Simulation rewind also resets the noise state.

  config.quality = Quality::Denied;
  Receiver denied(config);
  const auto outage = Sample(denied, 0);
  Check(!outage.valid && outage.fixType == 1 && outage.satellitesUsed == 0 &&
        outage.ephM == 9999, "Denied reception must not claim a valid fix");
  config.quality = Quality::Normal;
  config.elevationMaskDeg = 90;
  Receiver masked(config);
  Check(!Sample(masked, 0).valid, "90-degree mask produces outage");
  config = Config{};
  config.zones.push_back({52.23, 21.02, 500, 0, 200, Quality::Poor});
  config.zones.push_back({52.23, 21.02, 100, 0, 200, Quality::Denied});
  Receiver zones(config);
  Check(Sample(zones, 0).quality == Quality::Denied, "Last matching zone wins");
  Check(zones.Sample(1, 52.23, 21.02, 300, {}).quality == Quality::Normal,
        "Zone altitude bounds are respected");
  Check(zones.Sample(2, 0, 0, 100, {}).quality == Quality::Normal,
        "Outside zones uses receiver quality");

  config = Config{};
  Receiver clean(config);
  config.quality = Quality::Spoofing;
  config.spoofStartSeconds = 10;
  config.spoofRampSeconds = 20;
  config.spoofOffsetEnuM = {200, 100, 30};
  Receiver spoof(config);
  SameFix(Sample(clean, 0), Sample(spoof, 0));
  for (double time : {20.0, 30.0})
  {
    const auto reference = Sample(clean, time), altered = Sample(spoof, time);
    Check(altered.valid && altered.spoofing, "Spoofing preserves a claimed fix");
    Near(altered.ephM, reference.ephM, 0, "Spoofing does not advertise inflated uncertainty");
    const auto a = GeodeticToEcef(reference.latitudeDeg, reference.longitudeDeg, reference.altitudeM);
    const auto b = GeodeticToEcef(altered.latitudeDeg, altered.longitudeDeg, altered.altitudeM);
    const auto delta = EcefVectorToEnu({b.x-a.x, b.y-a.y, b.z-a.z}, 52.23, 21.02);
    const double fraction = time == 20 ? 0.5 : 1.0;
    Near(delta.x, 200 * fraction, 1e-6, "Spoofed east displacement");
    Near(delta.y, 100 * fraction, 1e-6, "Spoofed north displacement");
    Near(delta.z, 30 * fraction, 1e-6, "Spoofed vertical displacement");
    Near(altered.velocityEnuMps.x - reference.velocityEnuMps.x, time == 20 ? 15 : 0,
         1e-10, "Spoofed velocity follows ramp and stops at its end");
  }
  Invalid([] { Config c; c.updateRateHz = 0; Receiver invalid(c); }, "Zero update rate");
  Invalid([] { Config c; c.elevationMaskDeg = 91; Receiver invalid(c); }, "Invalid mask");
  Invalid([&] { receiver.Sample(-1, 0, 0, 0, {}); }, "Negative simulation time");
  Invalid([&] { receiver.Sample(0, 91, 0, 0, {}); }, "Invalid latitude");
  Invalid([&] { receiver.Sample(0, 0, 0, 0, {std::numeric_limits<double>::infinity(),0,0}); },
          "Nonfinite velocity");
  Check(ParseQuality("GPS-SPOOFING") == Quality::Spoofing &&
        ParseQuality("jamming") == Quality::Denied, "Quality aliases");
  Invalid([] { ParseQuality("unknown"); }, "Unknown quality");
}

void AlmanacAge()
{
  const auto path = std::filesystem::current_path() / "receiver-test.yuma";
  struct Cleanup
  {
    std::filesystem::path path;
    ~Cleanup() { std::error_code error; std::filesystem::remove(path, error); }
  } cleanup{path};
  Config config;
  const double gps = UnixToGpsSeconds(config.epochUnixSeconds, config.gpsUtcLeapSeconds);
  std::ofstream file(path);
  file << std::setprecision(17);
  for (const auto &s : SyntheticGpsConstellation(gps))
    file << "ID: " << s.prn << "\nHealth: 0\nEccentricity: 0\nTime of Applicability(s): "
         << s.timeOfApplicabilitySeconds << "\nOrbital Inclination(rad): " << s.inclinationRad
         << "\nRate of Right Ascen(r/s): " << s.rightAscensionRateRadPerSec
         << "\nSQRT(A)  (m 1/2): " << s.sqrtSemiMajorAxis
         << "\nRight Ascen at Week(rad): " << s.rightAscensionAtWeekRad
         << "\nArgument of Perigee(rad): 0\nMean Anom(rad): " << s.meanAnomalyRad
         << "\nAf0(s): 0\nAf1(s/s): 0\nweek: " << s.gpsWeek << '\n';
  file.close();
  Check(file.good(), "Write temporary almanac");
  config.almanacPath = path.string();
  config.maxAlmanacAgeDays = 1;
  Receiver fromFile(config);
  Check(Sample(fromFile, 0).valid, "Fresh file should produce a fix");
  Near(fromFile.AlmanacAgeSeconds(100), 100, 1e-6, "File ages with simulation time");
  Check(!Sample(fromFile, 86401).valid, "File becoming stale must invalidate fixes");
  config.epochUnixSeconds += 86401;
  Invalid([&] { Receiver stale(config); }, "Stale file rejected at startup");
  config.maxAlmanacAgeDays = 0;
  Receiver allowed(config);
  Check(allowed.Source().find("yuma:") == 0, "Explicit stale override keeps file source");
}
}

int main()
{
  try
  {
    Geometry();
    Almanac();
    Px4DefaultGeometry();
    ReceiverBehavior();
    AlmanacAge();
    std::cout << "Geometry, almanac parsing, receiver behavior and stale-data protection passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
