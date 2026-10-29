#include "real_gps/core/Almanac.hh"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>

namespace real_gps::core
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRotationRadPerSec = 7.2921151467e-5;
constexpr double kGravitationalParameter = 3.986005e14;

std::string Normalize(std::string text)
{
  std::string result;
  for (const unsigned char c : text)
    if (std::isalnum(c))
      result += static_cast<char>(std::tolower(c));
  return result;
}

double Number(std::string text)
{
  std::replace(text.begin(), text.end(), 'D', 'E');
  std::replace(text.begin(), text.end(), 'd', 'e');
  std::size_t end = 0;
  const double result = std::stod(text, &end);
  while (end < text.size() && std::isspace(static_cast<unsigned char>(text[end])))
    ++end;
  if (end != text.size() || !std::isfinite(result))
    throw std::invalid_argument("Invalid numeric field in Yuma almanac");
  return result;
}
}

double UnixToGpsSeconds(double unixSeconds, int gpsUtcLeapSeconds)
{
  return unixSeconds - kGpsEpochUnixSeconds + gpsUtcLeapSeconds;
}

int ResolveGpsWeek(int encodedWeek, double referenceGpsSeconds)
{
  if (encodedWeek < 0 || encodedWeek > 100000 || !std::isfinite(referenceGpsSeconds))
    throw std::invalid_argument("Invalid GPS week or reference epoch");
  if (encodedWeek >= 1024)
    return encodedWeek;
  const double referenceWeek = std::floor(referenceGpsSeconds / kGpsWeekSeconds);
  return encodedWeek + 1024 * static_cast<int>(std::round((referenceWeek - encodedWeek) / 1024.0));
}

std::vector<SatelliteAlmanac> ParseYumaAlmanac(std::istream &input,
                                            double referenceGpsSeconds)
{
  const std::set<std::string> fields{
      "id", "health", "eccentricity", "timeofapplicabilitys", "orbitalinclinationrad",
      "rateofrightascenrs", "sqrtam12", "rightascenatweekrad", "argumentofperigeerad",
      "meananomrad", "af0s", "af1ss", "week"};
  std::map<std::string, double> record;
  std::vector<SatelliteAlmanac> result;
  std::set<unsigned int> prns;
  const auto finish = [&]()
  {
    if (record.empty())
      return;
    for (const auto &field : fields)
      if (record.count(field) == 0)
        throw std::invalid_argument("Yuma record is missing field: " + field);
    const auto integer = [&](const std::string &field, int maximum)
    {
      const double value = record.at(field);
      if (value < 0 || value > maximum || std::floor(value) != value)
        throw std::invalid_argument("Invalid Yuma integer field: " + field);
      return static_cast<int>(value);
    };
    SatelliteAlmanac s;
    s.prn = integer("id", 32);
    s.health = integer("health", 63);
    s.gpsWeek = ResolveGpsWeek(integer("week", 100000), referenceGpsSeconds);
    s.timeOfApplicabilitySeconds = record.at("timeofapplicabilitys");
    s.eccentricity = record.at("eccentricity");
    s.inclinationRad = record.at("orbitalinclinationrad");
    s.rightAscensionRateRadPerSec = record.at("rateofrightascenrs");
    s.sqrtSemiMajorAxis = record.at("sqrtam12");
    s.rightAscensionAtWeekRad = record.at("rightascenatweekrad");
    s.argumentOfPerigeeRad = record.at("argumentofperigeerad");
    s.meanAnomalyRad = record.at("meananomrad");
    s.clockBiasSeconds = record.at("af0s");
    s.clockDriftSecondsPerSecond = record.at("af1ss");
    if (s.prn == 0 || !prns.insert(s.prn).second || s.timeOfApplicabilitySeconds < 0.0 ||
        s.timeOfApplicabilitySeconds >= kGpsWeekSeconds || s.eccentricity < 0.0 ||
        s.eccentricity >= 1.0 || s.sqrtSemiMajorAxis < 4000.0 ||
        s.sqrtSemiMajorAxis > 7000.0 || s.inclinationRad < 0.0 || s.inclinationRad > kPi ||
        std::abs(s.rightAscensionRateRadPerSec) > 1e-3)
      throw std::invalid_argument("Invalid or duplicate satellite in Yuma almanac");
    result.push_back(s);
    record.clear();
  };
  std::string line;
  bool headerNeedsRecord = false;
  while (std::getline(input, line))
  {
    const auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || line[first] == '#')
      continue;
    if (Normalize(line).find("almanacforprn") != std::string::npos)
    {
      if (headerNeedsRecord)
        throw std::invalid_argument("Yuma satellite header is missing its record");
      finish();
      headerNeedsRecord = true;
      continue;
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos)
      throw std::invalid_argument("Unrecognized line in Yuma almanac");
    const auto key = Normalize(line.substr(0, colon));
    if (fields.count(key) == 0)
      throw std::invalid_argument("Unrecognized Yuma field: " + key);
    if (key == "id")
    {
      finish();
      headerNeedsRecord = false;
    }
    else if (record.count("id") == 0)
      throw std::invalid_argument("Yuma field appears before satellite ID");
    if (!record.emplace(key, Number(line.substr(colon + 1))).second)
      throw std::invalid_argument("Duplicate Yuma field: " + key);
  }
  if (input.bad())
    throw std::runtime_error("Failed reading Yuma almanac");
  if (headerNeedsRecord)
    throw std::invalid_argument("Yuma satellite header is missing its record");
  finish();
  if (result.empty())
    throw std::invalid_argument("No satellite records in Yuma almanac");
  return result;
}

std::vector<SatelliteAlmanac> LoadYumaAlmanac(const std::string &path,
                                           double referenceGpsSeconds)
{
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Cannot open Yuma almanac: " + path);
  return ParseYumaAlmanac(input, referenceGpsSeconds);
}

std::vector<SatelliteAlmanac> SyntheticGpsConstellation(double referenceGpsSeconds)
{
  const int week = static_cast<int>(std::floor(referenceGpsSeconds / kGpsWeekSeconds));
  const double toa = referenceGpsSeconds - week * kGpsWeekSeconds;
  // Eight synthetic orbital planes with four satellites each. Staggered phase
  // offsets avoid the poor geometry of the former 24-satellite layout, which
  // exceeded PX4's default PDOP limit at McMillan even with normal reception.
  // These are simulation slots, not the operational GPS constellation. Keep
  // DOP and measurement covariance derived from the actual sight lines.
  constexpr std::array<double, 8> phaseDegrees{0, 35, 70, 100, 140, 165, 200, 240};
  std::vector<SatelliteAlmanac> result;
  result.reserve(32);
  for (std::size_t plane = 0; plane < phaseDegrees.size(); ++plane)
  {
    for (int slot = 0; slot < 4; ++slot)
    {
      SatelliteAlmanac s;
      s.prn = static_cast<unsigned int>(plane * 4 + slot + 1);
      s.gpsWeek = week;
      s.timeOfApplicabilitySeconds = toa;
      s.inclinationRad = 55.0 * kPi / 180.0;
      s.sqrtSemiMajorAxis = std::sqrt(26560000.0);
      s.rightAscensionAtWeekRad = plane * kPi / 4.0 + kEarthRotationRadPerSec * toa;
      s.rightAscensionRateRadPerSec = -8.0e-9;
      s.meanAnomalyRad = slot * kPi / 2.0 + phaseDegrees[plane] * kPi / 180.0;
      result.push_back(s);
    }
  }
  return result;
}

Vec3 SatellitePositionEcef(const SatelliteAlmanac &s, double gpsSeconds)
{
  // Almanac orbit model from IS-GPS-200. No broadcast-ephemeris harmonic terms.
  // Do not fold tk into +/- half a week: that would repeat old almanacs forever.
  const double tk = gpsSeconds - s.gpsWeek * kGpsWeekSeconds - s.timeOfApplicabilitySeconds;
  const double a = s.sqrtSemiMajorAxis * s.sqrtSemiMajorAxis;
  const double n = std::sqrt(kGravitationalParameter / (a * a * a));
  const double mean = std::remainder(s.meanAnomalyRad + n * tk, 2.0 * kPi);
  double eccentric = mean;
  for (int iteration = 0; iteration < 20; ++iteration)
  {
    const double change = (eccentric - s.eccentricity * std::sin(eccentric) - mean) /
                          (1.0 - s.eccentricity * std::cos(eccentric));
    eccentric -= change;
    if (std::abs(change) < 1e-13)
      break;
  }
  const double trueAnomaly = std::atan2(std::sqrt(1.0 - s.eccentricity * s.eccentricity) *
      std::sin(eccentric), std::cos(eccentric) - s.eccentricity);
  const double u = trueAnomaly + s.argumentOfPerigeeRad;
  const double radius = a * (1.0 - s.eccentricity * std::cos(eccentric));
  const double ascending = s.rightAscensionAtWeekRad +
      (s.rightAscensionRateRadPerSec - kEarthRotationRadPerSec) * tk -
      kEarthRotationRadPerSec * s.timeOfApplicabilitySeconds;
  const double x = radius * std::cos(u), y = radius * std::sin(u);
  return {x * std::cos(ascending) - y * std::cos(s.inclinationRad) * std::sin(ascending),
          x * std::sin(ascending) + y * std::cos(s.inclinationRad) * std::cos(ascending),
          y * std::sin(s.inclinationRad)};
}
}  // namespace real_gps::core
