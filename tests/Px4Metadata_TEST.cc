#include "../integration/px4/GZRealGps.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <gz/msgs/navsat.pb.h>

namespace
{
struct Report
{
  double latitude_deg{}, longitude_deg{}, altitude_msl_m{}, altitude_ellipsoid_m{};
  float eph{}, epv{}, hdop{}, vdop{}, s_variance_m_s{}, c_variance_rad{};
  uint8_t satellites_used{}, fix_type{}, jamming_state{}, spoofing_state{};
  float vel_n_m_s{}, vel_e_m_s{}, vel_d_m_s{}, vel_m_s{}, cog_rad{};
  bool vel_ned_valid{};
  uint64_t time_utc_usec{};
  float heading{}, heading_offset{};
};

void Check(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

void Set(gz::msgs::NavSat &message, const std::string &key, const std::string &value)
{
  for (auto &field : *message.mutable_header()->mutable_data())
  {
    if (field.key() != key) continue;
    field.clear_value();
    field.add_value(value);
    return;
  }
  auto *field = message.mutable_header()->add_data();
  field->set_key(key);
  field->add_value(value);
}

gz::msgs::NavSat Fix()
{
  gz::msgs::NavSat message;
  message.set_latitude_deg(52.23);
  message.set_longitude_deg(21.02);
  message.set_altitude(110);
  message.set_velocity_north(3);
  message.set_velocity_east(4);
  message.set_velocity_up(2);
  Set(message, "gnss.schema_version", "1");
  Set(message, "gnss.fix_type", "3");
  Set(message, "gnss.satellites_used", "8");
  Set(message, "gnss.eph", "2.5");
  Set(message, "gnss.epv", "4.5");
  Set(message, "gnss.hdop", "1.2");
  Set(message, "gnss.vdop", "1.8");
  Set(message, "gnss.speed_accuracy", "0.2");
  Set(message, "gnss.time_utc_usec", "1767225600000000");
  return message;
}

void Invalid(const gz::msgs::NavSat &message)
{
  Report report;
  report.latitude_deg = -77;
  gz_real_gps::GroundTruth truth;
  truth.valid = true;
  truth.latitude_deg = -66;
  Check(gz_real_gps::Decode(message, report, truth) == gz_real_gps::DecodeResult::Invalid,
        "Malformed metadata must reject the entire sample");
  Check(report.latitude_deg == -77 && truth.valid && truth.latitude_deg == -66,
        "Rejected metadata must not partially overwrite report or truth");
}
}

int main()
{
  try
  {
    using gz_real_gps::Decode;
    using gz_real_gps::DecodeResult;
    Report report;
    gz_real_gps::GroundTruth truth;
    Check(Decode(gz::msgs::NavSat{}, report, truth) == DecodeResult::Absent,
          "Ordinary Gazebo NavSat must retain the original PX4 callback path");
    auto message = Fix();
    Check(Decode(message, report, truth) == DecodeResult::Valid, "Complete fix metadata");
    Check(report.latitude_deg == 52.23 && report.longitude_deg == 21.02 &&
          report.altitude_msl_m == 110 && report.altitude_ellipsoid_m == 110,
          "Decoded coordinates and documented altitude convention");
    Check(report.satellites_used == 8 && report.fix_type == 3 && report.eph == 2.5f &&
          report.epv == 4.5f && report.s_variance_m_s == 0.2f,
          "Receiver quality must reach PX4 unchanged");
    Check(report.vel_n_m_s == 3 && report.vel_e_m_s == 4 && report.vel_d_m_s == -2 &&
          report.vel_m_s == 5 && report.vel_ned_valid,
          "ENU velocity must be converted to NED including the vertical sign");
    Check(std::abs(report.cog_rad - std::atan2(4.0, 3.0)) < 1e-6,
          "Course is clockwise from north");
    Check(report.time_utc_usec == 1767225600000000ULL &&
          std::isnan(report.heading) && std::isnan(report.heading_offset),
          "Preserve UTC and do not invent a receiver heading");
    Check(!truth.valid, "Measured coordinates must not be reused as ground truth");
    Set(message, "gnss.truth_latitude_deg", "52");
    Invalid(message);
    Set(message, "gnss.truth_longitude_deg", "21");
    Set(message, "gnss.truth_altitude_m", "100");
    Check(Decode(message, report, truth) == DecodeResult::Valid && truth.valid &&
          truth.latitude_deg == 52 && truth.longitude_deg == 21 && truth.altitude_m == 100,
          "Explicit ground truth stays separate from noisy measured coordinates");
    Set(message, "gnss.fix_type", "1");
    Set(message, "gnss.satellites_used", "0");
    Set(message, "gnss.jamming_state", "3");
    Set(message, "gnss.spoofing_state", "2");
    Check(Decode(message, report, truth) == DecodeResult::Valid && !report.vel_ned_valid &&
          report.fix_type == 1 && report.satellites_used == 0 &&
          report.jamming_state == 3 && report.spoofing_state == 2,
          "No-fix and interference states must survive decoding");
    for (const char *value : {"nan", "inf", "-1", "1 trailing", " 1", "1e999"})
    {
      message = Fix();
      Set(message, "gnss.eph", value);
      Invalid(message);
    }
    for (const char *value : {"-1", "256", "1.5", "18446744073709551616"})
    {
      message = Fix();
      Set(message, "gnss.satellites_used", value);
      Invalid(message);
    }
    message = Fix();
    Set(message, "gnss.schema_version", "2");
    Invalid(message);
    message = Fix();
    auto *duplicate = message.mutable_header()->add_data();
    duplicate->set_key("gnss.eph");
    duplicate->add_value("2.5");
    Invalid(message);
    message = Fix();
    message.mutable_header()->mutable_data(0)->add_value("1");
    Invalid(message);
    message = Fix();
    message.mutable_header()->mutable_data()->DeleteSubrange(0, 1);
    Invalid(message);
    message = Fix();
    message.set_latitude_deg(91);
    Invalid(message);
    message = Fix();
    message.set_velocity_up(std::numeric_limits<double>::quiet_NaN());
    Invalid(message);
    std::cout << "PX4 metadata decoding, quality, coordinate conventions and malformed input rejection passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
