#include "real_gps/adapters/gzsim/Coordinates.hh"
#include "real_gps/adapters/gzsim/Topics.hh"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <gz/msgs/navsat.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

namespace
{
void Check(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

void Near(double actual, double expected, double tolerance, const std::string &message)
{
  Check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
        message + ": got " + std::to_string(actual));
}

std::map<std::string, std::string> Metadata(const gz::msgs::NavSat &message)
{
  std::map<std::string, std::string> result;
  for (const auto &field : message.header().data())
  {
    Check(field.value_size() == 1, "Metadata must have one value per key");
    Check(result.emplace(field.key(), field.value(0)).second, "Duplicate metadata key");
  }
  return result;
}

std::string Model(const std::string &name, const std::string &options)
{
  return "<model name='" + name + "'><static>true</static><link name='base_link'/>"
    "<plugin filename='" REAL_GPS_LIBRARY "' name='real_gps::RealGpsSystem'>"
    "<antenna_offset>0 0 0.15</antenna_offset><update_rate>5</update_rate>" +
    options + "</plugin></model>";
}
}

int main()
{
  try
  {
    using namespace std::chrono_literals;
    const auto partition = "real_gps_test_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    Check(setenv("GZ_PARTITION", partition.c_str(), 1) == 0, "Set isolated transport partition");
    Check(setenv("GZ_IP", "127.0.0.1", 1) == 0, "Use loopback transport");
    Check(real_gps::gzsim::CanonicalTopic("gps/navsat/") == "/gps/navsat",
          "Relative and trailing-slash topic normalization");
    Check(real_gps::gzsim::CanonicalTopic("/gps/navsat") == "/gps/navsat",
          "Absolute topic normalization");
    Check(real_gps::gzsim::CanonicalTopic("").empty(), "Empty topic must be invalid");

    // A heading-free world is ENU at its origin on both supported Math APIs.
    gz::math::SphericalCoordinates frame(gz::math::SphericalCoordinates::EARTH_WGS84,
      gz::math::Angle(0.0), gz::math::Angle(0.0), 0.0, gz::math::Angle(0.0));
    const auto velocity = real_gps::gzsim::VelocityEnu(frame, {3, 4, 5}, 0, 0);
    Near(velocity.X(), 3, 1e-12, "ENU east velocity");
    Near(velocity.Y(), 4, 1e-12, "ENU north velocity");
    Near(velocity.Z(), 5, 1e-12, "ENU up velocity");

    std::mutex mutex;
    std::condition_variable received;
    std::vector<gz::msgs::NavSat> normalMessages, deniedMessages;
    gz::transport::Node subscriber;
    std::function<void(const gz::msgs::NavSat &)> normal = [&](const auto &message)
    {
      std::lock_guard<std::mutex> lock(mutex);
      normalMessages.push_back(message);
      received.notify_all();
    };
    std::function<void(const gz::msgs::NavSat &)> denied = [&](const auto &message)
    {
      std::lock_guard<std::mutex> lock(mutex);
      deniedMessages.push_back(message);
      received.notify_all();
    };
    Check(subscriber.Subscribe("/world/gps_test/model/normal/link/base_link/sensor/navsat_sensor/navsat",
                               normal), "Subscribe to default PX4 topic");
    Check(subscriber.Subscribe("/gps_test/denied", denied), "Subscribe to custom topic");
    gz::sim::ServerConfig config;
    Check(config.SetSdfString(
      "<sdf version='1.9'><world name='gps_test'>"
      "<physics name='default' type='ignored'><max_step_size>0.01</max_step_size></physics>"
      "<spherical_coordinates><surface_model>EARTH_WGS84</surface_model>"
      "<latitude_deg>52.23</latitude_deg><longitude_deg>21.02</longitude_deg>"
      "<elevation>100</elevation><heading_deg>0</heading_deg></spherical_coordinates>"
      "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>" +
      Model("normal", "<quality>normal</quality>") +
      Model("denied", "<quality>denied</quality><topic>/gps_test/denied</topic>") +
      "</world></sdf>"), "Accept test world");
    gz::sim::Server server(config);
    server.SetUpdatePeriod(2ms);
    Check(server.Run(false, 0, false), "Start headless server");
    std::vector<gz::msgs::NavSat> fixes, outages;
    bool complete = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      complete = received.wait_for(lock, 15s, [&]
      {
        return normalMessages.size() >= 5 && deniedMessages.size() >= 5;
      });
      fixes = normalMessages;
      outages = deniedMessages;
    }
    server.Stop();
    Check(complete, "Expected five normal and denied NavSat messages within 15 seconds");
    const auto &fix = fixes.back();
    const auto fields = Metadata(fix);
    Check(fields.at("gnss.schema_version") == "1", "Metadata schema");
    Check(fields.at("gnss.source") == "synthetic_gps", "Explicit synthetic source metadata");
    Check(fields.at("gnss.fix_type") == "3" && std::stoi(fields.at("gnss.satellites_used")) >= 4,
          "Normal model must publish a 3D fix");
    Check(fields.at("gnss.quality") == "normal", "Normal reception metadata");
    Check(std::stod(fields.at("gnss.eph")) > 0 && std::stod(fields.at("gnss.eph")) < 100,
          "Reported horizontal accuracy");
    Near(std::stod(fields.at("gnss.truth_latitude_deg")), 52.23, 1e-8, "World latitude");
    Near(std::stod(fields.at("gnss.truth_longitude_deg")), 21.02, 1e-8, "World longitude");
    Near(std::stod(fields.at("gnss.truth_altitude_m")), 100.15, 1e-5, "Antenna altitude");
    Near(fix.latitude_deg(), 52.23, 0.01, "Measured latitude");
    Near(fix.longitude_deg(), 21.02, 0.01, "Measured longitude");
    Check(fix.frame_id() == "gps_test::normal::base_link", "NavSat link frame: " + fix.frame_id());
    const auto seconds = [](const gz::msgs::NavSat &message)
    {
      return message.header().stamp().sec() + message.header().stamp().nsec() * 1e-9;
    };
    for (std::size_t i = 1; i < fixes.size(); ++i)
      Near(seconds(fixes[i]) - seconds(fixes[i-1]), 0.2, 1e-8, "Configured 5 Hz publication");
    Near(std::stod(fields.at("gnss.time_utc_usec")) / 1e6,
         1767225600.0 + seconds(fix), 1e-6, "Simulated UTC timestamp");
    const auto deniedFields = Metadata(outages.back());
    Check(deniedFields.at("gnss.fix_type") == "1" && deniedFields.at("gnss.satellites_used") == "0",
          "Denied model must publish no-fix metadata");
    Check(deniedFields.at("gnss.quality") == "denied", "Custom-topic denied reception metadata");
    std::cout << "Plugin load, both topic paths, coordinates, timing and GNSS transport metadata passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
