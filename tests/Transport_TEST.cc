#include "real_gps/adapters/gzsim/Coordinates.hh"
#include "real_gps/adapters/gzsim/Topics.hh"

#include <algorithm>
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
    std::vector<gz::msgs::NavSat> normalMessages, deniedMessages, scenarioMessages;
    std::vector<gz::msgs::NavSat> invalidMessages, absentMessages, connectionMessages;
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
    std::function<void(const gz::msgs::NavSat &)> scenario = [&](const auto &message)
    {
      std::lock_guard<std::mutex> lock(mutex);
      scenarioMessages.push_back(message);
      received.notify_all();
    };
    std::function<void(const gz::msgs::NavSat &)> invalid = [&](const auto &message)
    {
      std::lock_guard<std::mutex> lock(mutex);
      invalidMessages.push_back(message);
    };
    std::function<void(const gz::msgs::NavSat &)> absent = [&](const auto &message)
    {
      std::lock_guard<std::mutex> lock(mutex);
      absentMessages.push_back(message);
    };
    std::function<void(const gz::msgs::NavSat &)> connection = [&](const auto &message)
    {
      std::lock_guard<std::mutex> lock(mutex);
      connectionMessages.push_back(message);
      received.notify_all();
    };
    Check(subscriber.Subscribe("/gps_test/absent", absent), "Subscribe to absent receivers");
    Check(subscriber.Subscribe("/gps_test/connection", connection),
          "Subscribe to receiver connection timeline");
    Check(subscriber.Subscribe("/world/gps_test/model/normal/link/base_link/sensor/navsat_sensor/navsat",
                               normal), "Subscribe to default PX4 topic");
    Check(subscriber.Subscribe("/gps_test/denied", denied), "Subscribe to custom topic");
    Check(subscriber.Subscribe("/gps_test/scenario", scenario), "Subscribe to scenario topic");
    Check(subscriber.Subscribe("/gps_test/invalid", invalid), "Subscribe to rejected configurations");
    const std::string timeline =
      "<topic>/gps_test/scenario</topic><quality>normal</quality>"
      "<reacquisition_seconds>0.3</reacquisition_seconds><scenario>"
      "<event name='healthy'><at_seconds>0</at_seconds></event>"
      "<event name='low'><at_seconds>1</at_seconds><transition_seconds>0.6</transition_seconds>"
      "<spoofing_level>0.25</spoofing_level><offset_enu_m>100 40 20</offset_enu_m></event>"
      "<event name='high'><at_seconds>2</at_seconds><transition_seconds>0.6</transition_seconds>"
      "<spoofing_level>1</spoofing_level><offset_enu_m>100 40 20</offset_enu_m>"
      "<report_detection>true</report_detection></event>"
      "<event name='denied'><at_seconds>3</at_seconds><quality>denied</quality>"
      "<spoofing_level>1</spoofing_level><offset_enu_m>100 40 20</offset_enu_m>"
      "<report_detection>true</report_detection><report_jamming>true</report_jamming></event>"
      "<event name='recover'><at_seconds>4</at_seconds><transition_seconds>0.6</transition_seconds>"
      "<report_detection>true</report_detection></event>"
      "<event name='intermittent'><at_seconds>5</at_seconds><noise_scale>1.7</noise_scale>"
      "<dropout_period_seconds>0.8</dropout_period_seconds>"
      "<dropout_duration_seconds>0.2</dropout_duration_seconds></event>"
      "<event name='restored'><at_seconds>6.6</at_seconds></event>"
      "<event name='blackout'><at_seconds>7.4</at_seconds><output_enabled>false</output_enabled></event>"
      "<event name='output_restored'><at_seconds>8.4</at_seconds></event></scenario>";
    const std::string connectionTimeline =
      "<topic>/gps_test/connection</topic><quality>none</quality>"
      "<reacquisition_seconds>0.3</reacquisition_seconds><scenario>"
      "<event name='connected'><at_seconds>1</at_seconds><quality>normal</quality></event>"
      "<event name='disconnected'><at_seconds>2</at_seconds><quality>none</quality>"
      "<output_enabled>true</output_enabled></event>"
      "<event name='reconnected'><at_seconds>3</at_seconds><quality>normal</quality></event>"
      "</scenario>";
    const std::vector<std::string> badScenarios{
      "<scenario/>",
      "<scenario><evnt><at_seconds>0</at_seconds></evnt></scenario>",
      "<scenario><event><at_seconds>0</at_seconds><spoofing_leve1>1</spoofing_leve1></event></scenario>",
      "<scenario><event><spoofing_level>1</spoofing_level></event></scenario>",
      "<scenario><event><at_seconds>0</at_seconds><at_seconds>1</at_seconds></event></scenario>",
      "<spoofing/><scenario><event><at_seconds>0</at_seconds></event></scenario>",
      "<scenario><event><at_seconds>0</at_seconds></event></scenario>"
      "<scenario><event><at_seconds>1</at_seconds></event></scenario>"
    };
    std::string rejectedModels;
    for (std::size_t i = 0; i < badScenarios.size(); ++i)
      rejectedModels += Model("invalid_" + std::to_string(i),
                             "<topic>/gps_test/invalid</topic>" + badScenarios[i]);
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
      Model("none", "<quality>none</quality><topic>/gps_test/absent</topic>") +
      Model("none_zone", "<quality>normal</quality><topic>/gps_test/absent</topic>"
            "<zone><latitude_deg>52.23</latitude_deg><longitude_deg>21.02</longitude_deg>"
            "<radius_m>100</radius_m><quality>none</quality></zone>") +
      Model("connection", connectionTimeline) +
      Model("scenario", timeline) + rejectedModels +
      "</world></sdf>"), "Accept test world");
    gz::sim::Server server(config);
    server.SetUpdatePeriod(2ms);
    Check(server.Run(false, 0, false), "Start headless server");
    std::vector<gz::msgs::NavSat> fixes, outages, scenarioFixes, rejectedFixes;
    std::vector<gz::msgs::NavSat> absentFixes, connectionFixes;
    bool complete = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      complete = received.wait_for(lock, 15s, [&]
      {
        return normalMessages.size() >= 5 && deniedMessages.size() >= 5 &&
          !scenarioMessages.empty() && scenarioMessages.back().header().stamp().sec() >= 9 &&
          !connectionMessages.empty() && connectionMessages.back().header().stamp().sec() >= 9;
      });
      fixes = normalMessages;
      outages = deniedMessages;
      scenarioFixes = scenarioMessages;
      rejectedFixes = invalidMessages;
      absentFixes = absentMessages;
      connectionFixes = connectionMessages;
    }
    server.Stop();
    Check(complete, "Expected normal, denied and complete scenario NavSat sequence within 15 seconds");
    Check(rejectedFixes.empty(), "Invalid scenario SDF must disable publication");
    Check(absentFixes.empty(), "Static none quality and none zones publish zero NavSat messages");
    const auto &fix = fixes.back();
    const auto fields = Metadata(fix);
    Check(fields.at("gnss.schema_version") == "1", "Metadata schema");
    Check(fields.at("gnss.source") == "synthetic_gps", "Explicit synthetic source metadata");
    Check(fields.at("gnss.fix_type") == "3" && std::stoi(fields.at("gnss.satellites_used")) >= 6,
          "Normal model must publish a 3D fix with enough satellites for PX4");
    Check(std::hypot(std::stod(fields.at("gnss.hdop")),
                     std::stod(fields.at("gnss.vdop"))) <= 2.5,
          "Normal NavSat metadata must meet PX4's default PDOP limit");
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
    bool connected = false, reconnected = false, reconnecting = false;
    for (const auto &sample : connectionFixes)
    {
      const double time = seconds(sample);
      Check(time >= 1 && (time < 2 || time >= 3),
            "None quality suppresses all messages before connection and during disconnection");
      const auto metadata = Metadata(sample);
      Check(metadata.at("gnss.quality") == "normal",
            "Only an explicitly connected receiver publishes messages");
      const bool valid = metadata.at("gnss.fix_type") == "3";
      if (time < 2 && valid) connected = true;
      if (time >= 3)
      {
        if (metadata.at("gnss.reacquiring") == "1")
        {
          Check(!valid, "A reconnected receiver waits for its reacquisition delay");
          reconnecting = true;
        }
        if (valid) reconnected = true;
      }
    }
    Check(connected && reconnecting && reconnected,
          "None timeline supports connection, complete message loss and reacquisition");
    std::map<std::string, unsigned int> observed;
    bool recovered = false, reacquired = false, dropout = false, betweenDropouts = false;
    for (const auto &sample : scenarioFixes)
    {
      Check(seconds(sample) < 7.4 || seconds(sample) >= 8.4,
            "Transport blackout suppresses NavSat publication for its full interval");
      const auto metadata = Metadata(sample);
      const auto &event = metadata.at("gnss.scenario.event_name");
      ++observed[event];
      const double elapsed = std::stod(metadata.at("gnss.scenario.elapsed_seconds"));
      const double level = std::stod(metadata.at("gnss.scenario.spoofing_level"));
      const double east = std::stod(metadata.at("gnss.scenario.offset_east_m"));
      const double eastVelocity = std::stod(metadata.at("gnss.scenario.velocity_east_mps"));
      const bool hasFix = metadata.at("gnss.fix_type") == "3";
      Check(!metadata.at("gnss.status").empty(), "Every scenario sample reports receiver status");
      Near(std::stod(metadata.at("gnss.truth_latitude_deg")), 52.23, 1e-8,
           "Spoofing must preserve independent truth metadata");
      if (event == "low" || event == "high")
      {
        const double u = std::clamp(elapsed / 0.6, 0.0, 1.0);
        const double f = u * u * (3 - 2 * u);
        const double prior = event == "low" ? 0.0 : 0.25;
        const double target = event == "low" ? 0.25 : 1.0;
        Near(level, prior + (target - prior) * f, 1e-10, "Timed spoofing level");
        Near(east, 100 * level, 1e-8, "Timed spoofing ENU bias");
        Near(eastVelocity, u > 0 && u < 1 ? 100 * (target - prior) * 6 * u * (1 - u) / 0.6 : 0,
             1e-8, "Spoof velocity follows the parsed transition");
        Check(hasFix && metadata.at("gnss.spoofing_active") == (level > 0 ? "1" : "0"),
              "Spoofing preserves a claimed receiver fix");
        Check(metadata.at("gnss.spoofing_state") == (event == "high" ? "3" : "0"),
              "Reported detection is separate from attack truth");
      }
      else if (event == "denied")
      {
        Check(!hasFix && metadata.at("gnss.satellites_used") == "0" &&
              metadata.at("gnss.spoofing_active") == "1" &&
              metadata.at("gnss.spoofing_state") == "3" && metadata.at("gnss.jamming_state") == "3",
              "A denied receiver preserves attack truth and configured interference reports");
      }
      else if (event == "recover")
      {
        const double u = std::clamp(elapsed / 0.6, 0.0, 1.0);
        Near(east, 100 * (1 - u * u * (3 - 2 * u)), 1e-8,
             "Recovery removes the previous offset smoothly");
        if (metadata.at("gnss.reacquiring") == "1")
        {
          Check(!hasFix, "Reacquisition delay cannot publish a valid fix");
          reacquired = true;
        }
        if (elapsed >= 0.6)
        {
          Check(hasFix && metadata.at("gnss.spoofing_active") == "0" &&
                metadata.at("gnss.spoofing_state") == "0", "Recovery restores honest receiver output");
          recovered = true;
        }
      }
      else if (event == "intermittent")
      {
        Near(std::stod(metadata.at("gnss.scenario.noise_scale")), 1.7, 1e-12,
             "Scenario noise scale survives SDF parsing");
        if (metadata.at("gnss.scenario.dropout") == "1")
        {
          Check(!hasFix && metadata.at("gnss.jamming_state") == "0",
                "Periodic dropout is invalid without invented jamming detection");
          dropout = true;
        }
        else if (hasFix) betweenDropouts = true;
      }
      else if (event == "restored")
      {
        Check(metadata.at("gnss.scenario.dropout") == "0", "Next event clears dropout configuration");
        Near(std::stod(metadata.at("gnss.scenario.noise_scale")), 1.0, 1e-12,
             "Each event supplies a complete target state");
      }
    }
    for (const auto *event : {"healthy", "low", "high", "denied", "recover", "intermittent", "restored", "output_restored"})
      Check(observed[event] > 0, "Missing scenario event " + std::string(event));
    Check(recovered && reacquired && dropout && betweenDropouts,
          "Scenario must demonstrate recovery, reacquisition and intermittent reception");
    std::cout << "Plugin load, topics, coordinates, timing, scenario SDF, rejection and GNSS metadata passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
