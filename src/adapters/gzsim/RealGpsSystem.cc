#include "real_gps/core/Receiver.hh"
#include "real_gps/adapters/gzsim/Coordinates.hh"
#include "real_gps/adapters/gzsim/Topics.hh"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <gz/common/Console.hh>
#include <gz/msgs/navsat.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/NavSat.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace real_gps
{
namespace
{
template <typename T>
T Read(const sdf::ElementPtr &element, const std::string &name, const T &fallback)
{
  if (!element->HasElement(name)) return fallback;
  const auto result = element->Get<T>(name, fallback);
  if (!result.second) throw std::invalid_argument("Invalid <" + name + "> value");
  return result.first;
}

void CheckChildren(const sdf::ElementPtr &element,
                   std::initializer_list<std::string> allowed)
{
  for (auto child = element->GetFirstElement(); child; child = child->GetNextElement())
  {
    bool known = false;
    for (const auto &name : allowed) known = known || child->GetName() == name;
    if (!known)
      throw std::invalid_argument("Unknown <" + child->GetName() + "> inside <" +
                                  element->GetName() + ">");
    if (element->GetName() == "event" && child->GetNextElement(child->GetName()))
      throw std::invalid_argument("Duplicate <" + child->GetName() + "> inside <event>");
  }
}

std::vector<core::ScenarioEvent> ReadScenario(const sdf::ElementPtr &element)
{
  CheckChildren(element, {"event"});
  if (!element->HasElement("event"))
    throw std::invalid_argument("<scenario> requires at least one <event>");
  std::vector<core::ScenarioEvent> events;
  for (auto event = element->GetElement("event"); event;
       event = event->GetNextElement("event"))
  {
    CheckChildren(event, {"at_seconds", "transition_seconds", "quality", "spoofing_level",
                         "offset_enu_m", "spoof_velocity", "report_detection", "report_jamming",
                         "noise_scale", "dropout_period_seconds", "dropout_duration_seconds",
                         "output_enabled"});
    if (!event->HasElement("at_seconds"))
      throw std::invalid_argument("Every scenario event requires <at_seconds>");
    core::ScenarioEvent target;
    if (const auto name = event->GetAttribute("name")) target.name = name->GetAsString();
    target.atSeconds = Read<double>(event, "at_seconds", 0.0);
    target.transitionSeconds = Read<double>(event, "transition_seconds", target.transitionSeconds);
    if (event->HasElement("quality"))
      target.quality = core::ParseQuality(Read<std::string>(event, "quality", "normal"));
    target.spoofingLevel = Read<double>(event, "spoofing_level", target.spoofingLevel);
    const auto offset = Read<gz::math::Vector3d>(event, "offset_enu_m", {200, 0, 0});
    target.spoofOffsetEnuM = {offset.X(), offset.Y(), offset.Z()};
    target.spoofVelocity = Read<bool>(event, "spoof_velocity", target.spoofVelocity);
    target.reportDetection = Read<bool>(event, "report_detection", target.reportDetection);
    target.reportJamming = Read<bool>(event, "report_jamming", target.reportJamming);
    target.outputEnabled = Read<bool>(event, "output_enabled", target.outputEnabled);
    target.noiseScale = Read<double>(event, "noise_scale", target.noiseScale);
    target.dropoutPeriodSeconds = Read<double>(event, "dropout_period_seconds", 0.0);
    target.dropoutDurationSeconds = Read<double>(event, "dropout_duration_seconds", 0.0);
    events.push_back(std::move(target));
  }
  return events;
}

template <typename T>
void Metadata(gz::msgs::NavSat &message, const std::string &key, const T &value)
{
  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << std::setprecision(17) << value;
  auto *entry = message.mutable_header()->add_data();
  entry->set_key(key);
  entry->add_value(text.str());
}
}

class RealGpsSystem final : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate,
                            public gz::sim::ISystemPostUpdate,
                            public gz::sim::ISystemReset
{
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    try
    {
      gz::sim::Model model(entity);
      if (!model.Valid(ecm)) throw std::invalid_argument("Attach RealGpsSystem to a <model>");
      modelEntity_ = entity;
      const auto config = sdf->Clone();
      const auto linkName = Read<std::string>(config, "link_name", "base_link");
      sensorName_ = Read<std::string>(config, "sensor_name", "navsat_sensor");
      if (sensorName_.empty() || sensorName_.find('/') != std::string::npos)
        throw std::invalid_argument("sensor_name must be a nonempty unscoped name");
      link_ = gz::sim::Link(model.LinkByName(ecm, linkName));
      if (!link_.Valid(ecm)) throw std::invalid_argument("Cannot find model link " + linkName);
      staticModel_ = model.Static(ecm);
      config_.quality = core::ParseQuality(Read<std::string>(config, "quality", "normal"));
      config_.updateRateHz = Read<double>(config, "update_rate", 5.0);
      config_.epochUnixSeconds = Read<double>(config, "epoch_unix", config_.epochUnixSeconds);
      config_.gpsUtcLeapSeconds = Read<int>(config, "gps_utc_leap_seconds", config_.gpsUtcLeapSeconds);
      config_.seed = Read<std::uint64_t>(config, "seed", config_.seed);
      config_.almanacPath = Read<std::string>(config, "almanac_file", "");
      config_.maxAlmanacAgeDays = Read<double>(config, "max_almanac_age_days", config_.maxAlmanacAgeDays);
      config_.elevationMaskDeg = Read<double>(config, "elevation_mask_deg", config_.elevationMaskDeg);
      config_.noiseCorrelationSeconds = Read<double>(config, "noise_correlation_seconds", config_.noiseCorrelationSeconds);
      config_.reacquisitionSeconds = Read<double>(config, "reacquisition_seconds", config_.reacquisitionSeconds);
      antennaOffset_ = Read<gz::math::Vector3d>(config, "antenna_offset", gz::math::Vector3d::Zero);
      if (!antennaOffset_.IsFinite()) throw std::invalid_argument("antenna_offset must be finite");
      if (config->HasElement("spoofing"))
      {
        if (config->HasElement("scenario"))
          throw std::invalid_argument("Use either <scenario> or legacy <spoofing>, not both");
        auto spoof = config->GetElement("spoofing");
        config_.spoofStartSeconds = Read<double>(spoof, "start_seconds", config_.spoofStartSeconds);
        config_.spoofRampSeconds = Read<double>(spoof, "ramp_seconds", config_.spoofRampSeconds);
        const auto offset = Read<gz::math::Vector3d>(spoof, "offset_enu_m", {200, 0, 0});
        config_.spoofOffsetEnuM = {offset.X(), offset.Y(), offset.Z()};
        reportSpoofing_ = Read<bool>(spoof, "report_detection", false);
      }
      if (config->HasElement("scenario"))
      {
        const auto scenario = config->GetElement("scenario");
        if (scenario->GetNextElement("scenario"))
          throw std::invalid_argument("Only one <scenario> is allowed");
        config_.scenario = ReadScenario(scenario);
      }
      if (config->HasElement("zone"))
      {
        for (auto zone = config->GetElement("zone"); zone; zone = zone->GetNextElement("zone"))
        {
          core::DegradationZone area;
          if (!zone->HasElement("latitude_deg") || !zone->HasElement("longitude_deg"))
            throw std::invalid_argument("Every zone requires latitude_deg and longitude_deg");
          area.latitudeDeg = Read<double>(zone, "latitude_deg", 0);
          area.longitudeDeg = Read<double>(zone, "longitude_deg", 0);
          area.radiusM = Read<double>(zone, "radius_m", area.radiusM);
          area.minAltitudeM = Read<double>(zone, "min_altitude_m", area.minAltitudeM);
          area.maxAltitudeM = Read<double>(zone, "max_altitude_m", area.maxAltitudeM);
          area.quality = core::ParseQuality(Read<std::string>(zone, "quality", "poor"));
          area.elevationMaskDeg = Read<double>(zone, "elevation_mask_deg", -1);
          config_.zones.push_back(area);
        }
      }
      if (config->HasElement("topic"))
      {
        topic_ = gzsim::CanonicalTopic(Read<std::string>(config, "topic", ""));
        if (topic_.empty()) throw std::invalid_argument("Invalid NavSat <topic>");
      }
      receiver_ = std::make_unique<core::Receiver>(config_);
      link_.EnableVelocityChecks(ecm);
      enabled_ = true;
      if (config_.almanacPath.empty())
        gzwarn << "[RealGpsSystem] Using synthetic GPS constellation; set almanac_file and epoch_unix for real satellite orbits" << std::endl;
      else if (config_.maxAlmanacAgeDays <= 0)
        gzwarn << "[RealGpsSystem] Almanac age protection explicitly disabled" << std::endl;
    }
    catch (const std::exception &error)
    {
      enabled_ = false;
      gzerr << "[RealGpsSystem] Configuration failed: " << error.what() << std::endl;
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &ecm) override
  {
    if (!enabled_) return;
    if (!started_)
    {
      try
      {
        // Model Configure can run before its ParentEntity is linked to the
        // world. Resolve both the world and default scoped topic only after
        // SdfEntityCreator has finished constructing the complete scene.
        world_ = gz::sim::worldEntity(modelEntity_, ecm);
        const auto *spherical = ecm.Component<gz::sim::components::SphericalCoordinates>(world_);
        if (!spherical || spherical->Data().Surface() != gz::math::SphericalCoordinates::EARTH_WGS84)
          throw std::invalid_argument("World requires <spherical_coordinates> with EARTH_WGS84");
        if (topic_.empty())
          topic_ = gzsim::CanonicalTopic(gz::sim::scopedName(link_.Entity(), ecm) +
                                        "/sensor/" + sensorName_ + "/navsat");
        if (topic_.empty()) throw std::invalid_argument("Invalid scoped NavSat topic");
        publisher_ = node_.Advertise<gz::msgs::NavSat>(topic_);
        if (!publisher_) throw std::invalid_argument("Cannot advertise NavSat topic " + topic_);
        started_ = true;
        gzmsg << "[RealGpsSystem] " << topic_ << " quality=" << core::QualityName(config_.quality)
              << " source=" << receiver_->Source() << " epoch_unix=" << std::fixed
              << std::setprecision(0) << config_.epochUnixSeconds << std::defaultfloat << std::endl;
      }
      catch (const std::exception &error)
      {
        enabled_ = false;
        gzerr << "[RealGpsSystem] Configuration failed: " << error.what() << std::endl;
        return;
      }
    }
    // Prevent two different receivers publishing to the same PX4 input.
    ecm.Each<gz::sim::components::NavSat>([&](const gz::sim::Entity &entity,
      const gz::sim::components::NavSat *navsat)
    {
      const auto &sensor = navsat->Data();
      const auto topic = sensor.Topic().empty()
        ? gz::sim::scopedName(entity, ecm) + "/navsat" : sensor.Topic();
      if (gzsim::CanonicalTopic(topic) == topic_)
      {
        gzerr << "[RealGpsSystem] Duplicate NavSat source on " << topic_
              << "; remove the original <sensor type='navsat'> from this aircraft" << std::endl;
        enabled_ = false;
        return false;
      }
      return true;
    });
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (!enabled_ || !started_ || info.paused) return;
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (now < lastTime_) ResetState();
    lastTime_ = now;
    if (now + 1e-9 < nextTime_) return;
    const double period = 1.0 / config_.updateRateHz;
    nextTime_ = now + period;  // No catch-up burst after a large time step.
    try
    {
      const auto *spherical = ecm.Component<gz::sim::components::SphericalCoordinates>(world_);
      if (!spherical || !link_.Valid(ecm)) return;
      const auto &frame = spherical->Data();
      if (frame.Surface() != gz::math::SphericalCoordinates::EARTH_WGS84)
        throw std::invalid_argument("Only EARTH_WGS84 is supported");
      const auto pose = gz::sim::worldPose(link_.Entity(), ecm);
      const auto lla = gzsim::Geodetic(frame, pose.Pos() + pose.Rot().RotateVector(antennaOffset_));
      const auto worldVelocity = link_.WorldLinearVelocity(ecm, antennaOffset_);
      if (!worldVelocity && !staticModel_) return; // Physics has not populated velocity yet.
      const auto velocity = gzsim::VelocityEnu(frame, worldVelocity.value_or(gz::math::Vector3d::Zero), lla.X(), lla.Y());
      const auto fix = receiver_->Sample(now, lla.X(), lla.Y(), lla.Z(),
                                        {velocity.X(), velocity.Y(), velocity.Z()});
      if (!fix.scenario.outputEnabled) return;
      gz::msgs::NavSat message;
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      message.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
      message.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
      message.set_frame_id(gz::sim::scopedName(link_.Entity(), ecm, "::", false));
      message.set_latitude_deg(fix.latitudeDeg);
      message.set_longitude_deg(fix.longitudeDeg);
      message.set_altitude(fix.altitudeM);
      message.set_velocity_east(fix.velocityEnuMps.x);
      message.set_velocity_north(fix.velocityEnuMps.y);
      message.set_velocity_up(fix.velocityEnuMps.z);
      Metadata(message, "gnss.schema_version", 1);
      Metadata(message, "gnss.satellites_used", fix.satellitesUsed);
      Metadata(message, "gnss.satellites_visible", fix.satellitesVisible);
      Metadata(message, "gnss.fix_type", fix.fixType);
      Metadata(message, "gnss.eph", fix.ephM);
      Metadata(message, "gnss.epv", fix.epvM);
      Metadata(message, "gnss.hdop", fix.hdop);
      Metadata(message, "gnss.vdop", fix.vdop);
      Metadata(message, "gnss.speed_accuracy", fix.speedAccuracyMps);
      Metadata(message, "gnss.course_accuracy", fix.valid
        ? std::atan2(fix.speedAccuracyMps, std::hypot(fix.velocityEnuMps.x, fix.velocityEnuMps.y))
        : 3.14159265358979323846);
      Metadata(message, "gnss.time_utc_usec", static_cast<std::uint64_t>((config_.epochUnixSeconds + now) * 1e6));
      const bool reportSpoofing = config_.scenario.empty() ? reportSpoofing_ :
                                  fix.scenario.reportDetection;
      Metadata(message, "gnss.spoofing_state", reportSpoofing && fix.spoofing ? 3 : 0);
      Metadata(message, "gnss.jamming_state", fix.scenario.reportJamming &&
        (fix.quality == core::Quality::Denied || fix.scenario.dropout) ? 3 : 0);
      Metadata(message, "gnss.quality", core::QualityName(fix.quality));
      Metadata(message, "gnss.spoofing_active", fix.spoofing ? 1 : 0);
      Metadata(message, "gnss.status", fix.status);
      Metadata(message, "gnss.reacquiring", fix.reacquiring ? 1 : 0);
      Metadata(message, "gnss.scenario.event_index", fix.scenario.eventIndex);
      Metadata(message, "gnss.scenario.event_name", fix.scenario.eventName);
      Metadata(message, "gnss.scenario.elapsed_seconds", fix.scenario.eventElapsedSeconds);
      Metadata(message, "gnss.scenario.spoofing_level", fix.scenario.spoofingLevel);
      Metadata(message, "gnss.scenario.offset_east_m", fix.scenario.offsetEnuM.x);
      Metadata(message, "gnss.scenario.offset_north_m", fix.scenario.offsetEnuM.y);
      Metadata(message, "gnss.scenario.offset_up_m", fix.scenario.offsetEnuM.z);
      Metadata(message, "gnss.scenario.velocity_east_mps", fix.scenario.velocityOffsetEnuMps.x);
      Metadata(message, "gnss.scenario.velocity_north_mps", fix.scenario.velocityOffsetEnuMps.y);
      Metadata(message, "gnss.scenario.velocity_up_mps", fix.scenario.velocityOffsetEnuMps.z);
      Metadata(message, "gnss.scenario.transitioning", fix.scenario.transitioning ? 1 : 0);
      Metadata(message, "gnss.scenario.dropout", fix.scenario.dropout ? 1 : 0);
      Metadata(message, "gnss.scenario.noise_scale", fix.scenario.noiseScale);
      Metadata(message, "gnss.source", receiver_->Source());
      Metadata(message, "gnss.almanac_age_seconds", receiver_->AlmanacAgeSeconds(now));
      Metadata(message, "gnss.truth_latitude_deg", lla.X());
      Metadata(message, "gnss.truth_longitude_deg", lla.Y());
      Metadata(message, "gnss.truth_altitude_m", lla.Z());
      Metadata(message, "gnss.truth_velocity_east", velocity.X());
      Metadata(message, "gnss.truth_velocity_north", velocity.Y());
      Metadata(message, "gnss.truth_velocity_up", velocity.Z());
      publisher_.Publish(message);
    }
    catch (const std::exception &error)
    {
      gzerr << "[RealGpsSystem] Receiver disabled: " << error.what() << std::endl;
      enabled_ = false;
    }
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override { ResetState(); }

 private:
  void ResetState()
  {
    if (receiver_) receiver_->Reset();
    nextTime_ = 0;
    lastTime_ = -1;
  }
  core::Config config_;
  std::unique_ptr<core::Receiver> receiver_;
  gz::sim::Link link_;
  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity world_{gz::sim::kNullEntity};
  gz::math::Vector3d antennaOffset_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  std::string topic_;
  std::string sensorName_;
  bool enabled_{false};
  bool started_{false};
  bool staticModel_{false};
  bool reportSpoofing_{false};
  double nextTime_{0};
  double lastTime_{-1};
};
}

GZ_ADD_PLUGIN(real_gps::RealGpsSystem, gz::sim::System,
              real_gps::RealGpsSystem::ISystemConfigure,
              real_gps::RealGpsSystem::ISystemPreUpdate,
              real_gps::RealGpsSystem::ISystemPostUpdate,
              real_gps::RealGpsSystem::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(real_gps::RealGpsSystem, "real_gps::RealGpsSystem")
