#pragma once

#include <optional>
#include <string>
#include <vector>

#include "real_gps/core/Geometry.hh"

namespace real_gps::core
{
enum class Quality { Excellent, Normal, Poor, Spoofing, Denied, None };
Quality ParseQuality(const std::string &name);
std::string QualityName(Quality quality);

// Each event is a complete target state, held until the next event. An omitted
// quality uses the receiver/zone quality. Times are world simulation seconds.
struct ScenarioEvent
{
  std::string name;
  double atSeconds{0.0};
  double transitionSeconds{0.0};
  std::optional<Quality> quality;
  double spoofingLevel{0.0};
  Vec3 spoofOffsetEnuM{200.0, 0.0, 0.0};
  bool spoofVelocity{true};
  bool reportDetection{false};
  bool reportJamming{false};
  // Suppress NavSat publication to exercise autopilot sensor-timeout handling.
  bool outputEnabled{true};
  double noiseScale{1.0};
  // Periodic no-fix intervals, starting at this event's timestamp. Both zero
  // disables dropouts. Denied reports no fix; None suppresses receiver output.
  double dropoutPeriodSeconds{0.0};
  double dropoutDurationSeconds{0.0};
};

struct ScenarioState
{
  int eventIndex{-1};
  std::string eventName;
  double eventElapsedSeconds{0.0};
  std::optional<Quality> quality;
  double spoofingLevel{0.0};
  Vec3 offsetEnuM;
  Vec3 velocityOffsetEnuMps;
  bool spoofing{false};
  bool transitioning{false};
  bool dropout{false};
  bool reportDetection{false};
  bool reportJamming{false};
  bool outputEnabled{true};
  double noiseScale{1.0};
};

class Scenario
{
 public:
  explicit Scenario(std::vector<ScenarioEvent> events = {});
  ScenarioState Evaluate(double simulationSeconds) const;
  bool Empty() const { return events_.empty(); }

 private:
  std::vector<ScenarioEvent> events_;
};
}  // namespace real_gps::core
