#include "real_gps/core/Scenario.hh"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace real_gps::core
{
namespace
{
bool Nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }
Vec3 Target(const ScenarioEvent &event)
{
  return {event.spoofingLevel * event.spoofOffsetEnuM.x,
          event.spoofingLevel * event.spoofOffsetEnuM.y,
          event.spoofingLevel * event.spoofOffsetEnuM.z};
}
}

Scenario::Scenario(std::vector<ScenarioEvent> events) : events_(std::move(events))
{
  std::set<std::string> names;
  for (std::size_t i = 0; i < events_.size(); ++i)
  {
    auto &event = events_[i];
    if (event.name.empty()) event.name = "event_" + std::to_string(i);
    if (!names.insert(event.name).second)
      throw std::invalid_argument("Duplicate scenario event name: " + event.name);
    if (!Nonnegative(event.atSeconds) || !Nonnegative(event.transitionSeconds) ||
        !std::isfinite(event.atSeconds + event.transitionSeconds) ||
        (event.transitionSeconds > 0.0 &&
         event.atSeconds + event.transitionSeconds <= event.atSeconds) ||
        !Nonnegative(event.spoofingLevel) || event.spoofingLevel > 1.0 ||
        !Nonnegative(event.noiseScale) || event.noiseScale > 100.0 ||
        !std::isfinite(event.spoofOffsetEnuM.x) ||
        !std::isfinite(event.spoofOffsetEnuM.y) ||
        !std::isfinite(event.spoofOffsetEnuM.z) ||
        std::hypot(event.spoofOffsetEnuM.x, event.spoofOffsetEnuM.y,
                   event.spoofOffsetEnuM.z) > 1e7 ||
        !Nonnegative(event.dropoutPeriodSeconds) ||
        !Nonnegative(event.dropoutDurationSeconds) ||
        event.dropoutDurationSeconds > event.dropoutPeriodSeconds ||
        ((event.dropoutPeriodSeconds == 0.0) != (event.dropoutDurationSeconds == 0.0)))
      throw std::invalid_argument("Invalid scenario event: " + event.name);
    if (event.quality)
    {
      QualityName(*event.quality);
      if (*event.quality == Quality::Spoofing)
        throw std::invalid_argument("Scenario reception quality must not be spoofing; use spoofing_level");
    }
    if (i > 0 && (event.atSeconds <= events_[i - 1].atSeconds ||
        event.atSeconds < events_[i - 1].atSeconds + events_[i - 1].transitionSeconds))
      throw std::invalid_argument("Scenario timestamps must increase and transitions must not overlap");
    if (event.spoofVelocity && event.transitionSeconds > 0.0)
    {
      const auto prior = i == 0 ? Vec3{} : Target(events_[i - 1]);
      const auto target = Target(event);
      // Smoothstep's maximum derivative is 1.5 / duration. Bound each
      // generated velocity component to the PX4 bridge's numeric range and
      // reject overflowing rates before a sample can contain infinity or NaN.
      const double largestChange = std::max({std::abs(target.x - prior.x),
          std::abs(target.y - prior.y), std::abs(target.z - prior.z)});
      if (largestChange / event.transitionSeconds > 100000.0 / 1.5)
        throw std::invalid_argument("Scenario transition exceeds 100000 m/s spoof velocity: " +
                                    event.name);
    }
  }
}

ScenarioState Scenario::Evaluate(double seconds) const
{
  if (!Nonnegative(seconds)) throw std::invalid_argument("Invalid scenario time");
  ScenarioState state;
  const auto next = std::upper_bound(events_.begin(), events_.end(), seconds,
      [](double time, const ScenarioEvent &event) { return time < event.atSeconds; });
  if (next == events_.begin()) return state;
  const auto index = static_cast<std::size_t>(next - events_.begin() - 1);
  const auto &event = events_[index];
  const double elapsed = seconds - event.atSeconds;
  const double u = event.transitionSeconds == 0.0 ? 1.0 :
      std::clamp(elapsed / event.transitionSeconds, 0.0, 1.0);
  const double fraction = u * u * (3.0 - 2.0 * u);
  const auto prior = index == 0 ? Vec3{} : Target(events_[index - 1]);
  const auto target = Target(event);
  const Vec3 delta{target.x - prior.x, target.y - prior.y, target.z - prior.z};
  const double priorLevel = index == 0 ? 0.0 : events_[index - 1].spoofingLevel;
  const double priorNoise = index == 0 ? 1.0 : events_[index - 1].noiseScale;
  state.eventIndex = static_cast<int>(index);
  state.eventName = event.name;
  state.eventElapsedSeconds = elapsed;
  state.quality = event.quality;
  state.spoofingLevel = priorLevel + fraction * (event.spoofingLevel - priorLevel);
  state.noiseScale = priorNoise + fraction * (event.noiseScale - priorNoise);
  state.offsetEnuM = {prior.x + delta.x * fraction, prior.y + delta.y * fraction,
                     prior.z + delta.z * fraction};
  if (event.spoofVelocity && u > 0.0 && u < 1.0)
  {
    const double factor = 6.0 * u * (1.0 - u);
    // Dividing the displacement first keeps zero/tiny displacement finite
    // even for a subnormal duration whose reciprocal would overflow.
    state.velocityOffsetEnuMps = {delta.x / event.transitionSeconds * factor,
        delta.y / event.transitionSeconds * factor, delta.z / event.transitionSeconds * factor};
  }
  state.transitioning = u < 1.0;
  // Keep the recovery ramp classified as spoofed until its bias reaches zero.
  state.spoofing = state.spoofingLevel > 0.0;
  state.reportDetection = event.reportDetection;
  state.reportJamming = event.reportJamming;
  state.outputEnabled = event.outputEnabled;
  if (event.dropoutPeriodSeconds > 0.0)
    state.dropout = std::fmod(elapsed, event.dropoutPeriodSeconds) < event.dropoutDurationSeconds;
  return state;
}
}  // namespace real_gps::core
