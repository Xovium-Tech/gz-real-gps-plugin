#include "real_gps/core/Receiver.hh"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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

void NearVector(Vec3 actual, Vec3 expected, double tolerance, const std::string &message)
{
  Near(actual.x, expected.x, tolerance, message + " east");
  Near(actual.y, expected.y, tolerance, message + " north");
  Near(actual.z, expected.z, tolerance, message + " up");
}

void Invalid(const std::function<void()> &action, const std::string &message)
{
  try { action(); }
  catch (const std::invalid_argument &) { return; }
  throw std::runtime_error("Expected invalid_argument: " + message);
}

ScenarioEvent Event(double at, double level = 0, Vec3 offset = {200, 0, 0},
                    double transition = 0)
{
  ScenarioEvent event;
  event.atSeconds = at;
  event.spoofingLevel = level;
  event.spoofOffsetEnuM = offset;
  event.transitionSeconds = transition;
  return event;
}

Measurement Sample(Receiver &receiver, double time)
{
  return receiver.Sample(time, 52.23, 21.02, 100, {20, -5, 1});
}

Vec3 PositionDifference(const Measurement &a, const Measurement &b)
{
  const auto first = GeodeticToEcef(a.latitudeDeg, a.longitudeDeg, a.altitudeM);
  const auto second = GeodeticToEcef(b.latitudeDeg, b.longitudeDeg, b.altitudeM);
  return EcefVectorToEnu({first.x - second.x, first.y - second.y, first.z - second.z},
                         52.23, 21.02);
}

Vec3 Difference(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 Scaled(Vec3 value, double scale)
{
  return {value.x * scale, value.y * scale, value.z * scale};
}

void SameMeasurement(const Measurement &a, const Measurement &b)
{
  Near(a.latitudeDeg, b.latitudeDeg, 0, "Repeatable latitude");
  Near(a.longitudeDeg, b.longitudeDeg, 0, "Repeatable longitude");
  Near(a.altitudeM, b.altitudeM, 0, "Repeatable altitude");
  NearVector(a.velocityEnuMps, b.velocityEnuMps, 0, "Repeatable velocity");
  Near(a.ephM, b.ephM, 0, "Repeatable horizontal accuracy");
  Near(a.epvM, b.epvM, 0, "Repeatable vertical accuracy");
  Near(a.speedAccuracyMps, b.speedAccuracyMps, 0, "Repeatable speed accuracy");
  Near(a.hdop, b.hdop, 0, "Repeatable HDOP");
  Near(a.vdop, b.vdop, 0, "Repeatable VDOP");
  Check(a.fixType == b.fixType && a.valid == b.valid && a.quality == b.quality &&
        a.satellitesUsed == b.satellitesUsed && a.satellitesVisible == b.satellitesVisible &&
        a.status == b.status && a.reacquiring == b.reacquiring && a.spoofing == b.spoofing,
        "Repeatable receiver status");
  NearVector(a.scenario.offsetEnuM, b.scenario.offsetEnuM, 0, "Repeatable scenario offset");
  NearVector(a.scenario.velocityOffsetEnuMps, b.scenario.velocityOffsetEnuMps, 0,
             "Repeatable scenario velocity");
  Near(a.scenario.spoofingLevel, b.scenario.spoofingLevel, 0, "Repeatable severity");
  Near(a.scenario.noiseScale, b.scenario.noiseScale, 0, "Repeatable noise scale");
  Near(a.scenario.eventElapsedSeconds, b.scenario.eventElapsedSeconds, 0,
       "Repeatable event elapsed time");
  Check(a.scenario.eventIndex == b.scenario.eventIndex && a.scenario.eventName == b.scenario.eventName &&
        a.scenario.quality == b.scenario.quality && a.scenario.spoofing == b.scenario.spoofing &&
        a.scenario.transitioning == b.scenario.transitioning && a.scenario.dropout == b.scenario.dropout &&
        a.scenario.reportDetection == b.scenario.reportDetection &&
        a.scenario.reportJamming == b.scenario.reportJamming &&
        a.scenario.outputEnabled == b.scenario.outputEnabled, "Repeatable scenario diagnostics");
}

void ScenarioValidation()
{
  Check(Scenario{}.Empty(), "An omitted scenario is empty");
  Check(Scenario{}.Evaluate(1e9).eventIndex == -1, "An empty scenario stays inactive");
  Check(!Scenario({Event(0)}).Empty(), "An explicit healthy event is still a scenario");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<std::string, std::function<void(ScenarioEvent &)>>> invalid = {
    {"negative timestamp", [](auto &e) { e.atSeconds = -0.1; }},
    {"nonfinite timestamp", [=](auto &e) { e.atSeconds = nan; }},
    {"infinite timestamp", [=](auto &e) { e.atSeconds = inf; }},
    {"negative transition", [](auto &e) { e.transitionSeconds = -1; }},
    {"nonfinite transition", [=](auto &e) { e.transitionSeconds = nan; }},
    {"overflowing transition end", [](auto &e) {
      e.atSeconds = e.transitionSeconds = std::numeric_limits<double>::max();
    }},
    {"unrepresentable positive transition end", [](auto &e) {
      e.atSeconds = 1e16; e.transitionSeconds = 1;
    }},
    {"negative spoofing level", [](auto &e) { e.spoofingLevel = -0.01; }},
    {"spoofing level above one", [](auto &e) { e.spoofingLevel = 1.01; }},
    {"nonfinite spoofing level", [=](auto &e) { e.spoofingLevel = nan; }},
    {"nonfinite east offset", [=](auto &e) { e.spoofOffsetEnuM.x = nan; }},
    {"nonfinite north offset", [=](auto &e) { e.spoofOffsetEnuM.y = inf; }},
    {"nonfinite vertical offset", [=](auto &e) { e.spoofOffsetEnuM.z = -inf; }},
    {"unsupported offset magnitude", [](auto &e) { e.spoofOffsetEnuM = {1e7, 1e7, 0}; }},
    {"negative noise scale", [](auto &e) { e.noiseScale = -1; }},
    {"nonfinite noise scale", [=](auto &e) { e.noiseScale = nan; }},
    {"excessive noise scale", [](auto &e) { e.noiseScale = 101; }},
    {"negative dropout period", [](auto &e) { e.dropoutPeriodSeconds = -1; }},
    {"nonfinite dropout period", [=](auto &e) { e.dropoutPeriodSeconds = inf; }},
    {"nonfinite dropout duration", [=](auto &e) { e.dropoutDurationSeconds = nan; }},
    {"duration without period", [](auto &e) { e.dropoutDurationSeconds = 1; }},
    {"period without duration", [](auto &e) { e.dropoutPeriodSeconds = 1; }},
    {"duration exceeds period", [](auto &e) {
      e.dropoutPeriodSeconds = 1; e.dropoutDurationSeconds = 2;
    }},
    {"legacy spoofing as reception quality", [](auto &e) { e.quality = Quality::Spoofing; }},
    {"invalid reception enum", [](auto &e) { e.quality = static_cast<Quality>(999); }}
  };
  for (const auto &test : invalid)
  {
    auto event = Event(0);
    test.second(event);
    Invalid([&] { Scenario scenario({event}); }, test.first);
  }
  Invalid([] { Scenario scenario({Event(2), Event(1)}); }, "Unsorted events");
  Invalid([] { Scenario scenario({Event(1), Event(1)}); }, "Duplicate timestamp");
  Invalid([] { Scenario scenario({Event(1, 1, {}, 4), Event(4)}); }, "Overlapping ramps");
  auto first = Event(1), second = Event(2);
  first.name = second.name = "duplicate";
  Invalid([&] { Scenario scenario({first, second}); }, "Duplicate event name");
  const Scenario adjacent({Event(0.25, 1, {10, 0, 0}, 2), Event(2.25)});
  Check(adjacent.Evaluate(2.25).eventIndex == 1,
        "A new event may begin exactly when the previous transition ends");
  for (double time : {-1.0, nan, inf})
    Invalid([&] { adjacent.Evaluate(time); }, "Invalid simulation time");
}

void ScenarioFiniteTransitions()
{
  Invalid([] { Scenario scenario({Event(0, 1, {200, 0, 0}, 1e-309)}); },
          "Subnormal duration with nonfinite spoof velocity");
  for (const auto offset : {Vec3{200001, 0, 0}, {0, -200001, 0}, {0, 0, 200001}})
    Invalid([&] { Scenario scenario({Event(0, 1, offset, 3)}); },
            "Each spoof velocity component respects the bridge range");
  Invalid([] {
    Scenario scenario({Event(0, 1, {-200000, 0, 0}), Event(1, 1, {200000, 0, 0}, 3)});
  }, "Rate validation uses the change from the preceding target");
  Invalid([] {
    Scenario scenario({Event(0, 1, {200001, 0, 0}), Event(1, 0, {}, 3)});
  }, "Recovery velocity also respects the bridge range");
  const Scenario boundary({Event(0, 1, {200000, -200000, 200000}, 3)});
  NearVector(boundary.Evaluate(1.5).velocityOffsetEnuMps, {100000, -100000, 100000},
             1e-9, "Maximum allowed per-axis spoof velocity");

  // A reciprocal duration overflows here, but the actual rates are finite.
  const Scenario zero({Event(0, 0, {}, 1e-309)});
  NearVector(zero.Evaluate(5e-310).velocityOffsetEnuMps, {}, 0,
             "Zero displacement never becomes zero times infinity");
  const Scenario tiny({Event(0, 1, {1e-310, -1e-310, 0}, 1e-309)});
  NearVector(tiny.Evaluate(5e-310).velocityOffsetEnuMps, {0.15, -0.15, 0}, 1e-12,
             "Small finite rates survive subnormal durations");
  auto positionOnly = Event(0, 1, {200, 40, 20}, 1e-309);
  positionOnly.spoofVelocity = false;
  const auto sample = Scenario({positionOnly}).Evaluate(5e-310);
  NearVector(sample.offsetEnuM, {100, 20, 10}, 1e-9,
             "Position-only scenarios permit fast finite displacement transitions");
  NearVector(sample.velocityOffsetEnuMps, {}, 0,
             "Position-only scenarios do not evaluate a derivative");
}

void ScenarioTrajectory()
{
  const Scenario scenario({Event(0.25, 0.5, {200, 100, 20}, 4),
                           Event(10.25, 0.25, {-100, 200, -40}, 2),
                           Event(20.25, 0, {}, 4)});
  const auto before = scenario.Evaluate(0.249);
  Check(before.eventIndex == -1 && !before.spoofing && !before.transitioning,
        "Fractional onset must not activate early");
  NearVector(before.offsetEnuM, {}, 0, "No bias before onset");
  const auto onset = scenario.Evaluate(0.25);
  Check(onset.eventIndex == 0 && onset.transitioning, "Exact onset activates first ramp");
  NearVector(onset.offsetEnuM, {}, 0, "Position continuity at onset");
  NearVector(onset.velocityOffsetEnuMps, {}, 0, "Zero onset velocity");
  const auto quarter = scenario.Evaluate(1.25);
  NearVector(quarter.offsetEnuM, {15.625, 7.8125, 1.5625}, 1e-12,
             "Quarter-ramp smoothstep bias");
  NearVector(quarter.velocityOffsetEnuMps, {28.125, 14.0625, 2.8125}, 1e-12,
             "Quarter-ramp analytic velocity");
  Near(quarter.spoofingLevel, 0.078125, 1e-12, "Interpolated severity");
  const auto end = scenario.Evaluate(4.25);
  NearVector(end.offsetEnuM, {100, 50, 10}, 0, "Level scales the full target offset");
  NearVector(end.velocityOffsetEnuMps, {}, 0, "Zero end velocity");
  Check(end.spoofing && !end.transitioning, "Holding a nonzero bias remains spoofing");
  NearVector(scenario.Evaluate(10.25).offsetEnuM, {100, 50, 10}, 0,
             "Direction change starts from the prior target");
  const auto turn = scenario.Evaluate(11.25);
  NearVector(turn.offsetEnuM, {37.5, 50, 0}, 1e-12, "Changing direction and level");
  NearVector(turn.velocityOffsetEnuMps, {-93.75, 0, -15}, 1e-12,
             "Velocity follows the change between targets");
  Near(turn.spoofingLevel, 0.375, 1e-12, "Severity transitions between levels");
  const auto recovery = scenario.Evaluate(22.25);
  NearVector(recovery.offsetEnuM, {-12.5, 25, -5}, 1e-12, "Recovery removes the held bias");
  NearVector(recovery.velocityOffsetEnuMps, {9.375, -18.75, 3.75}, 1e-12,
             "Recovery velocity points back toward truth");
  Check(recovery.spoofing && recovery.transitioning, "Recovery remains marked as spoofed");
  const auto recovered = scenario.Evaluate(24.25);
  Check(!recovered.spoofing && !recovered.transitioning, "Recovery completes at exact end");
  NearVector(recovered.offsetEnuM, {}, 0, "Recovery ends at zero displacement");
  NearVector(recovered.velocityOffsetEnuMps, {}, 0, "Recovery ends at zero bias velocity");
  NearVector(scenario.Evaluate(1e9).offsetEnuM, {}, 0, "Large jumps find the final state");
  NearVector(scenario.Evaluate(1.25).offsetEnuM, quarter.offsetEnuM, 0,
             "Scenario evaluation is independent of call order");

  // Compare the velocity to a numerical derivative at arbitrary interior times.
  // This catches position/velocity inconsistency beyond the midpoint examples.
  for (double time : {0.75, 2.15, 10.75, 11.8, 21.25, 23.5})
  {
    constexpr double epsilon = 1e-5;
    const auto numerical = Scaled(Difference(scenario.Evaluate(time + epsilon).offsetEnuM,
        scenario.Evaluate(time - epsilon).offsetEnuM), 0.5 / epsilon);
    NearVector(scenario.Evaluate(time).velocityOffsetEnuMps, numerical, 1e-7,
               "Bias velocity must be the derivative of bias position");
  }
  const Scenario steps({Event(1.125, 1, {10, -20, 30}), Event(2.125)});
  NearVector(steps.Evaluate(1.125).offsetEnuM, {10, -20, 30}, 0, "Zero-duration attack step");
  NearVector(steps.Evaluate(1.125).velocityOffsetEnuMps, {}, 0,
             "Instantaneous steps have no fabricated finite velocity impulse");
  NearVector(steps.Evaluate(2.125).offsetEnuM, {}, 0, "Zero-duration recovery step");

  auto positionOnly = Event(0, 1, {80, 40, 20}, 4);
  positionOnly.spoofVelocity = false;
  const auto inconsistent = Scenario({positionOnly}).Evaluate(2);
  NearVector(inconsistent.offsetEnuM, {40, 20, 10}, 0, "Position-only attack still ramps");
  NearVector(inconsistent.velocityOffsetEnuMps, {}, 0, "Position-only attack leaves velocity alone");
}

void ScenarioStateAndDropouts()
{
  auto attack = Event(17.25, 0.5, {100, 0, 0});
  attack.name = "intermittent";
  attack.quality = Quality::Poor;
  attack.noiseScale = 4;
  attack.reportDetection = attack.reportJamming = true;
  attack.outputEnabled = false;
  attack.dropoutPeriodSeconds = 4;
  attack.dropoutDurationSeconds = 1.5;
  auto clear = Event(30);
  clear.name = "clear";
  const Scenario scenario({attack, clear});
  for (const auto &test : std::vector<std::pair<double, bool>>{
         {17.25, true}, {18.749, true}, {18.75, false}, {21.249, false},
         {21.25, true}, {22.75, false}, {29.25, true}, {30, false}})
    Check(scenario.Evaluate(test.first).dropout == test.second,
          "Dropout phase is relative to event onset at t=" + std::to_string(test.first));
  const auto active = scenario.Evaluate(18.25);
  Check(active.eventName == "intermittent" && active.quality == Quality::Poor &&
        active.reportDetection && active.reportJamming && active.spoofing && !active.outputEnabled,
        "Active event carries its complete state");
  Near(active.eventElapsedSeconds, 1, 0, "Event elapsed time");
  const auto cleared = scenario.Evaluate(30);
  Check(cleared.eventName == "clear" && !cleared.quality && !cleared.reportDetection &&
        !cleared.reportJamming && !cleared.spoofing && !cleared.dropout && cleared.outputEnabled,
        "A new event resets omitted target fields instead of inheriting them");
  Near(cleared.noiseScale, 1, 0, "Omitted noise scale restores the default");
  auto noise = Event(0, 0, {}, 4);
  noise.noiseScale = 4;
  Near(Scenario({noise}).Evaluate(2).noiseScale, 2.5, 0, "Noise scale follows transition");
  attack.dropoutDurationSeconds = attack.dropoutPeriodSeconds;
  Check(Scenario({attack}).Evaluate(1e6).dropout,
        "Equal period and duration consistently denies reception");
}

void ReceiverQualityAndCompatibility()
{
  Config config;
  config.quality = Quality::Excellent;
  config.zones.push_back({52.23, 21.02, 100, 0, 200, Quality::Denied});
  auto overrideZone = Event(2, 0.5);
  overrideZone.quality = Quality::Normal;
  config.scenario = {Event(1), overrideZone, Event(3)};
  Receiver receiver(config);
  for (double time : {0.0, 1.0})
  {
    const auto fix = Sample(receiver, time);
    Check(!fix.valid && fix.quality == Quality::Denied && fix.status == "denied",
          "Zone reception applies before events and when event quality is omitted");
  }
  const auto explicitQuality = Sample(receiver, 2);
  Check(explicitQuality.valid && explicitQuality.quality == Quality::Normal,
        "Explicit event reception quality overrides the matching zone");
  const auto restored = Sample(receiver, 3);
  Check(!restored.valid && restored.quality == Quality::Denied && !restored.spoofing,
        "Omitted quality returns to the zone and omitted spoofing level clears the attack");
  const auto outside = receiver.Sample(4, 0, 0, 100, {});
  Check(outside.valid && outside.quality == Quality::Excellent,
        "Outside zones an omitted event quality uses the receiver base quality");

  // A reception-quality override must not erase a physical zone elevation mask.
  config.zones[0].elevationMaskDeg = 90;
  Receiver masked(config);
  const auto blocked = Sample(masked, 2);
  Check(!blocked.valid && blocked.quality == Quality::Normal && blocked.spoofing &&
        blocked.status == "insufficient_geometry", "Geometry remains effective during an attack");

  config = Config{};
  config.quality = Quality::Spoofing;
  config.scenario = {Event(100)};
  Invalid([&] { Receiver invalid(config); }, "Legacy base spoofing mixed with timeline");
  config.quality = Quality::Normal;
  config.zones.push_back({52.23, 21.02, 100, 0, 200, Quality::Spoofing});
  Invalid([&] { Receiver invalid(config); }, "Legacy zone spoofing mixed with timeline");
  config.scenario.clear();
  Receiver legacyZone(config);
  Check(Sample(legacyZone, 100).spoofing, "Legacy spoofing zones remain supported without a timeline");

  config = Config{};
  Receiver normal(config);
  config.scenario = {Event(100, 1, {100, 0, 0}, 10)};
  Receiver futureAttack(config);
  for (double time : {0.0, 0.2, 2.5, 99.5})
    SameMeasurement(Sample(normal, time), Sample(futureAttack, time));
}

void ReceiverAttackAndNoise()
{
  Config config;
  Receiver clean(config);
  config.scenario = {Event(0.25, 0.5, {200, 100, 20}, 4), Event(10.25, 0, {}, 4)};
  Receiver attack(config);
  config.scenario[0].spoofVelocity = false;
  config.scenario[1].spoofVelocity = false;
  Receiver positionOnly(config);
  struct Expected { double time; Vec3 position; Vec3 velocity; };
  const std::vector<Expected> expected = {
    {0, {}, {}}, {0.25, {}, {}}, {1.25, {15.625, 7.8125, 1.5625}, {28.125, 14.0625, 2.8125}},
    {4.25, {100, 50, 10}, {}}, {12.25, {50, 25, 5}, {-37.5, -18.75, -3.75}},
    {14.25, {}, {}}
  };
  for (const auto &sample : expected)
  {
    const auto reference = Sample(clean, sample.time);
    const auto altered = Sample(attack, sample.time);
    const auto inconsistent = Sample(positionOnly, sample.time);
    Check(altered.valid && altered.status == "ok", "Spoofing keeps a usable claimed fix");
    NearVector(PositionDifference(altered, reference), sample.position, 1e-6,
               "Receiver applies the scenario's displacement");
    NearVector(Difference(altered.velocityEnuMps, reference.velocityEnuMps), sample.velocity,
               1e-10, "Receiver applies coherent scenario velocity");
    NearVector(PositionDifference(inconsistent, reference), sample.position, 1e-6,
               "Position-only receiver applies the same displacement");
    NearVector(inconsistent.velocityEnuMps, reference.velocityEnuMps, 0,
               "Position-only receiver does not alter velocity");
    Near(altered.ephM, reference.ephM, 0, "Undetected attack keeps receiver accuracy claims");
    Near(altered.speedAccuracyMps, reference.speedAccuracyMps, 0,
         "Undetected attack keeps speed accuracy claims");
  }

  config = Config{};
  Receiver unitNoise(config);
  auto noise = Event(0);
  noise.noiseScale = 3;
  config.scenario = {noise};
  Receiver tripleNoise(config);
  config.scenario[0].noiseScale = 0;
  Receiver noNoise(config);
  config.scenario[0].noiseScale = 1e-4;
  Receiver tinyNoise(config);
  Measurement truth;
  truth.latitudeDeg = 52.23; truth.longitudeDeg = 21.02; truth.altitudeM = 100;
  truth.velocityEnuMps = {20, -5, 1};
  for (double time : {0.0, 0.2, 1.5, 5.0})
  {
    const auto unit = Sample(unitNoise, time), triple = Sample(tripleNoise, time);
    const auto zero = Sample(noNoise, time);
    const auto tiny = Sample(tinyNoise, time);
    NearVector(PositionDifference(triple, truth), Scaled(PositionDifference(unit, truth), 3),
               1e-6, "Noise scale multiplies actual position error");
    NearVector(Difference(triple.velocityEnuMps, truth.velocityEnuMps),
               Scaled(Difference(unit.velocityEnuMps, truth.velocityEnuMps), 3), 1e-12,
               "Noise scale multiplies actual velocity error");
    Near(triple.ephM, 3 * unit.ephM, 1e-12, "Noise scale multiplies horizontal uncertainty");
    Near(triple.epvM, 3 * unit.epvM, 1e-12, "Noise scale multiplies vertical uncertainty");
    Near(triple.speedAccuracyMps, 3 * unit.speedAccuracyMps, 1e-12,
         "Noise scale multiplies speed uncertainty");
    Near(triple.hdop, unit.hdop, 0, "Noise scale does not misrepresent satellite geometry");
    NearVector(PositionDifference(zero, truth), {}, 1e-6, "Zero scale removes random position noise");
    NearVector(zero.velocityEnuMps, truth.velocityEnuMps, 0, "Zero scale removes random velocity noise");
    Check(zero.valid && tiny.valid, "Zero and very small noise scales retain valid fixes");
    for (const auto &fix : {zero, tiny})
    {
      Near(fix.ephM, 0.01, 0, "Low-noise horizontal accuracy retains a positive reporting floor");
      Near(fix.epvM, 0.01, 0, "Low-noise vertical accuracy retains a positive reporting floor");
      Near(fix.speedAccuracyMps, 0.001, 0,
           "Low-noise speed accuracy retains a positive reporting floor");
    }
    NearVector(PositionDifference(tiny, truth), Scaled(PositionDifference(unit, truth), 1e-4),
               1e-6, "Reported accuracy floor does not inflate physical position noise");
    NearVector(Difference(tiny.velocityEnuMps, truth.velocityEnuMps),
               Scaled(Difference(unit.velocityEnuMps, truth.velocityEnuMps), 1e-4), 1e-12,
               "Reported accuracy floor does not inflate physical velocity noise");
  }
}

void ReceiverDenialAndReacquisition()
{
  Config config;
  auto denied = Event(1, 0.75, {200, 0, 0});
  denied.quality = Quality::Denied;
  denied.reportDetection = denied.reportJamming = true;
  config.scenario = {denied, Event(2)};
  config.reacquisitionSeconds = 3;
  Receiver receiver(config);
  const auto healthy = Sample(receiver, 0);
  Check(healthy.valid && healthy.status == "ok", "Healthy startup has no outage reacquisition delay");
  const auto outage = Sample(receiver, 1);
  Check(!outage.valid && outage.fixType == 1 && outage.satellitesUsed == 0 &&
        outage.status == "denied" && outage.spoofing && outage.scenario.reportDetection &&
        outage.scenario.reportJamming, "Denied samples retain scenario truth and reporting controls");
  NearVector(outage.scenario.offsetEnuM, {150, 0, 0}, 0, "Attack truth evaluated despite denial");
  NearVector(PositionDifference(outage, healthy), {}, 1e-6, "No-fix output holds its last position");
  // No samples occurred between t=1 and t=100: recovery must begin when usable
  // observations actually return, not at an unobserved nominal event boundary.
  const auto firstUsable = Sample(receiver, 100);
  Check(!firstUsable.valid && firstUsable.reacquiring && firstUsable.status == "reacquiring",
        "First usable post-outage sample starts reacquisition");
  SameMeasurement(firstUsable, Sample(receiver, 100));
  Check(!Sample(receiver, 102.999).valid, "Reacquisition suppresses fixes until its deadline");
  const auto recovered = Sample(receiver, 103);
  Check(recovered.valid && !recovered.reacquiring && recovered.status == "ok",
        "Fix returns at exact reacquisition deadline");

  auto again = Event(2.5);
  again.quality = Quality::Denied;
  config.scenario = {denied, Event(2), again, Event(3)};
  Receiver interrupted(config);
  Sample(interrupted, 0); Sample(interrupted, 1);
  Check(Sample(interrupted, 2.2).reacquiring, "Recovery starts after first outage");
  Check(Sample(interrupted, 2.5).status == "denied", "New outage interrupts recovery");
  Check(Sample(interrupted, 3.25).reacquiring, "Returning reception restarts the full recovery delay");
  Check(!Sample(interrupted, 6.249).valid && Sample(interrupted, 6.25).valid,
        "Interrupted recovery uses its new deadline");

  config = Config{};
  auto intermittent = Event(1.25, 1);
  intermittent.dropoutPeriodSeconds = 4;
  intermittent.dropoutDurationSeconds = 1;
  config.scenario = {intermittent, Event(10)};
  Receiver dropout(config);
  Check(Sample(dropout, 0).valid, "Receiver fixes before periodic outage begins");
  for (double time : {1.25, 5.25})
  {
    const auto nofix = Sample(dropout, time);
    Check(!nofix.valid && nofix.status == "dropout" && nofix.scenario.dropout && nofix.spoofing,
          "Periodic outage retains active attack diagnostics");
    Check(Sample(dropout, time + 1).valid, "Dropout ends at its exact duration boundary");
  }
  Check(!Sample(dropout, 9.25).valid && Sample(dropout, 10).valid,
        "New event removes old dropout configuration");

  auto blackout = Event(1.25, 0.5);
  blackout.outputEnabled = false;
  config.scenario = {blackout, Event(3.5)};
  Receiver silent(config);
  const auto suppressed = Sample(silent, 1.25);
  Check(suppressed.valid && suppressed.spoofing && !suppressed.scenario.outputEnabled,
        "Output blackout preserves internal receiver observations for diagnostic scoring");
  const auto publishing = Sample(silent, 3.5);
  Check(publishing.valid && !publishing.spoofing && publishing.scenario.outputEnabled,
        "New event restores publication by default");
  for (double delay : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()})
  {
    config.reacquisitionSeconds = delay;
    Invalid([&] { Receiver invalid(config); }, "Invalid reacquisition interval");
  }
}

void ReceiverAbsence()
{
  Config config;
  config.reacquisitionSeconds = 0.5;
  auto disconnect = Event(1);
  disconnect.quality = Quality::None;
  // An enabled output flag cannot manufacture a missing receiver.
  disconnect.outputEnabled = true;
  config.scenario = {disconnect, Event(2)};
  Receiver receiver(config);
  Check(Sample(receiver, 0).valid, "Receiver fixes before disconnect");
  for (double time : {1.0, 1.2, 1.999})
  {
    const auto missing = Sample(receiver, time);
    Check(missing.quality == Quality::None && !missing.scenario.outputEnabled &&
          !missing.valid && missing.fixType == 0 && missing.satellitesUsed == 0 &&
          missing.status == "receiver_absent",
          "None event suppresses observations even when output_enabled is true");
  }
  const auto reconnect = Sample(receiver, 2);
  Check(reconnect.scenario.outputEnabled && reconnect.quality == Quality::Normal &&
        reconnect.reacquiring && !reconnect.valid && reconnect.fixType == 1,
        "Reconnected receiver publishes no-fix while reacquiring");
  Check(!Sample(receiver, 2.499).valid && Sample(receiver, 2.5).valid,
        "Reconnected receiver obeys the configured reacquisition delay");
  // Rewind must reset the disconnected/reacquisition state along with the clock.
  Check(Sample(receiver, 0).valid, "Simulation rewind restores the healthy initial receiver");

  config = Config{};
  config.quality = Quality::None;
  auto connect = Event(1);
  connect.quality = Quality::Normal;
  config.scenario = {Event(0), connect, Event(2)};
  Receiver initiallyAbsent(config);
  Check(!Sample(initiallyAbsent, 0).scenario.outputEnabled,
        "An event without quality cannot enable an absent base receiver");
  Check(Sample(initiallyAbsent, 1).valid &&
        Sample(initiallyAbsent, 1).scenario.outputEnabled,
        "An explicit normal event connects a receiver when base quality is none");
  Check(!Sample(initiallyAbsent, 2).scenario.outputEnabled,
        "Omitted event quality returns to the absent base receiver");

  config.quality = Quality::Normal;
  config.zones.push_back({52.23, 21.02, 100, 0, 200, Quality::None});
  Receiver zone(config);
  Check(!Sample(zone, 0).scenario.outputEnabled,
        "A none zone disconnects the receiver at the current location");
  const auto zoneOverride = Sample(zone, 1);
  Check(zoneOverride.valid && zoneOverride.scenario.outputEnabled,
        "Explicit normal event overrides a none zone");
  Check(!Sample(zone, 2).scenario.outputEnabled,
        "Omitted event quality restores the matching none zone");
  const auto outside = zone.Sample(3, 0, 0, 100, {});
  Check(outside.valid && outside.scenario.outputEnabled,
        "Leaving a none zone restores receiver output");
}

void ReplayAndStaleDiagnostics()
{
  Config config;
  auto denied = Event(3, 0.75);
  denied.quality = Quality::Denied;
  config.scenario = {Event(0.5, 0.5, {100, -50, 10}, 2), denied, Event(5)};
  config.reacquisitionSeconds = 1;
  const std::vector<double> times{0, 0.5, 1.5, 2.5, 3, 4, 5, 5.5, 6, 6.2};
  Receiver replay(config);
  std::vector<Measurement> original;
  for (double time : times) original.push_back(Sample(replay, time));
  replay.Reset();
  for (std::size_t i = 0; i < times.size(); ++i)
    SameMeasurement(original[i], Sample(replay, times[i]));
  // The first sample now moves time backwards and must reset both stochastic
  // receiver state and outage/reacquisition state.
  for (std::size_t i = 0; i < times.size(); ++i)
    SameMeasurement(original[i], Sample(replay, times[i]));

  const auto path = std::filesystem::current_path() / "scenario-test.yuma";
  struct Cleanup
  {
    std::filesystem::path path;
    ~Cleanup() { std::error_code error; std::filesystem::remove(path, error); }
  } cleanup{path};
  config = Config{};
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
  Check(file.good(), "Temporary test almanac must be written");
  config.almanacPath = path.string();
  config.maxAlmanacAgeDays = 1;
  config.scenario = {Event(100, 1, {100, 0, 0})};
  Receiver stale(config);
  Check(Sample(stale, 0).valid, "Fresh test almanac yields a fix");
  const auto fix = Sample(stale, 86401);
  Check(!fix.valid && fix.status == "stale_almanac" && fix.spoofing && fix.scenario.eventIndex == 0,
        "Stale-almanac diagnostics and attack truth survive early no-fix return");
}
}  // namespace

int main()
{
  try
  {
    ScenarioValidation();
    ScenarioFiniteTransitions();
    ScenarioTrajectory();
    ScenarioStateAndDropouts();
    ReceiverQualityAndCompatibility();
    ReceiverAttackAndNoise();
    ReceiverDenialAndReacquisition();
    ReceiverAbsence();
    ReplayAndStaleDiagnostics();
    std::cout << "Timed scenario validation, trajectories, receiver faults and replay passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
