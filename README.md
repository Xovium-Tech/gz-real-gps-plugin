# Real GPS Plugin

[![CI](https://github.com/Xovium-Tech/gz-real-gps-plugin/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/Xovium-Tech/gz-real-gps-plugin/actions/workflows/build.yml)
[![Latest release](https://img.shields.io/github/v/release/Xovium-Tech/gz-real-gps-plugin)](https://github.com/Xovium-Tech/gz-real-gps-plugin/releases/latest)

GPS receiver simulation for **Gazebo Harmonic and Jetty**, including PX4 fixed-wing aircraft. Satellite count, accuracy and fix availability change with aircraft location, simulation time, satellite geometry and reception quality. Modes include poor reception, loss of GPS and spoofing.

[![Real GPS plugin demo](https://raw.githubusercontent.com/Xovium-Tech/gz-real-gps-plugin/main/docs/images/gps-demo.gif)](https://raw.githubusercontent.com/Xovium-Tech/gz-real-gps-plugin/main/docs/images/gps-demo.gif)

**Source-only distribution:** download the release ZIP and compile it on the
computer running Gazebo. The archive contains source, headers, CMake files,
`build.sh`, the Python runtime helpers, this README, and LICENSE. It excludes
docs, images, videos, examples, tests, CI files, binaries, build outputs, and caches. GitHub's automatic
source ZIP/tar.gz downloads use the same exclusions. Python helpers download
satellite orbit data and connect the quality output to PX4/QGroundControl.

## Supported Gazebo versions

| Build target | Simulator | Documented build environment |
| --- | --- | --- |
| `harmonic` | Gazebo Harmonic / gz-sim8 | Ubuntu 24.04 |
| `jetty` | Gazebo Jetty / gz-sim10 | Ubuntu 24.04 |

Choose the target matching your installed simulator and PX4 setup. Each target
uses a separate build directory and produces one plugin library specific to that
Gazebo version.

## 1. Install dependencies

Install the chosen simulator using its official instructions:
[Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) or
[Jetty](https://gazebosim.org/docs/jetty/install_ubuntu/).

Install the common build dependencies:

```bash
sudo apt update
sudo apt install build-essential cmake git python3
```

Then install the development packages for **your chosen target only**, with the
Gazebo package repository configured:

```bash
# Harmonic / Ubuntu 24.04
sudo apt install libgz-sim8-dev libgz-plugin2-dev libgz-transport13-dev

# Jetty / Ubuntu 24.04
sudo apt install libgz-sim10-dev libgz-plugin4-dev libgz-transport15-dev
```

## 2. Get the source and build

```bash
git clone --depth 1 https://github.com/Xovium-Tech/gz-real-gps-plugin.git
cd gz-real-gps-plugin
```

Alternatively, download the source ZIP from the
[latest release](https://github.com/Xovium-Tech/gz-real-gps-plugin/releases/latest),
extract it and open a terminal in the extracted directory.

Run **one** command for your simulator:

```bash
JOBS=2 ./build.sh harmonic
JOBS=2 ./build.sh jetty
```

The helper builds in Release mode; its default target is `harmonic`. The output
is `build-harmonic/libRealGpsSystem.so` or `build-jetty/libRealGpsSystem.so`.
Increase `JOBS` if you have enough RAM for more parallel compilation.

Equivalent CMake commands, using Jetty as an example:

```bash
cmake -S . -B build-jetty -DGZ_DISTRO=jetty -DCMAKE_BUILD_TYPE=Release
cmake --build build-jetty --parallel 2
```

Replace `jetty` in both places with `harmonic` for a Harmonic build.

## 3. Load the plugin

The build helper prints the required search path. From the plugin directory,
set it in the terminal where you launch Gazebo or PX4:

```bash
# Change build-harmonic to build-jetty for Jetty.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build-harmonic${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

For a launch script, use the absolute plugin build directory instead of `$PWD`.

Set the world's `<spherical_coordinates>` to `EARTH_WGS84` with the same latitude, longitude and elevation as your terrain. Remove the aircraft's original `<sensor name="navsat_sensor" type="navsat">`, including one inherited from a base model. Add this block directly inside the aircraft `<model>` containing `base_link`:

```xml
<plugin filename="libRealGpsSystem.so" name="real_gps::RealGpsSystem">
  <link_name>base_link</link_name>
  <sensor_name>navsat_sensor</sensor_name>
  <quality>normal</quality>
  <update_rate>5</update_rate>
  <seed>42</seed>
  <elevation_mask_deg>10</elevation_mask_deg>
  <antenna_offset>0 0 0.15</antenna_offset>
</plugin>
```

The default topic is `/world/WORLD/model/MODEL/link/base_link/sensor/navsat_sensor/navsat`, matching PX4's standard subscription. Use `<topic>` to override it for custom or nested models. `antenna_offset` is in link-frame metres; `update_rate` is Hz. Restart the simulation after editing the SDF.

## Reception quality

| `<quality>` | Behaviour |
| --- | --- |
| `excellent` | Low noise: 0.45 m range-error sigma before satellite geometry. |
| `normal` | 1.5 m range-error sigma, correlated position drift and velocity noise. |
| `poor` | 6 m range-error sigma and a further 15° elevation mask; fewer satellites and possible fix loss. |
| `spoofing` | Gradually displaced position and matching velocity error, with normal reported uncertainty. |
| `denied` | Zero satellites, no fix and invalid velocity. |

Accuracy depends on satellite geometry; these values are not fixed position-accuracy outputs. Fewer than four usable satellites produces no fix. Aliases `gps_spoofing`, `gps-spoofing` and `jamming` are also accepted.

The default synthetic constellation is tested with `normal` reception against
PX4's PDOP limit of 2.5 for a full simulated day at McMillan, Warsaw, Zurich and
the equator, using the default 10-degree elevation mask. It also meets the default
satellite-count and reported-accuracy limits in those checks. Geometry is shared
by `normal`, `excellent` and `spoofing`; quality changes the receiver noise.
Other locations, higher masks, degradation zones or real almanacs can still
produce poor geometry. PX4's stationary-drift and other navigation checks remain
independent, so a 3D receiver fix alone does not guarantee navigation or home readiness.

For `spoofing`, add this inside the plugin:

```xml
<spoofing>
  <start_seconds>30</start_seconds>
  <ramp_seconds>60</ramp_seconds>
  <offset_enu_m>200 100 30</offset_enu_m>
  <report_detection>false</report_detection>
</spoofing>
```

The offset ramps to 200 m east, 100 m north and 30 m up. Timing starts at world simulation time zero. Set `report_detection` to `true` to simulate a receiver reporting the attack; otherwise PX4's spoofing state remains unknown.

To change quality by location, add one or more zones inside the plugin:

```xml
<zone>
  <latitude_deg>52.2300</latitude_deg>
  <longitude_deg>21.0200</longitude_deg>
  <radius_m>500</radius_m>
  <min_altitude_m>0</min_altitude_m>
  <max_altitude_m>300</max_altitude_m>
  <quality>poor</quality>
</zone>
```

The last matching zone wins. Altitudes are absolute in the world's height datum, not above terrain. Outside all zones, the base quality applies. Zones can also set `elevation_mask_deg`; spoofing zones use the global spoofing schedule.

## Timestamped spoofing and GPS-denied scenarios

Use a `<scenario>` inside the plugin to run a repeatable sequence without editing
the model during flight. Reception quality and spoofing are independent: a
receiver can report normal accuracy while its position is being displaced, or
experience spoofing together with poor reception or an outage.

```xml
<quality>normal</quality>
<seed>42</seed>
<reacquisition_seconds>2</reacquisition_seconds>
<scenario>
  <event name="healthy">
    <at_seconds>0</at_seconds>
  </event>
  <event name="low_spoof">
    <at_seconds>30</at_seconds>
    <transition_seconds>20</transition_seconds>
    <spoofing_level>0.25</spoofing_level>
    <offset_enu_m>200 100 20</offset_enu_m>
  </event>
  <event name="high_spoof">
    <at_seconds>70</at_seconds>
    <transition_seconds>20</transition_seconds>
    <spoofing_level>1</spoofing_level>
    <offset_enu_m>200 100 20</offset_enu_m>
  </event>
  <event name="recovery">
    <at_seconds>110</at_seconds>
    <transition_seconds>20</transition_seconds>
  </event>
  <event name="denied">
    <at_seconds>150</at_seconds>
    <quality>denied</quality>
  </event>
  <event name="restored">
    <at_seconds>180</at_seconds>
  </event>
</scenario>
```

`spoofing_level` is a fraction from 0 to 1 of that event's `offset_enu_m`, rather
than an RF power setting. Here low spoofing targets 50 m east, 25 m north and 5 m
up; high spoofing targets 200 m east, 100 m north and 20 m up. Increase the offset
to test larger displacements. The level, applied offset and noise scale change
smoothly over `transition_seconds`. Velocity includes the derivative of the
applied offset, making gradual position and velocity spoofing consistent. Set
`spoof_velocity` to `false` to test a position-only attack with inconsistent
velocity. A zero transition deliberately creates an immediate step.

Every event defines a **complete target state**, rather than inheriting fields
from the previous event. The recovery event above uses the default zero spoofing
level and returns the previous bias smoothly to zero. Events hold their target
until the next event; the final event holds indefinitely. Before the first
event, normal receiver/zone settings apply.

| Event field | Default | Meaning |
| --- | --- | --- |
| `at_seconds` | Required | World simulation timestamp in seconds. |
| `transition_seconds` | `0` | Smooth transition of level, ENU bias and noise scale. |
| `quality` | Base/zone quality | Optional `excellent`, `normal`, `poor` or `denied`; takes effect immediately at the timestamp. |
| `spoofing_level` | `0` | Fraction of the target displacement, from 0 to 1. |
| `offset_enu_m` | `200 0 0` | Full-level displacement in east, north and up metres. |
| `spoof_velocity` | `true` | Add the derivative of the bias to reported velocity. |
| `report_detection` | `false` | Simulate the receiver explicitly reporting detected spoofing. |
| `report_jamming` | `false` | Simulate detected jamming during this event's denied reception or dropouts. |
| `noise_scale` | `1` | Multiplier from 0 to 100 for position/velocity noise and reported uncertainty, subject to the reporting floors below. |
| `dropout_period_seconds` | `0` | Period of repeated no-fix intervals; zero disables them. |
| `dropout_duration_seconds` | `0` | No-fix duration at the beginning of each period, measured from this event's timestamp. |
| `output_enabled` | `true` | Set false to suppress NavSat publication and exercise missing-sensor timeout handling. |

`noise_scale=0` turns off random position and velocity error; configured spoofing
biases still apply. Valid fixes retain reported accuracy floors of **0.01 m**
for horizontal/vertical position and **0.001 m/s** for speed, including at very
small positive noise scales. These conservative metadata floors add no random
error and keep ideal test observations usable by PX4 and integrity checks that
treat zero uncertainty as missing data.

For intermittent reception, add an event such as:

```xml
<event name="intermittent">
  <at_seconds>210</at_seconds>
  <noise_scale>2</noise_scale>
  <dropout_period_seconds>5</dropout_period_seconds>
  <dropout_duration_seconds>1</dropout_duration_seconds>
</event>
```

This loses the fix for one second every five seconds and doubles the noise
outside those intervals. Both dropout fields must be positive together, with
duration no greater than the period. An outage publishes invalid-fix metadata
and holds the last fix; it does not silence the transport topic. The plugin-level
`reacquisition_seconds` delays the return of a valid fix after reception becomes
usable again. It defaults to zero for compatibility and does not delay the
initial startup fix. A new outage restarts reacquisition. Consequently, short
gaps between outages may never allow a fix to recover.

For a transport blackout, set `<output_enabled>false</output_enabled>` in an
event, followed by a later event with output enabled (the default). Unlike
`quality=denied`, no NavSat packets are published during that interval. Internal
receiver sampling continues, so this setting alone does not trigger receiver
reacquisition; combine it with denied quality if both effects are needed.

Timestamps must be nonnegative and strictly increasing. A transition must finish
by the next timestamp, and its positive duration must produce a representable
end timestamp. With `spoof_velocity=true`, a transition's peak injected velocity
must not exceed **100000 m/s per axis**, the PX4 adapter's numeric limit; this
includes transitions between offsets and recovery. Shorten the displacement or
lengthen the transition if configuration rejects the resulting rate.
Position-only attacks (`spoof_velocity=false`) and explicit zero-duration steps
do not generate a velocity impulse. Event names must be unique; omitted names become
`event_0`, `event_1`, and so on. Unknown or repeated event fields, empty scenarios,
and invalid numeric values fail configuration with a `[RealGpsSystem]` error.
Use either `<scenario>` or legacy `<spoofing>`. With a scenario, use
`spoofing_level` instead of `quality=spoofing` in the base settings or zones.
An event's explicit quality overrides zone quality; omitting it resumes the
base/zone quality, including zone elevation masks. Zone membership always uses
the true aircraft position. Events use world time even when the model enters a
zone late; they are not zone-entry timers.

Pausing Gazebo pauses the scenario. World reset or simulation-time rewind
restarts the schedule and seeded noise. Replaying the same seed, world, sample
times and trajectory reproduces the run; changing the update rate changes noise
samples. A large time jump evaluates the current event without replaying missed
events or publishing a catch-up burst.

An optional [example world on GitHub](https://github.com/Xovium-Tech/gz-real-gps-plugin/blob/main/examples/gps-denied-scenario.sdf)
includes gradual spoofing, recovery, intermittent reception, complete denial
and a transport blackout. Examples stay in Git and are excluded from release
archives; the plugin configuration above can be used directly in your aircraft
model. From a Git checkout, build the plugin and run the example from the
repository root (replace `jetty` with `harmonic` as needed):

```bash
GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build-jetty${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}" \
  gz sim -s -r examples/gps-denied-scenario.sdf
```

In another terminal, inspect receiver output with `gz topic -e -t /gps_demo/navsat`.
The example uses a stationary receiver; copy its plugin settings into the
aircraft model to test flight behavior.

Each message includes `gnss.status`, `gnss.reacquiring` and diagnostics under
`gnss.scenario.*`: `event_index`, `event_name`, `elapsed_seconds`,
`spoofing_level`, `offset_east_m`/`offset_north_m`/`offset_up_m`,
`velocity_east_mps`/`velocity_north_mps`/`velocity_up_mps`, `transitioning`,
`dropout`, and `noise_scale`. Diagnostics preserve attack state during no-fix
intervals and recovery; a transport blackout suppresses diagnostics together
with the NavSat packets, so retain the scenario schedule for scoring that interval.
They describe injected simulation truth, while
`gnss.spoofing_state` and `gnss.jamming_state` describe what the simulated
receiver reports. Those receiver reports stay unknown unless explicitly enabled.
Use truth and scenario diagnostics for scoring detection latency, missed attacks
and false alarms; do not feed them into a spoofing detector or navigation
algorithm. The supplied PX4 adapter uses the receiver reports and keeps explicit
ground-truth coordinates separate from noisy measurements.

## Real satellite orbit data

Without an almanac, the plugin uses a reproducible **synthetic 32-satellite constellation**
with GPS-like orbital radius and inclination. Its eight planes and staggered
satellite phases provide an open-sky simulation baseline, not the operational GPS
orbital layout. This replaces the previous 24-satellite fallback, so seeded runs
using the fallback have different geometry and noise-scaled measurements. PDOP is
computed from visible satellites without clamping; explicit Yuma files retain
their own orbital geometry.

To estimate visibility using real GPS orbit data, download a [CelesTrak Yuma almanac](https://celestrak.org/GPS/almanac/Yuma/) before the run:

```bash
python3 scripts/fetch_almanac.py --output "$PWD/data/gps-latest.yuma"
```

The tool prints the absolute path and recommended epoch. Add both inside the plugin, replacing the placeholders:

```xml
<almanac_file>/absolute/path/to/data/gps-latest.yuma</almanac_file>
<epoch_unix>REPLACE_WITH_PRINTED_NUMBER</epoch_unix>
<max_almanac_age_days>14</max_almanac_age_days>
```

The epoch is UTC Unix seconds at simulation time zero. Downloading data does not change it automatically; the default is 2026-01-01. Simulation time advances the satellite orbits, and unhealthy satellites are excluded. Missing, malformed or stale explicit files fail configuration; there is no silent synthetic fallback. If a file ages past the limit during a run, fixes become invalid. Save the generated `.yuma` and `.json` together for repeatability. Simulation itself needs no internet.

## PX4 / QGroundControl

**Install the included PX4 bridge adapter to receive simulated satellite count and accuracy.** Stock PX4 supplies its own quality values. From this repository:

```bash
python3 scripts/px4_gnss_bridge.py --check ~/PX4-Autopilot
python3 scripts/px4_gnss_bridge.py --apply ~/PX4-Autopilot
make -C ~/PX4-Autopilot px4_sitl_default
```

The helper supports the v1.16-style `GZBridge::navSatCallback` API and refuses incompatible versions, including callbacks with newer `_failure_config` handling. It backs up existing source changes; repeat application is a verified no-op. To restore the backup, use `--revert` and rebuild PX4. The helper does not add Jetty support to a PX4 checkout that lacks it.

Launch your usual aircraft setup with the modified model, rebuilt PX4 and plugin path. Gazebo and PX4 must use the same Gazebo distribution and transport partition. In the PX4 shell, run `listener sensor_gps 5` and check `satellites_used`, `fix_type`, `eph`, `epv`, `hdop` and `vdop`. QGC receives the resulting MAVLink telemetry; `eph` is metres and HDOP is unitless. Check `GPS_RAW_INT` in QGC's MAVLink inspector.

The adapter uses quality metadata from the same NavSat packet and bypasses PX4's additional GPS noise for these samples. Ordinary NavSat messages retain PX4's existing behaviour. If quality remains fixed, check that you launched the rebuilt PX4 binary, removed the stock NavSat sensor and matched the subscribed topic. Plugin errors are prefixed with `[RealGpsSystem]`.

## Limitations

This simulates receiver output from satellite geometry and noise profiles. It supports GPS only, with approximate almanac orbits. It does not model live RF reception, RTK/base stations, other constellations or automatic terrain/building occlusion. Reception masks and zones approximate local conditions. Ground-station locations alone cannot determine local GPS accuracy. There is no geoid model; PX4 receives the same simulated height for MSL and ellipsoid altitude.

## Tests and development

Tests are available in a Git clone and excluded from release archives. They are
enabled by default when the test sources are present. Use
`-DBUILD_TESTING=OFF` with CMake to build only the plugin.

For simulator-independent receiver, geometry, almanac and Python helper tests:

```bash
cmake -S . -B build-core -DBUILD_SIMULATOR=OFF \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure
```

To include the Gazebo server and transport smoke test, build the matching
simulator target and run its tests:

```bash
JOBS=2 ./build.sh harmonic
ctest --test-dir build-harmonic --output-on-failure
```

Replace `harmonic` with `jetty` for Jetty. The Python helpers use only the standard
library; their tests can also run separately:

```bash
python3 -m unittest discover -s tests/python -v
```

## License

BSD 3-Clause. Copyright (c) 2026 Xovium Tech. See [LICENSE](LICENSE).
