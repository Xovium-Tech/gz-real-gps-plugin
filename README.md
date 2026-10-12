# Real GPS Plugin

GPS receiver simulation for **Gazebo Harmonic and Jetty**, including PX4 fixed-wing aircraft. Satellite count, accuracy and fix availability change with aircraft location, simulation time, satellite geometry and reception quality. Modes include poor reception, loss of GPS and spoofing.

Build from source on the computer running Gazebo. Harmonic uses **gz-sim8**; Jetty uses **gz-sim10**. Each build produces one plugin library. Python helpers download satellite orbit data and connect the quality output to PX4/QGroundControl.

## 1. Build

Install Gazebo using the official [Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) or [Jetty](https://gazebosim.org/docs/jetty/install_ubuntu/) instructions. On Ubuntu 24.04, with the Gazebo package repository configured:

```bash
sudo apt update
sudo apt install build-essential cmake git python3

# Install the development packages for your Gazebo version:
sudo apt install libgz-sim8-dev libgz-plugin2-dev libgz-transport13-dev  # Harmonic
sudo apt install libgz-sim10-dev libgz-plugin4-dev libgz-transport15-dev  # Jetty
```

```bash
git clone https://github.com/Xovium-Tech/gz-real-gps-plugin.git
cd gz-real-gps-plugin

# Choose one:
JOBS=2 ./build.sh harmonic
JOBS=2 ./build.sh jetty
```

The helper builds in Release mode; its default target is `harmonic`. The output is `build-harmonic/libRealGpsSystem.so` or `build-jetty/libRealGpsSystem.so`. Use the version matching both Gazebo and PX4.

From the plugin directory, export the library path before launching your simulation:

```bash
# Change build-harmonic to build-jetty for Jetty.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build-harmonic${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

For a launch script, use the absolute plugin build directory instead of `$PWD`.

## 2. Add to the aircraft

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

## 3. Reception quality

| `<quality>` | Behaviour |
| --- | --- |
| `excellent` | Low noise: 0.45 m range-error sigma before satellite geometry. |
| `normal` | 1.5 m range-error sigma, correlated position drift and velocity noise. |
| `poor` | 6 m range-error sigma and a further 15° elevation mask; fewer satellites and possible fix loss. |
| `spoofing` | Gradually displaced position and matching velocity error, with normal reported uncertainty. |
| `denied` | Zero satellites, no fix and invalid velocity. |

Accuracy depends on satellite geometry; these values are not fixed position-accuracy outputs. Fewer than four usable satellites produces no fix. Aliases `gps_spoofing`, `gps-spoofing` and `jamming` are also accepted.

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

## 4. Real satellite orbit data

Without an almanac, the plugin uses a reproducible **synthetic 24-satellite GPS constellation**. To estimate visibility using real GPS orbit data, download a [CelesTrak Yuma almanac](https://celestrak.org/GPS/almanac/Yuma/) before the run:

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

## 5. PX4 / QGroundControl

**Install the included PX4 bridge adapter to receive simulated satellite count and accuracy.** Stock PX4 supplies its own quality values. From this repository:

```bash
python3 scripts/px4_gnss_bridge.py --check ~/PX4-Autopilot
python3 scripts/px4_gnss_bridge.py --apply ~/PX4-Autopilot
make -C ~/PX4-Autopilot px4_sitl_default
```

The helper supports the v1.16-style `GZBridge::navSatCallback` API and refuses incompatible versions, including callbacks with newer `_failure_config` handling. It backs up existing source changes; repeat application is a verified no-op. To restore the backup, use `--revert` and rebuild PX4. The helper does not add Jetty support to a PX4 checkout that lacks it.

Launch your usual aircraft setup with the modified model, rebuilt PX4 and plugin path. Gazebo and PX4 must use the same Gazebo distribution and transport partition. In the PX4 shell, run `listener sensor_gps 5` and check `satellites_used`, `fix_type`, `eph`, `epv`, `hdop` and `vdop`. QGC receives the resulting MAVLink telemetry; `eph` is metres and HDOP is unitless. Check `GPS_RAW_INT` in QGC's MAVLink inspector.

The adapter uses quality metadata from the same NavSat packet and bypasses PX4's additional GPS noise for these samples. Ordinary NavSat messages retain PX4's existing behaviour. If quality remains fixed, check that you launched the rebuilt PX4 binary, removed the stock NavSat sensor and matched the subscribed topic. Plugin errors are prefixed with `[RealGpsSystem]`.

## Limits and license

This simulates receiver output from satellite geometry and noise profiles. It supports GPS only, with approximate almanac orbits. It does not model live RF reception, RTK/base stations, other constellations or automatic terrain/building occlusion. Reception masks and zones approximate local conditions. Ground-station locations alone cannot determine local GPS accuracy. There is no geoid model; PX4 receives the same simulated height for MSL and ellipsoid altitude.

BSD 3-Clause. See [LICENSE](LICENSE).
