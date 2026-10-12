// SPDX-License-Identifier: BSD-3-Clause
// GNSS metadata decoder shared by the PX4 adapter and its standalone tests.
#pragma once

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>

namespace gz_real_gps
{

enum class DecodeResult { Absent, Valid, Invalid };

struct GroundTruth {
	bool valid{false};
	double latitude_deg{0.0};
	double longitude_deg{0.0};
	double altitude_m{0.0};
};

// Header values are strings, not protobuf numeric fields. Reject duplicates,
// truncated numbers, non-finite values and invalid ranges before publication.
template<typename Message>
class Metadata
{
public:
	explicit Metadata(const Message &message) : _message(message) {}

	bool present() const
	{
		for (const auto &entry : _message.header().data()) {
			if (entry.key().compare(0, 5, "gnss.") == 0) { return true; }
		}

		return false;
	}

	bool number(const char *key, double &value, double minimum, double maximum, bool required = true) const
	{
		const std::string *raw = nullptr;

		if (!lookup(key, raw) || (!raw && required)) { return false; }
		if (!raw) { return true; }
		if (raw->empty() || raw->find_first_of(" \t\r\n") != std::string::npos) { return false; }

		char *end = nullptr;
		errno = 0;
		const double parsed = std::strtod(raw->c_str(), &end);

		if (end != raw->c_str() + raw->size() || errno == ERANGE || !std::isfinite(parsed)
		    || parsed < minimum || parsed > maximum) { return false; }

		value = parsed;
		return true;
	}

	bool integer(const char *key, uint64_t &value, uint64_t maximum, bool required = true) const
	{
		const std::string *raw = nullptr;

		if (!lookup(key, raw) || (!raw && required)) { return false; }
		if (!raw) { return true; }
		if (raw->empty() || raw->find_first_not_of("0123456789") != std::string::npos) { return false; }

		char *end = nullptr;
		errno = 0;
		const auto parsed = std::strtoull(raw->c_str(), &end, 10);

		if (end != raw->c_str() + raw->size() || errno == ERANGE || parsed > maximum) { return false; }

		value = static_cast<uint64_t>(parsed);
		return true;
	}

	bool lookup(const char *key, const std::string *&value) const
	{
		value = nullptr;

		for (const auto &entry : _message.header().data()) {
			if (entry.key() == key) {
				if (value || entry.value_size() != 1) { return false; }
				value = &entry.value(0);
			}
		}

		return true;
	}

private:
	const Message &_message;
};

inline bool finiteInRange(double value, double minimum, double maximum)
{
	return std::isfinite(value) && value >= minimum && value <= maximum;
}

// Report is sensor_gps_s in PX4. Templating keeps this decoder independently
// testable against the real gz-msgs protobuf without PX4 build dependencies.
template<typename Message, typename Report>
DecodeResult Decode(const Message &message, Report &report, GroundTruth &truth)
{
	const Metadata<Message> metadata(message);

	if (!metadata.present()) { return DecodeResult::Absent; }

	uint64_t version = 0, satellites = 0, fix = 0, jamming = 0, spoofing = 0, utc = 0;
	double eph = 0.0, epv = 0.0, hdop = 0.0, vdop = 0.0, speed = 0.0;
	double course = 0.1;

	if (!metadata.integer("gnss.schema_version", version, 1) || version != 1
	    || !metadata.integer("gnss.satellites_used", satellites, 255)
	    || !metadata.integer("gnss.fix_type", fix, 6)
	    || !metadata.number("gnss.eph", eph, 0.0, 1.0e7)
	    || !metadata.number("gnss.epv", epv, 0.0, 1.0e7)
	    || !metadata.number("gnss.hdop", hdop, 0.0, 1.0e6)
	    || !metadata.number("gnss.vdop", vdop, 0.0, 1.0e6)
	    || !metadata.number("gnss.speed_accuracy", speed, 0.0, 1.0e5)
	    || !metadata.number("gnss.course_accuracy", course, 0.0, 1.0e5, false)
	    || !metadata.integer("gnss.jamming_state", jamming, 3, false)
	    || !metadata.integer("gnss.spoofing_state", spoofing, 3, false)
	    || !metadata.integer("gnss.time_utc_usec", utc, std::numeric_limits<uint64_t>::max(), false)
	    || !finiteInRange(message.latitude_deg(), -90.0, 90.0)
	    || !finiteInRange(message.longitude_deg(), -180.0, 180.0)
	    || !finiteInRange(message.altitude(), -1.0e7, 1.0e7)
	    || !finiteInRange(message.velocity_north(), -1.0e5, 1.0e5)
	    || !finiteInRange(message.velocity_east(), -1.0e5, 1.0e5)
	    || !finiteInRange(message.velocity_up(), -1.0e5, 1.0e5)) { return DecodeResult::Invalid; }

	// Only an explicit truth triplet can feed PX4 groundtruth. Never use the
	// measured coordinates: they already contain noise or spoofing offsets.
	GroundTruth decoded_truth;
	const std::string *truth_lat = nullptr, *truth_lon = nullptr, *truth_alt = nullptr;

	if (!metadata.lookup("gnss.truth_latitude_deg", truth_lat)
	    || !metadata.lookup("gnss.truth_longitude_deg", truth_lon)
	    || !metadata.lookup("gnss.truth_altitude_m", truth_alt)) { return DecodeResult::Invalid; }

	if (truth_lat || truth_lon || truth_alt) {
		if (!metadata.number("gnss.truth_latitude_deg", decoded_truth.latitude_deg, -90.0, 90.0)
		    || !metadata.number("gnss.truth_longitude_deg", decoded_truth.longitude_deg, -180.0, 180.0)
		    || !metadata.number("gnss.truth_altitude_m", decoded_truth.altitude_m, -1.0e7, 1.0e7)) {
			return DecodeResult::Invalid;
		}

		decoded_truth.valid = true;
	}

	Report decoded{};
	decoded.latitude_deg = message.latitude_deg();
	decoded.longitude_deg = message.longitude_deg();
	decoded.altitude_msl_m = message.altitude();
	decoded.altitude_ellipsoid_m = message.altitude();
	decoded.eph = static_cast<float>(eph);
	decoded.epv = static_cast<float>(epv);
	decoded.hdop = static_cast<float>(hdop);
	decoded.vdop = static_cast<float>(vdop);
	decoded.s_variance_m_s = static_cast<float>(speed);
	decoded.c_variance_rad = static_cast<float>(course);
	decoded.satellites_used = static_cast<uint8_t>(satellites);
	decoded.fix_type = static_cast<uint8_t>(fix);
	decoded.vel_n_m_s = static_cast<float>(message.velocity_north());
	decoded.vel_e_m_s = static_cast<float>(message.velocity_east());
	decoded.vel_d_m_s = static_cast<float>(-message.velocity_up());
	decoded.vel_m_s = static_cast<float>(std::hypot(message.velocity_north(), message.velocity_east()));
	decoded.cog_rad = static_cast<float>(std::atan2(message.velocity_east(), message.velocity_north()));
	decoded.vel_ned_valid = fix >= 3;
	decoded.time_utc_usec = utc;
	decoded.jamming_state = static_cast<uint8_t>(jamming);
	decoded.spoofing_state = static_cast<uint8_t>(spoofing);
	decoded.heading = std::numeric_limits<float>::quiet_NaN();
	decoded.heading_offset = std::numeric_limits<float>::quiet_NaN();
	report = decoded;
	truth = decoded_truth;
	return DecodeResult::Valid;
}

} // namespace gz_real_gps
