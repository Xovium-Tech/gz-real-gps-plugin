	// GZ_REAL_GPS_ADAPTER_V1_BEGIN
	// The plugin has already applied receiver errors and supplied its quality.
	// Keep the ordinary Gazebo sensor path below for messages without metadata.
	sensor_gps_s real_gps{};
	gz_real_gps::GroundTruth real_gps_truth;
	const auto real_gps_status = gz_real_gps::Decode(msg, real_gps, real_gps_truth);

	if (real_gps_status == gz_real_gps::DecodeResult::Invalid) {
		PX4_WARN("invalid gz-real-gps metadata; dropping GNSS sample");
		return;
	}

	if (real_gps_status == gz_real_gps::DecodeResult::Valid) {
		const uint64_t real_gps_now = hrt_absolute_time();
		device::Device::DeviceId real_gps_id{};
		real_gps_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
		real_gps_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
		real_gps_id.devid_s.bus = 1;
		real_gps_id.devid_s.address = 1;
		real_gps.timestamp = real_gps_now;
		real_gps.timestamp_sample = real_gps_now;
		real_gps.device_id = real_gps_id.devid;

		if (real_gps_truth.valid) {
			vehicle_global_position_s real_groundtruth{};
			real_groundtruth.timestamp = real_gps_now;
			real_groundtruth.timestamp_sample = real_gps_now;
			real_groundtruth.lat = real_gps_truth.latitude_deg;
			real_groundtruth.lon = real_gps_truth.longitude_deg;
			real_groundtruth.alt = static_cast<float>(real_gps_truth.altitude_m);
			_gpos_ground_truth_pub.publish(real_groundtruth);
		}

		_sensor_gps_pub.publish(real_gps);
		return;
	}
	// GZ_REAL_GPS_ADAPTER_V1_END
