/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#ifndef EMSAD_SENSOR_DATA_HPP
#define EMSAD_SENSOR_DATA_HPP

#include <uORB/topics/emsad.h>

class MavlinkStreamEmsadSensorData : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamEmsadSensorData(mavlink); }

	static constexpr const char *get_name_static() { return "EMSAD_SENSOR_DATA"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_EMSAD_SENSOR_DATA; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _emsad_sub.advertised() ? MAVLINK_MSG_ID_EMSAD_SENSOR_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamEmsadSensorData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _emsad_sub{ORB_ID(emsad)};

	bool send() override
	{
		emsad_s emsad{};

		if (_emsad_sub.update(&emsad)) {
			mavlink_emsad_sensor_data_t msg{};
			msg.timestamp = emsad.timestamp;
			msg.acc = emsad.acc;
			msg.pressure_diff = emsad.pressure_diff;
			msg.capacitor_volt = emsad.capacitor_volt;
			msg.pressure = emsad.pressure;
			msg.status_flags = emsad.status_flags;
			msg.fsm_state = emsad.fsm_state;
			msg.mcu_timestamp = emsad.mcu_timestamp;

			mavlink_msg_emsad_sensor_data_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // EMSAD_SENSOR_DATA_HPP
