/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "mcp9600.h"

#include <cstring>

MCP9600::MCP9600(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors")),
	_collection_errors(perf_alloc(PC_COUNT, MODULE_NAME": collection errors"))
{
	_sensor_temp.device_id = get_device_id();

	// Advertise immediately so the uORB instance is reserved and the ROS2 topic
	// is always visible, even before the sensor is physically connected.
	_sensor_temp_pub.advertise();
}

MCP9600::~MCP9600()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
}

int MCP9600::read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	const int ret = transfer(&reg, 1, buf, len);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return ret;
}

int MCP9600::write_reg(uint8_t reg, uint8_t value)
{
	uint8_t buf[2] = {reg, value};
	return transfer(buf, sizeof(buf), nullptr, 0);
}

int MCP9600::probe()
{
	uint8_t id[2] = {};

	if (read_reg(MCP9600_REG_DEVICE_ID, id, sizeof(id)) != PX4_OK) {
		PX4_ERR("probe: failed to read device ID (bus=%u addr=0x%02x)", get_device_bus(), get_device_address());
		return PX4_ERROR;
	}

	if (id[0] != MCP9600_DEVICE_ID_MSB && id[0] != MCP9601_DEVICE_ID_MSB) {
		PX4_ERR("probe: unexpected device ID 0x%02x (expected 0x40 or 0x41)", (unsigned)id[0]);
		return PX4_ERROR;
	}

	PX4_DEBUG("MCP9600 found: device_id=0x%02x rev=0x%02x", (unsigned)id[0], (unsigned)id[1]);
	return PX4_OK;
}

int MCP9600::init()
{
	if (I2C::init() != PX4_OK) {
		PX4_ERR("I2C init/probe failed (bus=%u addr=0x%02x)", get_device_bus(), get_device_address());
		return PX4_ERROR;
	}

	/* Place device in normal (continuous) conversion mode */
	if (write_reg(MCP9600_REG_DEVICE_CONFIG, MCP9600_DEVICE_CONFIG_NORMAL) != PX4_OK) {
		PX4_ERR("failed to write device config (bus=%u addr=0x%02x)", get_device_bus(), get_device_address());
		return PX4_ERROR;
	}

	_initialized = true;
	ScheduleOnInterval(MCP9600_SAMPLE_INTERVAL_MS);
	return PX4_OK;
}

int MCP9600::force_init()
{
	int ret = init();

	if (!_initialized) {
		ScheduleDelayed(MCP9600_INIT_RETRY_US);
	}

	return ret;
}

bool MCP9600::read_temperature(float &temperature_c)
{
	uint8_t raw[2] = {};

	if (read_reg(MCP9600_REG_HOT_JUNCTION, raw, sizeof(raw)) != PX4_OK) {
		return false;
	}

	/* 16-bit signed value: 12-bit integer + 4-bit fraction, 0.0625 °C/LSB */
	const int16_t raw16 = static_cast<int16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
	temperature_c = static_cast<float>(raw16) * MCP9600_TEMP_LSB;
	return true;
}

void MCP9600::RunImpl()
{
	if (!_initialized) {
		// Publish a sentinel so the uORB topic (and the ROS2 topic) stays visible
		// even while the sensor is not yet connected.
		_sensor_temp.timestamp   = hrt_absolute_time();
		_sensor_temp.temperature = -273.15f;
		_sensor_temp_pub.publish(_sensor_temp);

		if (init() != PX4_OK) {
			ScheduleDelayed(MCP9600_INIT_RETRY_US);
		}

		return;
	}

	perf_begin(_cycle_perf);

	float temperature_c = 0.f;

	if (read_temperature(temperature_c)) {
		_sensor_temp.timestamp   = hrt_absolute_time();
		_sensor_temp.temperature = temperature_c;
		_sensor_temp_pub.publish(_sensor_temp);

	} else {
		perf_count(_collection_errors);
	}

	perf_end(_cycle_perf);
}

void MCP9600::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_collection_errors);

	if (_initialized) {
		float temperature_c = 0.f;

		if (read_temperature(temperature_c)) {
			PX4_INFO("temperature: %.4f °C", (double)temperature_c);

		} else {
			PX4_WARN("failed to read temperature");
		}

	} else {
		PX4_INFO("Not initialized. Retrying every %d ms.", MCP9600_INIT_RETRY_US / 1000);
	}
}
