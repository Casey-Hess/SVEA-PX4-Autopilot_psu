/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include "svea_ina238.h"

#include <cstring>

SVEA_INA238::SVEA_INA238(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, "svea_ina238_read")),
	_comms_errors(perf_alloc(PC_COUNT, "svea_ina238_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "svea_ina238_collection_err"))
{
	if (config.custom2 > 0) {
		_rshunt = static_cast<float>(config.custom2) * 1e-9f;
	}

	_current_lsb = _max_current / INA238_DN_MAX;
	_shunt_calibration = static_cast<uint16_t>(INA238_CONST * _current_lsb * _rshunt);

	I2C::_retries = 5;
}

SVEA_INA238::~SVEA_INA238()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
}

int SVEA_INA238::read_reg(uint8_t reg, uint16_t &data)
{
	uint16_t rx = 0;
	const int ret = transfer(&reg, 1, reinterpret_cast<uint8_t *>(&rx), sizeof(rx));

	if (ret == PX4_OK) {
		data = swap16(rx);

	} else {
		perf_count(_comms_errors);
	}

	return ret;
}

int SVEA_INA238::write_reg(uint8_t reg, uint16_t value)
{
	uint8_t data[3] = {reg, static_cast<uint8_t>((value & 0xff00) >> 8), static_cast<uint8_t>(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int SVEA_INA238::probe()
{
	uint16_t mfg_id = 0;
	uint16_t dev_id = 0;

	if (read_reg(INA238_REG_MANUFACTURER_ID, mfg_id) != PX4_OK || mfg_id != INA238_MFG_ID_TI) {
		PX4_ERR("probe failed: mfg_id=0x%04x (expected=0x%04x)", (unsigned)mfg_id, (unsigned)INA238_MFG_ID_TI);
		return PX4_ERROR;
	}

	if (read_reg(INA238_REG_DEVICE_ID, dev_id) != PX4_OK) {
		PX4_ERR("probe failed: could not read device ID");
		return PX4_ERROR;
	}

	const uint16_t die_id = (dev_id >> 4) & 0xFFF;

	if (die_id != INA238_DIE_ID) {
		PX4_ERR("probe failed: die_id=0x%04x (expected=0x%04x)", (unsigned)die_id, (unsigned)INA238_DIE_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int SVEA_INA238::init()
{
	if (I2C::init() != PX4_OK) {
		PX4_ERR("I2C init/probe failed (bus=%u addr=0x%02x)", get_device_bus(), get_device_address());
		return PX4_ERROR;
	}

	if (write_reg(INA238_REG_CONFIG, INA238_RST_BIT) != PX4_OK) {
		PX4_ERR("failed to reset (bus=%u addr=0x%02x)", get_device_bus(), get_device_address());
		return PX4_ERROR;
	}

	if (write_reg(INA238_REG_SHUNT_CAL, _shunt_calibration) != PX4_OK) {
		PX4_ERR("failed to write shunt cal (bus=%u addr=0x%02x cal=%u)", get_device_bus(), get_device_address(),
			(unsigned)_shunt_calibration);
		return PX4_ERROR;
	}

	if (write_reg(INA238_REG_CONFIG, INA238_ADCRANGE_HIGH) != PX4_OK) {
		PX4_ERR("failed to write config (bus=%u addr=0x%02x)", get_device_bus(), get_device_address());
		return PX4_ERROR;
	}

	if (write_reg(INA238_REG_ADCCONFIG, INA238_ADCCONFIG_DEFAULT) != PX4_OK) {
		PX4_ERR("failed to write adcconfig (bus=%u addr=0x%02x)", get_device_bus(), get_device_address());
		return PX4_ERROR;
	}

	_initialized = true;
	start();
	return PX4_OK;
}

int SVEA_INA238::force_init()
{
	int ret = init();
	start();
	return ret;
}

void SVEA_INA238::start()
{
	ScheduleClear();
	// When the sensor is ready, delay one full conversion cycle before the first
	// collect so the ADC result is valid (64 averages × 540 µs ≈ 35 ms).
	// When not initialized, fire RunImpl immediately so the sentinel publish
	// happens quickly and the ROS2 topic becomes visible without delay.
	ScheduleDelayed(_initialized ? INA238_CONVERSION_INTERVAL : 5);
}

void SVEA_INA238::publish_status(bool valid, int16_t bus_raw, int16_t current_raw)
{
	const float voltage_v = valid ? (static_cast<float>(bus_raw) * INA238_VSCALE) : 0.f;
	const float current_a = valid ? (static_cast<float>(current_raw) * _current_lsb) : -1.f;
	const float power_w   = valid ? (voltage_v * current_a) : -1.f;

	memset(&_pm_status, 0, sizeof(_pm_status));
	_pm_status.timestamp  = hrt_absolute_time();
	_pm_status.voltage_v  = voltage_v;
	_pm_status.current_a  = current_a;
	_pm_status.power_w    = power_w;
	_pm_status.rbv        = bus_raw;
	_pm_status.rc         = current_raw;
	_pm_status.rcal       = static_cast<int16_t>(_shunt_calibration);

	_pm_pub_topic.publish(_pm_status);
}

int SVEA_INA238::collect()
{
	perf_begin(_sample_perf);

	uint16_t bus_raw_u16   = 0;
	uint16_t current_raw_u16 = 0;

	const bool bus_ok     = (read_reg(INA238_REG_VBUS, bus_raw_u16) == PX4_OK);
	const bool current_ok = (read_reg(INA238_REG_CURRENT, current_raw_u16) == PX4_OK);
	const bool valid      = bus_ok && current_ok;

	if (!valid) {
		perf_count(_collection_errors);
	}

	publish_status(valid,
		       static_cast<int16_t>(bus_raw_u16),
		       static_cast<int16_t>(current_raw_u16));

	perf_end(_sample_perf);
	return valid ? PX4_OK : PX4_ERROR;
}

void SVEA_INA238::RunImpl()
{
	if (_initialized) {
		if (collect() != PX4_OK) {
			perf_count(_collection_errors);
		}

		ScheduleDelayed(INA238_CONVERSION_INTERVAL);

	} else {
		// Publish invalid reading so the uORB topic and its ROS2 bridge entry
		// remain visible even while the sensor is not yet connected.
		publish_status(false, 0, 0);

		if (init() != PX4_OK) {
			ScheduleDelayed(INA238_SVEA_INIT_RETRY_US);
		}
	}
}

void SVEA_INA238::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_collection_errors);

	PX4_INFO("shunt: %.6f Ohm  max_current: %.3f A  current_lsb: %.6f A  shunt_cal: %u",
		 (double)_rshunt, (double)_max_current, (double)_current_lsb, (unsigned)_shunt_calibration);

	if (_initialized) {
		uint16_t vbus = 0;
		uint16_t current = 0;
		const bool ok = (read_reg(INA238_REG_VBUS, vbus) == PX4_OK) &&
				(read_reg(INA238_REG_CURRENT, current) == PX4_OK);

		if (ok) {
			PX4_INFO("Vbus=%.4f V  I=%.4f A  P=%.4f W",
				 (double)(static_cast<int16_t>(vbus) * INA238_VSCALE),
				 (double)(static_cast<int16_t>(current) * _current_lsb),
				 (double)(static_cast<int16_t>(vbus) * INA238_VSCALE * static_cast<int16_t>(current) * _current_lsb));

		} else {
			PX4_WARN("failed to read INA238 status registers");
		}

	} else {
		PX4_INFO("Not initialized. Retrying every %d ms.", INA238_SVEA_INIT_RETRY_US / 1000);
	}
}
