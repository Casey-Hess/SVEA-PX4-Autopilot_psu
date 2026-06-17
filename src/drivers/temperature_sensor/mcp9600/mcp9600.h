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

#pragma once

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/sensor_temp.h>
#include <uORB/PublicationMulti.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

/* MCP9600 register addresses */
#define MCP9600_REG_HOT_JUNCTION    0x00   /* Hot junction (thermocouple) temperature */
#define MCP9600_REG_COLD_JUNCTION   0x02   /* Cold junction (ambient) temperature */
#define MCP9600_REG_STATUS          0x04   /* Status */
#define MCP9600_REG_TC_CONFIG       0x05   /* Thermocouple sensor configuration */
#define MCP9600_REG_DEVICE_CONFIG   0x06   /* Device configuration */
#define MCP9600_REG_DEVICE_ID       0x20   /* Device ID / revision */

/* Expected device ID MSB: 0x40 = MCP9600, 0x41 = MCP9601 */
#define MCP9600_DEVICE_ID_MSB       0x40
#define MCP9601_DEVICE_ID_MSB       0x41

/* Device config: normal mode (bits 1:0 = 00), 18-bit ADC (bits 4:3 = 00) */
#define MCP9600_DEVICE_CONFIG_NORMAL 0x00

/* Temperature LSB: 0.0625 °C per count (16-bit signed, 12-bit integer + 4-bit fraction) */
#define MCP9600_TEMP_LSB            0.0625f

#define MCP9600_SAMPLE_INTERVAL_MS  200_ms   /* 5 Hz */
#define MCP9600_INIT_RETRY_US       500000

class MCP9600 : public device::I2C, public I2CSPIDriver<MCP9600>
{
public:
	MCP9600(const I2CSPIDriverConfig &config);
	~MCP9600() override;

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();


	int init() override;
	int force_init();
	void RunImpl();
	void print_status() override;

protected:
	int probe() override;

private:
	int read_reg(uint8_t reg, uint8_t *buf, uint8_t len);
	int write_reg(uint8_t reg, uint8_t value);
	bool read_temperature(float &temperature_c);

	bool _initialized{false};
        uORB::Publication<sensor_temp_s> _sensor_temp_pub{ORB_ID(sensor_temp)};
	sensor_temp_s _sensor_temp{};

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _collection_errors;
};
