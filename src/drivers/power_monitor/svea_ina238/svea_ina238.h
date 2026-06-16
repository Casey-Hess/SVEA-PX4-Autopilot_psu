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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/power_monitor.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

#define INA238_SVEA_BASEADDR           0x44
#define INA238_SVEA_INIT_RETRY_US      500000

/* INA238 register addresses */
#define INA238_REG_CONFIG              0x00
#define INA238_REG_ADCCONFIG           0x01
#define INA238_REG_SHUNT_CAL           0x02
#define INA238_REG_VBUS                0x05
#define INA238_REG_CURRENT             0x07
#define INA238_REG_MANUFACTURER_ID     0x3E
#define INA238_REG_DEVICE_ID           0x3F

#define INA238_MFG_ID_TI               0x5449
#define INA238_DIE_ID                  0x0238

/* CONFIG register bits */
#define INA238_RST_BIT                 (1 << 15)
#define INA238_ADCRANGE_HIGH           0x0000   /* ±163.84 mV shunt range */

/* ADCCONFIG: shunt+bus continuous, 540 us CT, 64 averages */
#define INA238_MODE_SHUNT_BUS_CONT     (0xF << 12)
#define INA238_VBUSCT_540US            (0x4 << 9)
#define INA238_VSHCT_540US             (0x4 << 6)
#define INA238_VTCT_540US              (0x4 << 3)
#define INA238_AVERAGES_64             (0x3 << 0)
#define INA238_ADCCONFIG_DEFAULT \
	(INA238_MODE_SHUNT_BUS_CONT | INA238_VBUSCT_540US | INA238_VSHCT_540US | INA238_VTCT_540US | INA238_AVERAGES_64)

#define INA238_SAMPLE_FREQUENCY_HZ     4
#define INA238_SAMPLE_INTERVAL_US      (1_s / INA238_SAMPLE_FREQUENCY_HZ)
#define INA238_CONVERSION_INTERVAL     (INA238_SAMPLE_INTERVAL_US - 7)

#define INA238_VSCALE                  3.125e-3f   /* 3.125 mV/LSB bus voltage */
#define INA238_DN_MAX                  32768.0f    /* 2^15 */
#define INA238_CONST                   819.2e6f    /* calibration constant */
#define INA238_DEFAULT_MAX_CURRENT     164.0f      /* Amps */
#define INA238_DEFAULT_SHUNT           0.001f      /* Ohm */

#define swap16(w) __builtin_bswap16((w))

class SVEA_INA238 : public device::I2C, public I2CSPIDriver<SVEA_INA238>
{
public:
	SVEA_INA238(const I2CSPIDriverConfig &config);
	virtual ~SVEA_INA238();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void RunImpl();
	int init() override;
	int force_init();
	void print_status() override;

protected:
	int probe() override;

private:
	bool _initialized{false};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _collection_errors;

	float _max_current{INA238_DEFAULT_MAX_CURRENT};
	float _rshunt{INA238_DEFAULT_SHUNT};
	float _current_lsb{INA238_DEFAULT_MAX_CURRENT / INA238_DN_MAX};
	uint16_t _shunt_calibration{0};

	uORB::PublicationMulti<power_monitor_s> _pm_pub_topic{ORB_ID(power_monitor)};
	power_monitor_s _pm_status{};

	int read_reg(uint8_t reg, uint16_t &data);
	int write_reg(uint8_t reg, uint16_t data);
	void start();
	int collect();
	void publish_status(bool valid, int16_t bus_raw, int16_t current_raw);
};
