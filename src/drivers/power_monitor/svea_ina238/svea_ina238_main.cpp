/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "svea_ina238.h"

I2CSPIDriverBase *SVEA_INA238::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	SVEA_INA238 *instance = new SVEA_INA238(config);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (config.keep_running) {
		if (instance->force_init() != PX4_OK) {
			PX4_INFO("Failed to init svea_ina238 on bus %d addr 0x%02x, retrying periodically.",
				 config.bus, config.i2c_address);
		}

	} else {
		const int ret = instance->init();

		if (ret != PX4_OK) {
			PX4_ERR("init failed (%d) on bus %d addr 0x%02x", ret, config.bus, config.i2c_address);
			delete instance;
			return nullptr;
		}
	}

	return instance;
}

void SVEA_INA238::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the SVEA-specific INA238 power monitor.

Multiple instances can run simultaneously on the same bus at different I2C addresses.

Use -f to enable keep-running mode: if initialization fails (sensor not yet powered),
the driver will keep retrying every 0.5 s and begin publishing once the sensor connects.
The topic is always registered at startup so it appears in the ROS2 topic list.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("svea_ina238", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x44);
	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', 0.001f, 0.000001f, 0.1f, "Shunt resistor (Ohm)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int svea_ina238_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = SVEA_INA238;
	BusCLIArguments cli{true, false};
	cli.i2c_address = INA238_SVEA_BASEADDR;
	cli.default_i2c_frequency = 100000;
	cli.support_keep_running = true;
	cli.custom2 = 0;

	while ((ch = cli.getOpt(argc, argv, "r:")) != EOF) {
		switch (ch) {
		case 'r': {
				const float shunt = strtof(cli.optArg(), nullptr);

				if (shunt <= 0.f) {
					PX4_ERR("invalid shunt value");
					return -1;
				}

				const int32_t shunt_nano_ohm = static_cast<int32_t>(shunt * 1e9f);

				if (shunt_nano_ohm <= 0) {
					PX4_ERR("invalid shunt value");
					return -1;
				}

				cli.custom2 = shunt_nano_ohm;
			}
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_POWER_DEVTYPE_INA238);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
