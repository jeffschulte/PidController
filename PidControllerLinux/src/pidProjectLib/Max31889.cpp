#include "Max31889.h"
#include "CP2112.h" // this is leaky for definitions of BYTE and such, not sure if we care
//#include <synchapi.h>
#include <unistd.h>
#include <limits>
#include <iostream>

/// <summary>
/// Opens the i2c device. There is no MAX31889-specific initialization
/// </summary>
/// <returns>0 on success</returns>
int Max31889::initialize()
{
	auto retval = i2c.initialize();
	return retval;
}


/// <summary>
/// Requests and returns a temperature reading.
/// This starts with a request for a temperature reading
/// We loop on checking the status for a bit indicating a temperature reading is ready to be read.
/// We read 2 bytes from the FIFO data register.
/// </summary>
/// <returns>temperature reading in Celcius or positive infinity if things don't work</returns>
double Max31889::temperature()
{
	uint8_t status_buffer[buffer_size];
	uint8_t temperature_buffer[buffer_size];
	uint8_t temperature_request[2] = { temperature_request_register, temperature_request_command };

	i2c.write(address, temperature_request, 2);
	for (auto wait = 0; wait < wait_limit; wait++) {
		i2c.read(address, status_register, status_buffer, buffer_size, 1);
		if (status_buffer[0] & 0x01) {
			// we have a temperature waiting for us
			i2c.read(address, temperature_read_register, temperature_buffer, buffer_size, 2);
			//uint16_t readTemp = (uint16_t(temperature_buffer[0]) << 8) + uint16_t(temperature_buffer[1]);
			//printf("readTemp %d\n", readTemp);
			//printf("read temp %d\n", uint16_t((temperature_buffer[0] << 8) + temperature_buffer[1]));
			return ((temperature_buffer[0] << 8) + temperature_buffer[1]) * 0.005;
			//return float(readTemp) * 0.005;
		}
		sleep(3);
	}
	return std::numeric_limits<double>::infinity();;
}

/// <summary>
/// Closes the i2c device. There is no MAX31889-specific cleanup.
/// </summary>
/// <returns>0 on success</returns>
int Max31889::cleanup()
{
	auto retval = i2c.cleanup();
	return 0;
}
