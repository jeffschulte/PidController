#pragma once
#include "CP2112.h"
class Max31889
{
private:
	CP2112 i2c{};
	uint8_t address = 0xA0;
	static const int buffer_size = 61; // this is a cp2112 constraint
	static const uint8_t status_register = 0x00;
	static const uint8_t temperature_request_register = 0x14;
	static const uint8_t temperature_request_command = 0b11000001;  // those two msb are reserved and must be HIGH. The lsb is our actual request bit
	static const uint8_t temperature_read_register = 0x08;
	int wait_limit = 10;

public:
	int initialize();
	double temperature();
	int cleanup();
};

