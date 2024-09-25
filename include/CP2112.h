#pragma once
#include <cstdint>
#include "SLABCP2112.h"

class CP2112
{
private:
	HID_SMBUS_DEVICE device;
public:
	int initialize();
	int read(uint8_t address, uint8_t read_register, uint8_t* data, std::size_t buffer_length, std::size_t num_bytes);
	int write(uint8_t address, uint8_t* data, std::size_t num_bytes);
	int cleanup();
};

