// MAX31889viaCP2112.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "Max31889.h"
// #include "MockMax31889.h"
#include <synchapi.h>
#include <iomanip>
//import std;

/// <summary>
/// Sample program that uses the CP2112EK as a USB to i2c bridge to talk to a MAX31889 thermometer.
/// </summary>
/// <remarks>
/// This main (and whole project) will not compile without the <c>/std:c++latest</c> flag and the "Build ISO C++23 Standard Library Modules" in visual studio 2022.
/// </remarks>
/// <returns>0 for success</returns>
int myMain() {
	const auto default_precision{ std::cout.precision() };
	Max31889 thermometer{};
	thermometer.initialize();
	for (int i = 0; i < 10; i++) {
		auto temperature = thermometer.temperature();
		std::cout << "Recording temperature: " << std::setprecision(4) << temperature << " degrees Celcius." << std::endl;
		Sleep(2000);
	}
	std::cout << std::setprecision(default_precision);
	thermometer.cleanup();
	return 0;
}
