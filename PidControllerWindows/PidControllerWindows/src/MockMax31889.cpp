#include "MockMax31889.h"

int MockMax31889::initialize() {
	return 0;
}

double MockMax31889::temperature() {
	if (index >= max) {
		return simulated_run[max-1];
	}
	return simulated_run[index++];
}

int MockMax31889::cleanup() {
	return 0;
}
