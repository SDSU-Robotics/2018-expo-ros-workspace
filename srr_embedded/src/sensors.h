#pragma once

#include "mcp3008Spi.h"
#include <pigpiod_if2.h>

class ADCsensor
{
public:
	void init(mcp3008Spi* adc, int channel);
	unsigned int getValue();
	
private:
	mcp3008Spi* adc_;
	int channel_;
};