#pragma once

#include "mcp3008Spi.h"

class ADCsensor
{
public:
	void init(mcp3008Spi* adc, int channel);
	double getValue();
	
private:
	mcp3008Spi* adc_;
	int channel_;
};